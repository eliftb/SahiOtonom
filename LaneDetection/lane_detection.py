import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
from collections import deque
import cv2
import torch
import numpy as np

from utils.utils import (
    select_device,
    driving_area_mask, lane_line_mask, show_seg_result
)
from utils.ros_logger import apply_log_level
from utils.config import is_cv_display_enabled

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.br = CvBridge()

        import pathlib
        self.weights = str(pathlib.Path(__file__).parent / 'models' / 'tusimple_18.pt')
        self.device = select_device('0')
        self.half = self.device.type != 'cpu'
        self.img_size = 640

        self.subscription = self.create_subscription(
            Image, '/zed2i_rgb/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/lane/detection_output', 10)
        self.lateral_pub = self.create_publisher(Float32, '/lane/lateral_deviation', 10)
        self.intersection_pub = self.create_publisher(Int32, '/lane/intersection_direction', 10)

        self.previous_deviation = 0.0
        self.max_history_size = 5
        self.deviation_history = deque(maxlen=self.max_history_size)

        self.intersection_history_size = 3
        self.intersection_history = deque(maxlen=self.intersection_history_size)
        self.stable_intersection_count = 0
        self.min_stable_frames = 2

        self._cv_display = is_cv_display_enabled()
        self.load_model()
        self.get_logger().info(
            f'🚦 Lane Detection Node başlatıldı. '
            f'Ekran: {"AÇIK" if self._cv_display else "KAPALI (headless)"}'
        )
        apply_log_level(self)

    def load_model(self):
        self.model = torch.jit.load(self.weights).to(self.device)
        if self.half:
            self.model.half()
        self.model.eval()
        self.get_logger().info('✅ Model yüklendi.')

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        shape = img.shape[:2]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:
            r = min(r, 1.0)
        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        if auto:
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)
        elif scaleFill:
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]
        dw /= 2
        dh /= 2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, ratio, (dw, dh)

    def preprocess_image(self, im0):
        lab = cv2.cvtColor(im0, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        img, _, _ = self.letterbox(enhanced, new_shape=self.img_size, stride=32)
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        return img

    def find_lane_lines(self, lane_mask):
        """
        Şerit çizgilerini tespit eder ve sol/sağ şerit sınırlarını bulur
        """
        height, width = lane_mask.shape
        
        # Alt yarıda odaklan (daha stabil sonuçlar için)
        roi_height_start = int(height * 0.7)  # Alt %30 için %70'den başla
        roi = lane_mask[roi_height_start:, :]
        
        # Morfolojik işlemlerle gürültüyü temizle
        kernel = np.ones((3, 3), np.uint8)
        roi_cleaned = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        roi_cleaned = cv2.morphologyEx(roi_cleaned, cv2.MORPH_OPEN, kernel)
        
        # Konturları bul
        contours, _ = cv2.findContours(roi_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None
        
        # En büyük konturları al (ana şerit çizgileri)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
        
        left_line = None
        right_line = None
        center_x = width / 2
        
        for contour in contours:
            # Konturun merkez x koordinatını hesapla
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                # Sol ve sağ şeridi ayır
                if cx < center_x and left_line is None:
                    left_line = contour
                elif cx > center_x and right_line is None:
                    right_line = contour
        
        return left_line, right_line

    def compute_lateral_deviation(self, lane_mask):
        """
        Geliştirilmiş lateral deviation hesaplama
        Şerit çizgilerinin ortasına göre araç pozisyonunu hesaplar
        """
        height, width = lane_mask.shape
        image_center = width / 2
        
        # Şerit çizgilerini bul
        left_line, right_line = self.find_lane_lines(lane_mask)
        
        lane_center = None
        
        if left_line is not None and right_line is not None:
            # Her iki şerit de varsa, ortalarını hesapla
            left_M = cv2.moments(left_line)
            right_M = cv2.moments(right_line)
            
            if left_M["m00"] != 0 and right_M["m00"] != 0:
                left_cx = left_M["m10"] / left_M["m00"]
                right_cx = right_M["m10"] / right_M["m00"]
                lane_center = (left_cx + right_cx) / 2
                
        elif left_line is not None:
            # Sadece sol şerit varsa, tahmini sağ şeridi hesapla
            left_M = cv2.moments(left_line)
            if left_M["m00"] != 0:
                left_cx = left_M["m10"] / left_M["m00"]
                # Tipik şerit genişliği ~3.5m, piksel cinsinden yaklaşık olarak
                estimated_lane_width = width * 0.4  # Görüntü genişliğinin %40'ı
                estimated_right_cx = left_cx + estimated_lane_width
                lane_center = (left_cx + estimated_right_cx) / 2
                
        elif right_line is not None:
            # Sadece sağ şerit varsa, tahmini sol şeridi hesapla
            right_M = cv2.moments(right_line)
            if right_M["m00"] != 0:
                right_cx = right_M["m10"] / right_M["m00"]
                estimated_lane_width = width * 0.4
                estimated_left_cx = right_cx - estimated_lane_width
                lane_center = (estimated_left_cx + right_cx) / 2
        
        # Eğer hiçbir şerit bulunamazsa
        if lane_center is None:
            # Alt kısımdaki tüm piksellerin ortalamasını al (eski yöntem)
            bottom_quarter = lane_mask[4*height//5:, :]
            indices = np.column_stack(np.where(bottom_quarter > 0))
            
            if indices.size == 0:
                if len(self.deviation_history) > 0:
                    return self.deviation_history[-1]
                return 0.0
            
            lane_center = np.mean(indices[:, 1])
        
        # Lateral deviation hesapla
        deviation = (lane_center - image_center) / (width / 2)
        deviation = float(np.clip(deviation, -1.0, 1.0))
        
        # Geçmiş değerlerle yumuşat (deque maxlen ile otomatik kırpılır)
        self.deviation_history.append(deviation)

        if len(self.deviation_history) >= 3:
            # Son 3 değerin ağırlıklı ortalaması (en son değer daha ağırlıklı)
            weights = np.array([0.2, 0.3, 0.5])
            smooth_deviation = np.average(self.deviation_history[-3:], weights=weights)
        else:
            smooth_deviation = deviation
            
        return float(smooth_deviation)

    def detect_brightness_level(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        return brightness

    def detect_horizontal_lines(self, lane_mask):
        height, width = lane_mask.shape

        roi_start = int(0.5 * height)
        roi_end = int(0.85 * height)
        roi = lane_mask[roi_start:roi_end, :]

        blurred = cv2.GaussianBlur(roi, (5,5), 0)
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                                minLineLength=width // 4, maxLineGap=20)

        has_horizontal = False
        strongest = 0

        if lines is not None:
            horizontal_lines = []
            for line in lines:
                x1,y1,x2,y2 = line[0]
                angle = np.arctan2((y2 - y1), (x2 - x1)) * 180 / np.pi
                if abs(angle) < 15:
                    horizontal_lines.append(line)
                    cv2.line(roi, (x1,y1), (x2,y2), (255,255,255), 2)

            strongest = len(horizontal_lines)
            has_horizontal = strongest > 0

        if self._cv_display:
            cv2.imshow('ROI Edges', edges)
        return has_horizontal, strongest

    def detect_intersection_direction(self, lane_mask, original_image):
        height, width = lane_mask.shape

        brightness = self.detect_brightness_level(original_image)

        has_horizontal_line, horizontal_strength = self.detect_horizontal_lines(lane_mask)

        if not has_horizontal_line:
            intersection_flags = 0
            self.stable_intersection_count = 0
        else:
            left_band = lane_mask[:, :width // 3]
            right_band = lane_mask[:, -width // 3:]

            left_sum = np.sum(left_band)
            right_sum = np.sum(right_band)

            direction_threshold = 1.3

            if left_sum > right_sum * direction_threshold:
                intersection_flags = 1
            elif right_sum > left_sum * direction_threshold:
                intersection_flags = 2
            else:
                intersection_flags = 4

        self.intersection_history.append(intersection_flags)

        if len(self.intersection_history) >= self.min_stable_frames:
            if all(flag == intersection_flags for flag in self.intersection_history[-self.min_stable_frames:]):
                self.stable_intersection_count += 1
            else:
                self.stable_intersection_count = 0

        if self.stable_intersection_count >= self.min_stable_frames:
            final_direction = intersection_flags
        else:
            final_direction = 0

        self.get_logger().debug(
            f'Brightness: {brightness:.1f} | Horizontal: {has_horizontal_line} ({horizontal_strength}) | '
            f'Direction: {final_direction} | Stable: {self.stable_intersection_count}'
        )
        return final_direction

    def image_callback(self, msg):
        try:
            im0s = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = self.preprocess_image(im0s)
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()
            img /= 255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            with torch.no_grad():
                [pred, anchor_grid], seg, ll = self.model(img)

            da_seg_mask = driving_area_mask(seg)
            ll_seg_mask = lane_line_mask(ll)
            ll_seg_mask = cv2.resize(ll_seg_mask, (im0s.shape[1], im0s.shape[0]), interpolation=cv2.INTER_NEAREST)

            im0 = im0s.copy()
            show_seg_result(im0, (da_seg_mask, ll_seg_mask), is_demo=True)

            # Geliştirilmiş lateral deviation hesaplama
            lateral_deviation = self.compute_lateral_deviation(ll_seg_mask)
            self.lateral_pub.publish(Float32(data=lateral_deviation))

            intersection_direction = self.detect_intersection_direction(ll_seg_mask, im0s)
            self.intersection_pub.publish(Int32(data=intersection_direction))

            # Debug bilgilerini göster
            debug_text = f'LatDev: {lateral_deviation:.3f} | Int: {intersection_direction}'
            cv2.putText(im0, debug_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Şerit merkezi gösterimi için
            height, width = ll_seg_mask.shape
            image_center = width // 2
            cv2.line(im0, (image_center, 0), (image_center, height), (255, 0, 0), 2)  # Mavi çizgi = görüntü merkezi

            if self._cv_display:
                cv2.imshow('LANE NODE', im0)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rclpy.shutdown()
                    cv2.destroyAllWindows()
                    return

            self.publisher.publish(self.br.cv2_to_imgmsg(im0, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()