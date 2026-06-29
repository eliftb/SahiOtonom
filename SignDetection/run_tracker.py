import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from collections import defaultdict, deque, Counter
import json
import os
import pathlib
from utils.ros_logger import apply_log_level
from utils.config import is_cv_display_enabled

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')
        self.br = CvBridge()

        self.declare_parameter('model_name', 'UltraConservative_BEST_mAP0.9248_20250801_115028.pt')
        
        # history_length: Bir nesnenin sınıfını belirlemek için kaç kare geriye bakılacağı.
        self.declare_parameter('history_length', 10)
        # min_confidence_frames: history_length içinde bir sınıfın en az kaç kere
        # görülmesi gerektiği ki "stabil" olarak kabul edilsin.
        self.declare_parameter('min_confidence_frames', 4)
        self.declare_parameter('left_turn_fallback_enabled', True)
        self.declare_parameter('left_turn_confirm_frames', 3)

        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.history_length = self.get_parameter('history_length').get_parameter_value().integer_value
        self.min_confidence_frames = self.get_parameter('min_confidence_frames').get_parameter_value().integer_value
        self.left_turn_fallback_enabled = bool(
            self.get_parameter('left_turn_fallback_enabled').value)
        self.left_turn_confirm_frames = max(
            1, int(self.get_parameter('left_turn_confirm_frames').value))
        self.left_turn_detection_streak = 0

        # Her bir takip ID'sinin sınıf geçmişini tutmak için deque kullanıyoruz.
        # deque, maxlen parametresi sayesinde otomatik olarak eski verileri atar.
        self.track_class_history = defaultdict(lambda: deque(maxlen=self.history_length))
        
        try:
            model_path = str(pathlib.Path(__file__).parent / model_name)
            self.model = YOLO(model_path)
            self.get_logger().info(f'✅ Levha modeli yüklendi: {model_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Levha modeli yüklenemedi: {e}')
            raise RuntimeError(f'Model yüklenemedi: {model_path}') from e
            
        self.subscription = self.create_subscription(Image, '/zed2i_rgb/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/sign_detection/output', 10)
        self.class_publisher = self.create_publisher(String, '/sign_detection/class', 10)
        self.event_publisher = self.create_publisher(String, '/sign_detection/events', 10)

        self._cv_display = is_cv_display_enabled()
        self._request_shutdown = False
        self.get_logger().info(
            f'✅ Gelişmiş Levha Tespit Düğümü Başlatıldı. '
            f'Ekran: {"AÇIK" if self._cv_display else "KAPALI (headless)"}'
        )
        apply_log_level(self)

    def image_callback(self, msg):
        try:
            # Görüntüyü ROS'tan alıp OpenCV formatına çeviriyoruz (BGR)
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLO ile takip et (henüz çizim yapma)
            results = self.model.track(frame, persist=True, conf=0.4, iou=0.5, verbose=False)
            
            # Geliştirilmiş fonksiyonumuzla sonuçları işle, stabilize et ve çiz
            annotated_frame, stable_class_names, stable_detections = (
                self.draw_stabilized_annotations(results[0], frame.copy())
            )

            # Eğitim veri setinde bulunmayan üçgen "sola dönüş" levhasını
            # simülasyondaki şekli ve ok yönü üzerinden ayrıca algıla.
            left_turn_box = None
            if self.left_turn_fallback_enabled:
                left_turn_box = self.detect_left_turn_warning(frame)

            if left_turn_box is None:
                self.left_turn_detection_streak = 0
            else:
                self.left_turn_detection_streak += 1

            left_turn_already_detected = any(
                name in {'sola-don', 'sola-mecburi-yon'}
                for name in stable_class_names
            )
            if (
                left_turn_box is not None
                and not left_turn_already_detected
                and self.left_turn_detection_streak
                >= self.left_turn_confirm_frames
            ):
                x1, y1, x2, y2 = left_turn_box
                frame_height, frame_width = frame.shape[:2]
                box_width = max(x2 - x1, 1)
                box_height = max(y2 - y1, 1)
                frame_area = max(float(frame_height * frame_width), 1.0)
                stable_class_names.append('sola-don')
                stable_detections.append({
                    'class': 'sola-don',
                    'track_id': -100,
                    'area_ratio': round(
                        float(box_width * box_height) / frame_area, 6),
                    'center_x': round(
                        float(x1 + x2) / (2.0 * max(frame_width, 1)), 4),
                    'center_y': round(
                        float(y1 + y2) / (2.0 * max(frame_height, 1)), 4),
                })
                self._draw_left_turn_annotation(
                    annotated_frame, left_turn_box)
            
            # Sonucu ekranda göster (CV_DISPLAY=true ise)
            if self._cv_display:
                cv2.imshow("Sign Detection (Stabilized & Correct Color)", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._request_shutdown = True
                    return

            # Sonucu ROS'ta yayınla
            self.publisher.publish(self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8"))
            self.class_publisher.publish(
                String(data=','.join(stable_class_names) if stable_class_names else 'none'))
            self.event_publisher.publish(
                String(data=json.dumps(stable_detections, separators=(',', ':'))))
        except Exception as e:
            self.get_logger().error(f'Levha Tespit Callback Hatası: {e}')
            import traceback
            traceback.print_exc()

    def draw_stabilized_annotations(self, result, frame):
        """
        Tespit sonuçlarını alır, sınıflandırmayı stabilize eder ve
        sonuçları manuel olarak OpenCV ile görüntü üzerine çizer.
        """
        # Eğer takip edilen bir nesne yoksa, orijinal görüntüyü direkt döndür.
        if not hasattr(result.boxes, 'id') or result.boxes.id is None:
            return frame, [], []

        track_ids = result.boxes.id.int().cpu().tolist()
        boxes = result.boxes.xywh.cpu()
        class_ids = result.boxes.cls.int().cpu().tolist()
        stable_class_names = []
        stable_detections = []
        frame_height, frame_width = frame.shape[:2]
        frame_area = max(float(frame_height * frame_width), 1.0)

        # Her bir tespit edilen nesne için döngü
        for box, track_id, class_id in zip(boxes, track_ids, class_ids):
            # 1. Bu nesnenin sınıflandırma geçmişini güncelle
            self.track_class_history[track_id].append(class_id)
            
            # 2. Geçmişi analiz et
            history = self.track_class_history[track_id]
            if not history:
                continue

            # 3. Geçmişteki en yaygın sınıfı ve görülme sayısını bul
            most_common_class, confidence_count = Counter(history).most_common(1)[0]

            # 4. Sadece yeterince emin olduğumuz (stabil) sınıfları çiz
            if confidence_count >= self.min_confidence_frames:
                x, y, w, h = box
                x1, y1 = int(x - w / 2), int(y - h / 2)
                x2, y2 = int(x + w / 2), int(y + h / 2)
                
                # Stabil sınıfın adını modelden al
                stable_class_name = self.model.names[most_common_class]
                stable_class_names.append(stable_class_name)
                stable_detections.append({
                    'class': stable_class_name,
                    'track_id': int(track_id),
                    'area_ratio': round(float(w * h) / frame_area, 6),
                    'center_x': round(float(x) / max(frame_width, 1), 4),
                    'center_y': round(float(y) / max(frame_height, 1), 4),
                })
                label = f"ID:{track_id} {stable_class_name}"

                # RENK DÜZELTMESİ: OpenCV BGR formatında renk bekle
                color = (255, 165, 0) 

                # Dikdörtgeni çiz
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # Etiket için daha okunaklı bir arka plan çiz
                (w_text, h_text), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, y1 - 20), (x1 + w_text, y1), color, -1)
                cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) # Yazı Beyaz
        
        return frame, stable_class_names, stable_detections

    @staticmethod
    def detect_left_turn_warning(frame):
        """
        Simülasyondaki kırmızı üçgen ve sola kıvrılan siyah oku bulur.

        Dıştaki üçgen kontrolü bariyerlerin kırmızı parçalarını, okun üst
        bölümünün sola kayması ise diğer üçgen levhaları eler.
        """
        if frame is None or frame.size == 0:
            return None

        frame_height, frame_width = frame.shape[:2]
        frame_area = max(float(frame_height * frame_width), 1.0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, (0, 100, 12), (12, 255, 255)),
            cv2.inRange(hsv, (168, 100, 12), (179, 255, 255)),
        )

        # Yol ve bariyerlerin yoğun olduğu alt bölge levha aramasına dahil değil.
        red_mask[int(frame_height * 0.70):, :] = 0
        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
        )

        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < max(120.0, frame_area * 0.00012):
                continue

            x, y, width, height = cv2.boundingRect(contour)
            if width < 18 or height < 18:
                continue

            aspect_ratio = width / max(float(height), 1.0)
            extent = area / max(float(width * height), 1.0)
            perimeter = cv2.arcLength(contour, True)
            polygon = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            if not (
                0.65 <= aspect_ratio <= 1.35
                and 3 <= len(polygon) <= 5
                and 0.25 <= extent <= 0.75
            ):
                continue

            shifted_contour = contour - np.array([[[x, y]]])
            triangle_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.drawContours(
                triangle_mask, [shifted_contour], -1, 255, thickness=-1)

            erosion_size = max(3, int(round(min(width, height) * 0.09)))
            if erosion_size % 2 == 0:
                erosion_size += 1
            interior_mask = cv2.erode(
                triangle_mask,
                np.ones((erosion_size, erosion_size), dtype=np.uint8),
            )

            roi = frame[y:y + height, x:x + width]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            black_mask = cv2.inRange(gray, 0, 85)
            black_mask[red_mask[y:y + height, x:x + width] > 0] = 0
            black_mask = cv2.bitwise_and(black_mask, interior_mask)
            black_mask = cv2.morphologyEx(
                black_mask,
                cv2.MORPH_OPEN,
                np.ones((3, 3), dtype=np.uint8),
            )

            arrow_contours, _ = cv2.findContours(
                black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not arrow_contours:
                continue

            arrow = max(arrow_contours, key=cv2.contourArea)
            arrow_area = cv2.contourArea(arrow)
            if arrow_area / max(float(width * height), 1.0) < 0.015:
                continue

            arrow_mask = np.zeros_like(black_mask)
            cv2.drawContours(arrow_mask, [arrow], -1, 255, thickness=-1)
            arrow_y, arrow_x = np.where(arrow_mask > 0)
            upper = (
                (arrow_y >= height * 0.35)
                & (arrow_y < height * 0.57)
            )
            lower = (
                (arrow_y >= height * 0.55)
                & (arrow_y < height * 0.80)
            )
            if not upper.any() or not lower.any():
                continue

            # Sola dönen okun üst/ok ucu bölümü, gövdesinden daha soldadır.
            left_shift = float(arrow_x[lower].mean() - arrow_x[upper].mean())
            if left_shift < max(3.0, width * 0.04):
                continue

            candidates.append((area, (x, y, x + width, y + height)))

        if not candidates:
            return None
        return max(candidates, key=lambda item: item[0])[1]

    @staticmethod
    def _draw_left_turn_annotation(frame, box):
        x1, y1, x2, y2 = box
        color = (0, 220, 0)
        label = "sola-don"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (text_width, text_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        label_top = max(y1 - text_height - 8, 0)
        cv2.rectangle(
            frame,
            (x1, label_top),
            (x1 + text_width + 6, y1),
            color,
            thickness=-1,
        )
        cv2.putText(
            frame,
            label,
            (x1 + 3, max(y1 - 5, text_height + 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()
        self.get_logger().info("Levha Tespit Düğümü kapatıldı.")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SignDetectionNode()
    except RuntimeError as e:
        rclpy.shutdown()
        return
    try:
        while rclpy.ok() and not node._request_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
