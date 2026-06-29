"""
Gazebo Lane Detection (OpenCV tabanlı)
======================================
tusimple_18.pt modeli olmadan çalışır.
Gazebo'daki beyaz/sarı şerit çizgilerini HSV renk filtresi +
Hough Lines ile tespit eder; mevcut node'larla aynı topic'leri yayınlar.

Yayınlar:
    /lane/lateral_deviation     (std_msgs/Float32)  — [-1, +1]
    /lane/heading_deviation     (std_msgs/Float32)  — şeride göre açı sapması
    /lane/intersection_direction (std_msgs/Int32)   — 0: yok

Parametreler (--ros-args -p key:=value):
    camera_topic   : Kaynak görüntü topic'i (varsayılan: /zed2i_rgb/image_raw)
    white_only     : true → sadece beyaz, false → beyaz+sarı (varsayılan: true)
    roi_ratio      : Görüntünün kaçta birini ROI olarak al — alttan (varsayılan: 0.45)
    history_size   : Sapma yumuşatma geçmişi (varsayılan: 5)
    edge_ignore_ratio: Görüntü kenarlarını yok sayma oranı (varsayılan: 0.12)
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from utils.ros_logger import apply_log_level


class GazeboLaneDetection(Node):

    def __init__(self):
        super().__init__('gazebo_lane_detection')

        self.declare_parameter('camera_topic', '/zed2i_rgb/image_raw')
        self.declare_parameter('white_only',   True)
        self.declare_parameter('roi_ratio',    0.45)
        self.declare_parameter('history_size', 5)
        self.declare_parameter('edge_ignore_ratio', 0.12)
        self.declare_parameter('lane_half_width_ratio', 0.27)
        self.declare_parameter('min_lane_pixels', 80)
        self.declare_parameter('intersection_history_size', 5)
        self.declare_parameter('intersection_min_frames', 3)
        self.declare_parameter('intersection_min_line_ratio', 0.20)

        cam_topic       = self.get_parameter('camera_topic').value
        self._white_only = self.get_parameter('white_only').value
        self._roi_ratio  = float(self.get_parameter('roi_ratio').value)
        history_size     = int(self.get_parameter('history_size').value)
        self._edge_ignore_ratio = float(self.get_parameter('edge_ignore_ratio').value)
        self._lane_half_width_ratio = float(self.get_parameter('lane_half_width_ratio').value)
        self._min_lane_pixels = int(self.get_parameter('min_lane_pixels').value)
        intersection_history_size = int(
            self.get_parameter('intersection_history_size').value)
        self._intersection_min_frames = int(
            self.get_parameter('intersection_min_frames').value)
        self._intersection_min_line_ratio = float(
            self.get_parameter('intersection_min_line_ratio').value)

        self.bridge   = CvBridge()
        self._history = []
        self._heading_history = []
        self._history_size = history_size
        self._intersection_history = []
        self._intersection_history_size = max(3, intersection_history_size)

        self.sub = self.create_subscription(
            Image, cam_topic, self._image_callback, 10)

        self.lat_pub = self.create_publisher(Float32, '/lane/lateral_deviation', 10)
        self.heading_pub = self.create_publisher(
            Float32, '/lane/heading_deviation', 10)
        self.int_pub = self.create_publisher(Int32,   '/lane/intersection_direction', 10)

        self.get_logger().info(
            f'Gazebo Lane Detection başlatıldı.\n'
            f'  Kaynak  : {cam_topic}\n'
            f'  Mod     : {"Sadece beyaz" if self._white_only else "Beyaz + sarı"}\n'
            f'  ROI     : alt {self._roi_ratio*100:.0f}%'
        )
        apply_log_level(self)

    # ------------------------------------------------------------------ #

    def _lane_mask(self, frame_bgr):
        """Beyaz (ve isteğe bağlı sarı) şerit piksellerini maskeler."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Beyaz: düşük doygunluk, yüksek parlaklık
        white_mask = cv2.inRange(hsv,
                                 np.array([0,   0,  160]),
                                 np.array([180, 60, 255]))

        if self._white_only:
            return white_mask

        # Sarı
        yellow_mask = cv2.inRange(hsv,
                                  np.array([15, 80,  80]),
                                  np.array([40, 255, 255]))
        return cv2.bitwise_or(white_mask, yellow_mask)

    def _compute_lane_geometry(self, mask, width):
        """
        Şerit maskesinden yanal sapma ve yön sapması hesaplar.
        Kaldırım/kenar çizgilerinin baskın çıkmasını azaltmak için görüntü kenarları
        yok sayılır ve sol/sağ şeritler merkez çevresindeki histogramdan seçilir.
        """
        h, w = mask.shape
        cx = w // 2

        # ROI: alt bölge
        roi_start = int(h * (1 - self._roi_ratio))
        roi = mask[roi_start:, :]

        # Morfolojik temizlik
        kernel = np.ones((5, 5), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN,  kernel)

        edge = int(w * self._edge_ignore_ratio)
        roi[:, :edge] = 0
        roi[:, w - edge:] = 0

        expected_half_width = w * self._lane_half_width_ratio
        roi_height = roi.shape[0]
        near_center, near_source = self._lane_center_in_band(
            roi,
            roi_height // 2,
            roi_height,
            cx,
            edge,
            expected_half_width,
        )
        if near_center is None:
            return None, None

        far_center, far_source = self._lane_center_in_band(
            roi,
            int(roi_height * 0.08),
            int(roi_height * 0.48),
            cx,
            edge,
            expected_half_width,
        )

        heading_deviation = None
        if (
            far_center is not None
            and (
                near_source == far_source
                or near_source == 'both'
                or far_source == 'both'
            )
        ):
            heading_deviation = float(np.clip(
                (far_center - near_center) / (w / 2.0),
                -1.0,
                1.0,
            ))

        lateral_deviation = float(np.clip(
            (near_center - cx) / (w / 2.0),
            -1.0,
            1.0,
        ))
        return lateral_deviation, heading_deviation

    def _lane_center_in_band(
        self,
        roi,
        start_y,
        end_y,
        cx,
        edge,
        expected_half_width,
    ):
        band = roi[max(0, start_y):min(roi.shape[0], end_y), :]
        if band.size == 0:
            return None, None

        histogram = np.sum(band > 0, axis=0).astype(float)
        if float(np.sum(histogram)) < self._min_lane_pixels:
            return None, None

        w = roi.shape[1]
        min_sep = int(w * 0.08)
        left_end = max(edge, cx - min_sep)
        right_start = min(w - edge, cx + min_sep)
        left_x = self._weighted_peak(histogram[edge:left_end], edge)
        right_x = self._weighted_peak(
            histogram[right_start:w - edge],
            right_start,
        )

        if left_x is not None and right_x is not None:
            return (left_x + right_x) / 2.0, 'both'
        if left_x is not None:
            return left_x + expected_half_width, 'left'
        if right_x is not None:
            return right_x - expected_half_width, 'right'
        return None, None

    def _weighted_peak(self, histogram, offset):
        """Histogramdaki en güçlü şerit kolonunun ağırlıklı merkezini döndürür."""
        if histogram.size == 0 or float(np.sum(histogram)) < self._min_lane_pixels:
            return None

        peak = int(np.argmax(histogram))
        if histogram[peak] <= 0:
            return None

        window = 35
        start = max(0, peak - window)
        end = min(histogram.size, peak + window + 1)
        weights = histogram[start:end]
        if float(np.sum(weights)) < self._min_lane_pixels:
            return None

        xs = np.arange(start, end, dtype=float) + offset
        return float(np.average(xs, weights=weights))

    def _smooth(self, value):
        self._history.append(value)
        if len(self._history) > self._history_size:
            self._history.pop(0)
        n = len(self._history)
        if n >= 3:
            weights = np.linspace(0.5, 1.0, n)
            return float(np.average(self._history, weights=weights))
        return float(np.mean(self._history))

    def _smooth_heading(self, value):
        self._heading_history.append(float(value))
        if len(self._heading_history) > self._history_size:
            self._heading_history.pop(0)
        weights = np.linspace(0.5, 1.0, len(self._heading_history))
        return float(np.average(self._heading_history, weights=weights))

    def _detect_intersection_direction(self, mask):
        """0=yok, 1=sol, 2=sag, 4=iki yon acik."""
        height, width = mask.shape
        roi = mask[int(height * 0.42):int(height * 0.82), :]
        if roi.size == 0:
            return 0

        kernel = np.ones((5, 5), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        edges = cv2.Canny(roi, 50, 150)
        min_line_length = max(30, int(width * self._intersection_min_line_ratio))
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=35,
            minLineLength=min_line_length,
            maxLineGap=25,
        )

        horizontal_length = 0.0
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
                if angle <= 15.0 or angle >= 165.0:
                    horizontal_length += float(np.hypot(x2 - x1, y2 - y1))

        if horizontal_length < width * 0.35:
            raw_direction = 0
        else:
            lower = mask[int(height * 0.35):, :]
            left_score = float(np.count_nonzero(lower[:, :width // 2]))
            right_score = float(np.count_nonzero(lower[:, width // 2:]))
            min_side_score = max(80.0, lower.size * 0.002)
            left_open = left_score >= min_side_score
            right_open = right_score >= min_side_score

            if left_open and right_open:
                raw_direction = 4
            elif left_open:
                raw_direction = 1
            elif right_open:
                raw_direction = 2
            else:
                raw_direction = 4

        self._intersection_history.append(raw_direction)
        if len(self._intersection_history) > self._intersection_history_size:
            self._intersection_history.pop(0)

        if len(self._intersection_history) < self._intersection_min_frames:
            return 0
        recent = self._intersection_history[-self._intersection_min_frames:]
        return raw_direction if raw_direction != 0 and all(
            value == raw_direction for value in recent
        ) else 0

    def _image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'imgmsg_to_cv2 hatası: {e}')
            return

        mask = self._lane_mask(frame)
        deviation, heading_deviation = self._compute_lane_geometry(
            mask, frame.shape[1])

        if deviation is None:
            # Şerit bulunamadı — önceki değeri koru
            if self._history:
                deviation = self._history[-1]
            else:
                deviation = 0.0
            self.get_logger().debug('Şerit bulunamadı, önceki değer kullanılıyor.')
        else:
            deviation = self._smooth(deviation)

        if heading_deviation is None:
            heading_deviation = (
                self._heading_history[-1] * 0.75
                if self._heading_history
                else 0.0
            )
        else:
            heading_deviation = self._smooth_heading(heading_deviation)

        intersection_direction = self._detect_intersection_direction(mask)
        self.lat_pub.publish(Float32(data=deviation))
        self.heading_pub.publish(Float32(data=heading_deviation))
        self.int_pub.publish(Int32(data=intersection_direction))

        self.get_logger().debug(
            f'Sapma: {deviation:.3f} yon={heading_deviation:.3f} '
            f'kavsak={intersection_direction}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboLaneDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
