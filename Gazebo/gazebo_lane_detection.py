"""
Gazebo Lane Detection (OpenCV tabanlı)
======================================
tusimple_18.pt modeli olmadan çalışır.
Gazebo'daki beyaz/sarı şerit çizgilerini HSV renk filtresi +
Hough Lines ile tespit eder; mevcut node'larla aynı topic'leri yayınlar.

Yayınlar:
    /lane/lateral_deviation     (std_msgs/Float32)  — [-1, +1]
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

        cam_topic       = self.get_parameter('camera_topic').value
        self._white_only = self.get_parameter('white_only').value
        self._roi_ratio  = float(self.get_parameter('roi_ratio').value)
        history_size     = int(self.get_parameter('history_size').value)
        self._edge_ignore_ratio = float(self.get_parameter('edge_ignore_ratio').value)
        self._lane_half_width_ratio = float(self.get_parameter('lane_half_width_ratio').value)
        self._min_lane_pixels = int(self.get_parameter('min_lane_pixels').value)

        self.bridge   = CvBridge()
        self._history = []
        self._history_size = history_size

        self.sub = self.create_subscription(
            Image, cam_topic, self._image_callback, 10)

        self.lat_pub = self.create_publisher(Float32, '/lane/lateral_deviation', 10)
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

    def _compute_deviation(self, mask, width):
        """
        Şerit maskesinden lateral deviation hesaplar.
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

        histogram = np.sum(roi > 0, axis=0).astype(float)
        if float(np.sum(histogram)) < self._min_lane_pixels:
            return None

        min_sep = int(w * 0.08)
        expected_half_width = w * self._lane_half_width_ratio

        left_region = histogram[edge:max(edge, cx - min_sep)]
        right_region = histogram[min(w - edge, cx + min_sep):w - edge]

        left_x = self._weighted_peak(left_region, edge)
        right_x = self._weighted_peak(right_region, min(w - edge, cx + min_sep))

        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2.0
        elif left_x is not None:
            lane_center = left_x + expected_half_width
        elif right_x is not None:
            lane_center = right_x - expected_half_width
        else:
            return None

        deviation = (lane_center - cx) / (w / 2)
        return float(np.clip(deviation, -1.0, 1.0))

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

    def _image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'imgmsg_to_cv2 hatası: {e}')
            return

        mask = self._lane_mask(frame)
        deviation = self._compute_deviation(mask, frame.shape[1])

        if deviation is None:
            # Şerit bulunamadı — önceki değeri koru
            if self._history:
                deviation = self._history[-1]
            else:
                deviation = 0.0
            self.get_logger().debug('Şerit bulunamadı, önceki değer kullanılıyor.')
        else:
            deviation = self._smooth(deviation)

        self.lat_pub.publish(Float32(data=deviation))
        self.int_pub.publish(Int32(data=0))  # Gazebo'da kavşak tespiti yok

        self.get_logger().debug(f'Sapma: {deviation:.3f}')


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
