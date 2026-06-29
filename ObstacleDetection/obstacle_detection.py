import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import numpy as np
from utils.ros_logger import apply_log_level

class LidarObstacleDetector(Node):
    """
    LiDAR topic'inden araç önündeki engelleri tespit eder.
    scan_topic parametresiyle topic adı değiştirilebilir:
      Gerçek LiDAR : /scan  (varsayılan)
      Gazebo        : /lidar
    Yayınlanan topic'ler:
        /obstacle_detected  (Bool)    — engel var/yok
        /obstacle_distance  (Float32) — en yakın engel mesafesi (m)
        /obstacle_side      (Bool)    — True=sağ taraf, False=sol taraf
        /barrier/left_distance      (Float32) — sol bariyer mesafesi (m)
        /barrier/right_distance     (Float32) — sağ bariyer mesafesi (m)
        /barrier/min_distance       (Float32) — LiDAR'daki en yakın geçerli mesafe (m)
        /barrier/lateral_correction (Float32) — şerit sapmasına eklenecek düzeltme
        /barrier/safety_stop        (Bool)    — çok yakın bariyer için sert duruş
    """
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_threshold', 5.0)
        self.declare_parameter('obstacle_min_distance', 1.20)
        self.declare_parameter('front_angle_range_deg', 10.0)
        self.declare_parameter('obstacle_min_points', 6)
        self.declare_parameter('obstacle_confirm_frames', 3)
        self.declare_parameter('obstacle_clear_frames', 4)
        self.declare_parameter('barrier_enabled', True)
        self.declare_parameter('barrier_safe_distance', 0.9)
        self.declare_parameter('barrier_influence_distance', 1.8)
        self.declare_parameter('barrier_correction_gain', 0.35)
        self.declare_parameter('barrier_max_correction', 0.35)
        self.declare_parameter('barrier_hard_stop_distance', 0.12)
        self.declare_parameter('barrier_side_min_angle_deg', 35.0)
        self.declare_parameter('barrier_side_max_angle_deg', 115.0)

        scan_topic = self.get_parameter('scan_topic').value
        self.OBSTACLE_THRESHOLD = self.get_parameter('obstacle_threshold').value
        self.OBSTACLE_MIN_DISTANCE = float(self.get_parameter('obstacle_min_distance').value)
        self.FRONT_ANGLE_RANGE_DEG = self.get_parameter('front_angle_range_deg').value
        self.OBSTACLE_MIN_POINTS = int(self.get_parameter('obstacle_min_points').value)
        self.OBSTACLE_CONFIRM_FRAMES = int(self.get_parameter('obstacle_confirm_frames').value)
        self.OBSTACLE_CLEAR_FRAMES = int(self.get_parameter('obstacle_clear_frames').value)
        self.BARRIER_ENABLED = bool(self.get_parameter('barrier_enabled').value)
        self.BARRIER_SAFE_DISTANCE = float(self.get_parameter('barrier_safe_distance').value)
        self.BARRIER_INFLUENCE_DISTANCE = float(self.get_parameter('barrier_influence_distance').value)
        self.BARRIER_CORRECTION_GAIN = float(self.get_parameter('barrier_correction_gain').value)
        self.BARRIER_MAX_CORRECTION = float(self.get_parameter('barrier_max_correction').value)
        self.BARRIER_HARD_STOP_DISTANCE = float(self.get_parameter('barrier_hard_stop_distance').value)
        self.BARRIER_SIDE_MIN_ANGLE = np.deg2rad(float(self.get_parameter('barrier_side_min_angle_deg').value))
        self.BARRIER_SIDE_MAX_ANGLE = np.deg2rad(float(self.get_parameter('barrier_side_max_angle_deg').value))

        self.scan_subscriber = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        # Engelin sol/sağ tarafını yayınlar: True = sağda, False = solda
        self.obstacle_side_pub = self.create_publisher(Bool, '/obstacle_side', 10)
        self.left_barrier_distance_pub = self.create_publisher(Float32, '/barrier/left_distance', 10)
        self.right_barrier_distance_pub = self.create_publisher(Float32, '/barrier/right_distance', 10)
        self.min_barrier_distance_pub = self.create_publisher(Float32, '/barrier/min_distance', 10)
        self.barrier_correction_pub = self.create_publisher(Float32, '/barrier/lateral_correction', 10)
        self.barrier_safety_stop_pub = self.create_publisher(Bool, '/barrier/safety_stop', 10)

        # Durum değişikliği loglaması için önceki durum takibi
        self._prev_obstacle_detected = False
        self._obstacle_confirm_count = 0
        self._obstacle_clear_count = 0
        self._obstacle_active = False
        self._last_obstacle_distance = float('inf')
        self._last_obstacle_side = False

        self.get_logger().info(
            f'LiDAR Engel Dedektörü başlatıldı | '
            f'scan_topic={scan_topic} '
            f'eşik={self.OBSTACLE_THRESHOLD}m ön açı=±{self.FRONT_ANGLE_RANGE_DEG/2:.0f}° '
            f'min_nokta={self.OBSTACLE_MIN_POINTS} confirm={self.OBSTACLE_CONFIRM_FRAMES}'
        )
        apply_log_level(self)

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        if msg.angle_increment == 0.0:
            self.get_logger().warn("Geçersiz angle_increment değeri.")
            return

        ranges = np.asarray(msg.ranges, dtype=float)
        indices = np.arange(ranges.size, dtype=float)
        angles = msg.angle_min + indices * msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        half_front_range = np.deg2rad(self.FRONT_ANGLE_RANGE_DEG) / 2.0
        front_mask = np.abs(angles) <= half_front_range

        scan_valid_mask = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
        )
        obstacle_valid_mask = scan_valid_mask & (ranges >= self.OBSTACLE_MIN_DISTANCE)
        valid_mask = front_mask & obstacle_valid_mask
        front_ranges = ranges[valid_mask]

        left_barrier, right_barrier = self._side_barrier_distances(ranges, angles, scan_valid_mask)
        scan_min_distance = self._min_or_inf(ranges[scan_valid_mask])
        correction = self._barrier_correction(left_barrier, right_barrier) if self.BARRIER_ENABLED else 0.0
        safety_stop = scan_min_distance <= self.BARRIER_HARD_STOP_DISTANCE
        self.publish_barrier_status(left_barrier, right_barrier, scan_min_distance, correction, safety_stop)

        if front_ranges.size == 0:
            obstacle_detected = self._confirmed_obstacle(
                False,
                float('inf'),
                False,
            )
            publish_distance = (
                self._last_obstacle_distance
                if obstacle_detected
                else float('inf')
            )
            publish_side = self._last_obstacle_side if obstacle_detected else False
            self._log_state_change(obstacle_detected, publish_distance, publish_side)
            self.publish_obstacle_status(obstacle_detected, publish_distance, publish_side)
            return

        obstacle_mask = valid_mask & (ranges <= self.OBSTACLE_THRESHOLD)
        obstacle_point_count = int(np.count_nonzero(obstacle_mask))
        candidate_detected = obstacle_point_count >= self.OBSTACLE_MIN_POINTS

        if np.any(obstacle_mask):
            left_min = self._min_or_inf(ranges[obstacle_mask & (angles > 0.0)])
            right_min = self._min_or_inf(ranges[obstacle_mask & (angles <= 0.0)])
            min_dist = float(np.min(ranges[obstacle_mask]))
        else:
            left_min = self._min_or_inf(ranges[valid_mask & (angles > 0.0)])
            right_min = self._min_or_inf(ranges[valid_mask & (angles <= 0.0)])
            min_dist = float(np.min(front_ranges))

        obstacle_is_right = right_min <= left_min

        obstacle_detected = self._confirmed_obstacle(
            candidate_detected,
            min_dist,
            obstacle_is_right,
        )
        publish_distance = (
            self._last_obstacle_distance
            if obstacle_detected
            else float('inf')
        )
        publish_side = self._last_obstacle_side if obstacle_detected else obstacle_is_right

        self._log_state_change(obstacle_detected, publish_distance, publish_side)
        self.publish_obstacle_status(obstacle_detected, publish_distance, publish_side)

    def _confirmed_obstacle(self, candidate_detected: bool, distance: float, is_right: bool):
        if candidate_detected:
            self._obstacle_confirm_count += 1
            self._obstacle_clear_count = 0
            self._last_obstacle_distance = distance
            self._last_obstacle_side = is_right
            if self._obstacle_confirm_count >= self.OBSTACLE_CONFIRM_FRAMES:
                self._obstacle_active = True
        else:
            self._obstacle_clear_count += 1
            if self._obstacle_clear_count >= self.OBSTACLE_CLEAR_FRAMES:
                self._obstacle_confirm_count = 0
                self._obstacle_active = False
                self._last_obstacle_distance = float('inf')

        return self._obstacle_active

    @staticmethod
    def _min_or_inf(values):
        return float(np.min(values)) if values.size > 0 else float('inf')

    def _side_barrier_distances(self, ranges, angles, scan_valid_mask):
        left_mask = (
            scan_valid_mask
            & (angles >= self.BARRIER_SIDE_MIN_ANGLE)
            & (angles <= self.BARRIER_SIDE_MAX_ANGLE)
        )
        right_mask = (
            scan_valid_mask
            & (angles <= -self.BARRIER_SIDE_MIN_ANGLE)
            & (angles >= -self.BARRIER_SIDE_MAX_ANGLE)
        )
        return self._min_or_inf(ranges[left_mask]), self._min_or_inf(ranges[right_mask])

    def _barrier_correction(self, left_distance, right_distance):
        """
        Pozitif düzeltme: sol bariyerden uzaklaşmak için sağa yönlendirir.
        Negatif düzeltme: sağ bariyerden uzaklaşmak için sola yönlendirir.
        """
        correction = 0.0
        if left_distance < self.BARRIER_INFLUENCE_DISTANCE:
            correction += (self.BARRIER_SAFE_DISTANCE - left_distance) * self.BARRIER_CORRECTION_GAIN
        if right_distance < self.BARRIER_INFLUENCE_DISTANCE:
            correction -= (self.BARRIER_SAFE_DISTANCE - right_distance) * self.BARRIER_CORRECTION_GAIN
        return float(np.clip(correction, -self.BARRIER_MAX_CORRECTION, self.BARRIER_MAX_CORRECTION))

    def _log_state_change(self, detected: bool, distance: float, is_right: bool):
        """Durum değiştiğinde INFO, sürekli aynı durumda DEBUG log yazar."""
        side_str = 'SAĞ' if is_right else 'SOL'
        if detected != self._prev_obstacle_detected:
            if detected:
                self.get_logger().info(
                    f'🚨 Engel GİRDİ: {distance:.2f}m ({side_str} taraf)')
            else:
                self.get_logger().info('✅ Engel temizlendi.')
            self._prev_obstacle_detected = detected
        elif detected:
            self.get_logger().debug(
                f'Engel: {distance:.2f}m ({side_str})')

    def publish_obstacle_status(self, detected: bool, distance: float, is_right: bool):
        detected_msg = Bool()
        detected_msg.data = bool(detected)
        self.obstacle_detected_pub.publish(detected_msg)

        distance_msg = Float32()
        distance_msg.data = float(distance) if detected else -1.0
        self.obstacle_distance_pub.publish(distance_msg)

        side_msg = Bool()
        side_msg.data = bool(is_right)
        self.obstacle_side_pub.publish(side_msg)

    def publish_barrier_status(
        self,
        left_distance: float,
        right_distance: float,
        min_distance: float,
        correction: float,
        safety_stop: bool,
    ):
        left_msg = Float32()
        left_msg.data = float(left_distance) if np.isfinite(left_distance) else -1.0
        self.left_barrier_distance_pub.publish(left_msg)

        right_msg = Float32()
        right_msg.data = float(right_distance) if np.isfinite(right_distance) else -1.0
        self.right_barrier_distance_pub.publish(right_msg)

        min_msg = Float32()
        min_msg.data = float(min_distance) if np.isfinite(min_distance) else -1.0
        self.min_barrier_distance_pub.publish(min_msg)

        correction_msg = Float32()
        correction_msg.data = float(correction)
        self.barrier_correction_pub.publish(correction_msg)

        safety_msg = Bool()
        safety_msg.data = bool(safety_stop)
        self.barrier_safety_stop_pub.publish(safety_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
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
