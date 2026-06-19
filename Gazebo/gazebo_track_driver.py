"""
Gazebo Track Driver
===================
Alzada X Car icin sade Gazebo surucusu.

Bu node eski Gazebo zincirinin yerine gecer:
    kamera + lidar -> /cmd_vel

Amac:
    - Parkuru serit goruntusune gore takip etmek
    - Bariyerlere yaklasinca sadece direksiyonla uzaklasmak
    - Gercekten cok yakin acil durum disinda durmamak
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import cv2
import numpy as np
import rclpy
import time
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from utils.ros_logger import apply_log_level


class GazeboTrackDriver(Node):
    def __init__(self):
        super().__init__('gazebo_track_driver')

        self.declare_parameter('camera_topic', '/serit_takip_kamerasi/image_raw')
        self.declare_parameter('scan_topic', '/lidar')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('base_speed', 0.16)
        self.declare_parameter('min_speed', 0.09)
        self.declare_parameter('lane_gain', 0.85)
        self.declare_parameter('barrier_gain', 0.45)
        self.declare_parameter('max_angular_z', 0.45)
        self.declare_parameter('left_turn_gain', 1.0)
        self.declare_parameter('right_turn_gain', 1.0)

        self.declare_parameter('roi_ratio', 0.62)
        self.declare_parameter('edge_ignore_ratio', 0.08)
        self.declare_parameter('lane_half_width_ratio', 0.28)
        self.declare_parameter('history_size', 4)

        self.declare_parameter('barrier_safe_distance', 0.85)
        self.declare_parameter('barrier_influence_distance', 1.6)
        self.declare_parameter('front_emergency_distance', 0.50)
        self.declare_parameter('front_slow_distance', 1.2)
        self.declare_parameter('hard_stop_distance', 0.42)
        self.declare_parameter('front_angle_range_deg', 10.0)
        self.declare_parameter('side_min_angle_deg', 30.0)
        self.declare_parameter('side_max_angle_deg', 115.0)
        self.declare_parameter('emergency_turn_speed', 0.0)
        self.declare_parameter('avoidance_hold_duration', 3.0)
        self.declare_parameter('avoidance_speed', 0.10)
        self.declare_parameter('return_hold_duration', 2.0)
        self.declare_parameter('return_speed', 0.08)
        self.declare_parameter('return_turn_ratio', 0.55)
        self.declare_parameter('return_max_angular_z', 1.8)
        self.declare_parameter('return_turn_sign', -1.0)
        self.declare_parameter('return_min_duration', 1.0)
        self.declare_parameter('return_center_tolerance', 0.08)

        camera_topic = self.get_parameter('camera_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.lane_gain = float(self.get_parameter('lane_gain').value)
        self.barrier_gain = float(self.get_parameter('barrier_gain').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.left_turn_gain = float(self.get_parameter('left_turn_gain').value)
        self.right_turn_gain = float(self.get_parameter('right_turn_gain').value)

        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.edge_ignore_ratio = float(self.get_parameter('edge_ignore_ratio').value)
        self.lane_half_width_ratio = float(self.get_parameter('lane_half_width_ratio').value)
        self.history_size = int(self.get_parameter('history_size').value)

        self.barrier_safe_distance = float(self.get_parameter('barrier_safe_distance').value)
        self.barrier_influence_distance = float(self.get_parameter('barrier_influence_distance').value)
        self.front_emergency_distance = float(self.get_parameter('front_emergency_distance').value)
        self.front_slow_distance = float(self.get_parameter('front_slow_distance').value)
        self.hard_stop_distance = float(self.get_parameter('hard_stop_distance').value)
        self.front_half_angle = np.deg2rad(float(self.get_parameter('front_angle_range_deg').value) / 2.0)
        self.side_min_angle = np.deg2rad(float(self.get_parameter('side_min_angle_deg').value))
        self.side_max_angle = np.deg2rad(float(self.get_parameter('side_max_angle_deg').value))
        self.emergency_turn_speed = float(self.get_parameter('emergency_turn_speed').value)
        self.avoidance_hold_duration = float(self.get_parameter('avoidance_hold_duration').value)
        self.avoidance_speed = float(self.get_parameter('avoidance_speed').value)
        self.return_hold_duration = float(self.get_parameter('return_hold_duration').value)
        self.return_speed = float(self.get_parameter('return_speed').value)
        self.return_turn_ratio = float(self.get_parameter('return_turn_ratio').value)
        self.return_max_angular_z = float(self.get_parameter('return_max_angular_z').value)
        self.return_turn_sign = float(self.get_parameter('return_turn_sign').value)
        self.return_min_duration = float(self.get_parameter('return_min_duration').value)
        self.return_center_tolerance = float(self.get_parameter('return_center_tolerance').value)

        self.bridge = CvBridge()
        self.deviation_history = []
        self.last_deviation = 0.0
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.front_distance = float('inf')
        self.nearest_distance = float('inf')
        self.have_image = False
        self._avoidance_dir = 0.0      # kilitli kaçış yönü
        self._avoidance_active = False
        self._avoidance_last_seen = 0.0
        self._return_active = False
        self._return_start = 0.0

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.create_timer(0.1, self.drive_loop)

        self.get_logger().info(
            f'Gazebo Track Driver basladi | camera={camera_topic} scan={scan_topic} cmd={cmd_vel_topic}\n'
            f'  On dur mesafesi  : {self.front_emergency_distance} m\n'
            f'  On yavasla mesaf.: {self.front_slow_distance} m\n'
            f'  Sert min mesafe  : {self.hard_stop_distance} m'
        )
        apply_log_level(self)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Kamera donusum hatasi: {exc}')
            return

        deviation = self.compute_lane_deviation(frame)
        if deviation is None:
            deviation = self.last_deviation
        else:
            self.last_deviation = self.smooth_deviation(deviation)
            deviation = self.last_deviation

        self.have_image = True
        self.get_logger().debug(f'lane_deviation={deviation:.3f}')

    def compute_lane_deviation(self, frame):
        h, w = frame.shape[:2]
        cx = w // 2
        roi_start = int(h * (1.0 - self.roi_ratio))
        roi = frame[roi_start:, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array([0, 0, 145]), np.array([180, 85, 255]))
        yellow = cv2.inRange(hsv, np.array([15, 60, 80]), np.array([45, 255, 255]))
        mask = cv2.bitwise_or(white, yellow)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        edge = int(w * self.edge_ignore_ratio)
        mask[:, :edge] = 0
        mask[:, w - edge:] = 0

        # Alt yari daha onemli: aracin hemen onundeki serit.
        lower = mask[mask.shape[0] // 2:, :]
        histogram = np.sum(lower > 0, axis=0).astype(float)
        if np.sum(histogram) < 60:
            return None

        min_sep = int(w * 0.07)
        left_x = self.weighted_peak(histogram[edge:max(edge, cx - min_sep)], edge)
        right_x = self.weighted_peak(histogram[min(w - edge, cx + min_sep):w - edge],
                                     min(w - edge, cx + min_sep))

        expected_half_width = w * self.lane_half_width_ratio
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2.0
        elif left_x is not None:
            lane_center = left_x + expected_half_width
        elif right_x is not None:
            lane_center = right_x - expected_half_width
        else:
            return None

        return float(np.clip((lane_center - cx) / (w / 2.0), -1.0, 1.0))

    @staticmethod
    def weighted_peak(histogram, offset):
        if histogram.size == 0 or np.sum(histogram) < 40:
            return None
        peak = int(np.argmax(histogram))
        if histogram[peak] <= 0:
            return None
        window = 35
        start = max(0, peak - window)
        end = min(histogram.size, peak + window + 1)
        weights = histogram[start:end]
        if np.sum(weights) <= 0:
            return None
        xs = np.arange(start, end, dtype=float) + offset
        return float(np.average(xs, weights=weights))

    def smooth_deviation(self, value):
        self.deviation_history.append(float(value))
        if len(self.deviation_history) > self.history_size:
            self.deviation_history.pop(0)
        weights = np.linspace(0.6, 1.0, len(self.deviation_history))
        return float(np.average(self.deviation_history, weights=weights))

    def scan_callback(self, msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        idx = np.arange(ranges.size, dtype=float)
        angles = msg.angle_min + idx * msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        front = valid & (np.abs(angles) <= self.front_half_angle)
        left = valid & (angles >= self.side_min_angle) & (angles <= self.side_max_angle)
        right = valid & (angles <= -self.side_min_angle) & (angles >= -self.side_max_angle)

        self.front_distance = self.min_or_inf(ranges[front])
        self.left_distance = self.min_or_inf(ranges[left])
        self.right_distance = self.min_or_inf(ranges[right])
        self.nearest_distance = self.min_or_inf(ranges[valid])

    @staticmethod
    def min_or_inf(values):
        return float(np.min(values)) if values.size > 0 else float('inf')

    def scale_angular(self, angular):
        if angular > 0.0:
            angular *= self.left_turn_gain
        elif angular < 0.0:
            angular *= self.right_turn_gain
        return float(np.clip(angular, -self.max_angular_z, self.max_angular_z))

    def scale_return_angular(self, angular):
        angular *= self.return_turn_sign
        if angular > 0.0:
            angular *= self.left_turn_gain
        elif angular < 0.0:
            angular *= self.right_turn_gain
        return float(np.clip(angular, -self.return_max_angular_z, self.return_max_angular_z))

    def barrier_angular_correction(self):
        correction = 0.0
        if self.left_distance < self.barrier_influence_distance:
            # Sol bariyer yakin: saga don (negative angular.z)
            correction -= (self.barrier_safe_distance - self.left_distance) * self.barrier_gain
        if self.right_distance < self.barrier_influence_distance:
            # Sag bariyer yakin: sola don (positive angular.z)
            correction += (self.barrier_safe_distance - self.right_distance) * self.barrier_gain
        return float(np.clip(correction, -0.25, 0.25))

    def _choose_avoidance_dir(self):
        """On engel var: sag mi sol mu daha genis? O yone don."""
        if self.right_distance >= self.left_distance:
            # Sagda daha fazla alan → saga don (negative angular.z)
            return -1.0
        else:
            # Solda daha fazla alan → sola don (positive angular.z)
            return 1.0

    def _avoidance_angular(self, direction):
        self.get_logger().warn(
            f'KACINIS: on={self.front_distance:.2f}m '
            f'sol={self.left_distance:.2f}m sag={self.right_distance:.2f}m '
            f'yon={"SOL" if direction > 0 else "SAG"}',
            throttle_duration_sec=0.5)
        return direction * self.max_angular_z

    def drive_loop(self):
        twist = Twist()

        if not self.have_image:
            self.cmd_pub.publish(twist)
            return

        now = time.monotonic()
        obstacle_in_avoidance_zone = self.front_distance < self.front_slow_distance

        if obstacle_in_avoidance_zone:
            if not self._avoidance_active:
                self._avoidance_dir = self._choose_avoidance_dir()
            self._avoidance_active = True
            self._avoidance_last_seen = now
        elif (
            self._avoidance_active
            and now - self._avoidance_last_seen > self.avoidance_hold_duration
        ):
            self._avoidance_active = False
            self._return_active = True
            self._return_start = now
            self.get_logger().info('Kacis bitti, serit ortalama manevrasi basladi.')

        return_elapsed = now - self._return_start if self._return_active else 0.0
        return_centered = abs(self.last_deviation) <= self.return_center_tolerance
        return_timed_out = return_elapsed > self.return_hold_duration
        return_can_finish = return_elapsed >= self.return_min_duration and return_centered
        if self._return_active and (return_can_finish or return_timed_out):
            self._return_active = False
            self._avoidance_dir = 0.0
            self.get_logger().info(
                f'Serit ortalama manevrasi bitti, normal takibe donuldu '
                f'(dev={self.last_deviation:.2f}, sure={return_elapsed:.1f}s).')

        # --- ACİL DUR: geri gitmeden dur ve açık yöne yönlen ---
        emergency = (
            self.front_distance < self.front_emergency_distance
            or self.nearest_distance < self.hard_stop_distance
        )
        if emergency:
            if self._avoidance_dir == 0.0:
                self._avoidance_dir = self._choose_avoidance_dir()
            self.get_logger().warn(
                f'ACİL DUR → GERİ GİTMEDEN YÖNLEN '
                f'(on={self.front_distance:.2f}m min={self.nearest_distance:.2f}m '
                f'yon={"SOL" if self._avoidance_dir > 0 else "SAG"})',
                throttle_duration_sec=0.5)
            twist.linear.x = self.emergency_turn_speed
            twist.angular.z = self.scale_angular(self._avoidance_dir * self.max_angular_z)

        elif self._avoidance_active:
            # Yaklasma bolgesi: yavasla ve kilitli yone kac. Engel on koniden
            # ciksa bile hold suresi boyunca manevrayi yarida kesme.
            measured_front = (
                self.front_distance
                if np.isfinite(self.front_distance)
                else self.front_slow_distance
            )
            proximity = 1.0 - (measured_front - self.front_emergency_distance) / \
                        max(self.front_slow_distance - self.front_emergency_distance, 1e-6)
            proximity = float(np.clip(proximity, 0.0, 1.0))
            speed = float(self.avoidance_speed * (1.0 - 0.6 * proximity))
            speed = max(speed, self.min_speed)
            angular = self._avoidance_angular(self._avoidance_dir)
            twist.linear.x = speed
            twist.angular.z = self.scale_angular(angular)

        elif self._return_active:
            # Kacisin tersine donerek araci eski serit merkezine geri tasir.
            # Sure dolmadan once serit merkezi yakalanirsa normal takibe doner.
            lane_angular = -self.last_deviation * self.lane_gain
            if abs(self.last_deviation) > self.return_center_tolerance and abs(lane_angular) > 1e-3:
                # Serit merkezi gorunuyorsa donus yonunu sapma belirlesin.
                # Bu, aracin kactigi yan seritte kalmasini engeller.
                return_dir = 1.0 if lane_angular > 0.0 else -1.0
            else:
                return_dir = -self._avoidance_dir
            return_angular = return_dir * self.return_max_angular_z * self.return_turn_ratio
            angular = self.scale_return_angular(return_angular + 1.10 * lane_angular)
            self.get_logger().warn(
                f'ORTALAMA: yon={"SOL" if return_dir > 0 else "SAG"} '
                f'dev={self.last_deviation:.2f} sure={return_elapsed:.1f}s',
                throttle_duration_sec=0.5)
            twist.linear.x = self.return_speed
            twist.angular.z = angular

        else:
            # Normal suurus: serit takip + bariyer duzeltmesi
            lane_angular = -self.last_deviation * self.lane_gain
            angular = lane_angular + self.barrier_angular_correction()
            angular = self.scale_angular(angular)
            curve_ratio = min(abs(angular) / max(self.max_angular_z, 1e-6), 1.0)
            speed = self.base_speed - (self.base_speed - self.min_speed) * curve_ratio
            twist.linear.x = float(speed)
            twist.angular.z = angular

        self.cmd_pub.publish(twist)

    def destroy_node(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTrackDriver()
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
