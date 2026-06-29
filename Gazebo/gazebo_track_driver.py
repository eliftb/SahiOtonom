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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from utils.ros_logger import apply_log_level
import math


class GazeboTrackDriver(Node):
    def __init__(self):
        super().__init__('gazebo_track_driver')

        self.declare_parameter('camera_topic', '/serit_takip_kamerasi/image_raw')
        self.declare_parameter('scan_topic', '/lidar')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')

        self.declare_parameter('base_speed', 0.28)
        self.declare_parameter('min_speed', 0.14)
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
        self.declare_parameter('front_slow_distance', 3.0)
        self.declare_parameter('hard_stop_distance', 0.42)
        self.declare_parameter('front_angle_range_deg', 10.0)
        self.declare_parameter('side_min_angle_deg', 30.0)
        self.declare_parameter('side_max_angle_deg', 115.0)
        self.declare_parameter('emergency_turn_speed', 0.0)
        self.declare_parameter('avoidance_hold_duration', 3.0)
        self.declare_parameter('avoidance_speed', 0.22)
        self.declare_parameter('avoidance_phase_duration', 12.0)
        self.declare_parameter('avoidance_center_duration', 24.0)
        self.declare_parameter('avoidance_center_speed', 0.22)
        self.declare_parameter('avoidance_turn_angular', 0.85)
        self.declare_parameter('avoidance_return_angular', 0.85)
        self.declare_parameter('obstacle_escape_angular', 0.85)
        self.declare_parameter('obstacle_return_angular', 0.55)
        self.declare_parameter('avoidance_return_duration', 20.0)
        self.declare_parameter('return_hold_duration', 2.0)
        self.declare_parameter('return_speed', 0.18)
        self.declare_parameter('return_turn_ratio', 0.55)
        self.declare_parameter('return_max_angular_z', 1.8)
        self.declare_parameter('return_turn_sign', -1.0)
        self.declare_parameter('return_min_duration', 1.0)
        self.declare_parameter('return_center_tolerance', 0.08)
        self.declare_parameter('pose_maneuver_enabled', False)
        self.declare_parameter('pose_maneuver_x', 0.0)
        self.declare_parameter('pose_maneuver_y', 0.0)
        self.declare_parameter('pose_maneuver_radius', 1.5)
        self.declare_parameter('pose_maneuver_turn_duration', 20.0)
        self.declare_parameter('pose_maneuver_center_duration', 20.0)

        camera_topic = self.get_parameter('camera_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

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
        self.avoidance_phase_duration = float(self.get_parameter('avoidance_phase_duration').value)
        self.avoidance_center_duration = float(self.get_parameter('avoidance_center_duration').value)
        self.avoidance_center_speed = float(self.get_parameter('avoidance_center_speed').value)
        self.avoidance_turn_angular = float(self.get_parameter('avoidance_turn_angular').value)
        self.avoidance_return_angular = float(self.get_parameter('avoidance_return_angular').value)
        self.obstacle_escape_angular = float(self.get_parameter('obstacle_escape_angular').value)
        self.obstacle_return_angular = float(self.get_parameter('obstacle_return_angular').value)
        self.avoidance_return_duration = float(self.get_parameter('avoidance_return_duration').value)
        self.return_hold_duration = float(self.get_parameter('return_hold_duration').value)
        self.return_speed = float(self.get_parameter('return_speed').value)
        self.return_turn_ratio = float(self.get_parameter('return_turn_ratio').value)
        self.return_max_angular_z = float(self.get_parameter('return_max_angular_z').value)
        self.return_turn_sign = float(self.get_parameter('return_turn_sign').value)
        self.return_min_duration = float(self.get_parameter('return_min_duration').value)
        self.return_center_tolerance = float(self.get_parameter('return_center_tolerance').value)
        self.pose_maneuver_enabled = bool(self.get_parameter('pose_maneuver_enabled').value)
        self.pose_maneuver_x = float(self.get_parameter('pose_maneuver_x').value)
        self.pose_maneuver_y = float(self.get_parameter('pose_maneuver_y').value)
        self.pose_maneuver_radius = float(self.get_parameter('pose_maneuver_radius').value)
        self.pose_maneuver_turn_duration = float(self.get_parameter('pose_maneuver_turn_duration').value)
        self.pose_maneuver_center_duration = float(self.get_parameter('pose_maneuver_center_duration').value)

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
        self._avoidance_phase = 'normal'
        self._avoidance_phase_start = 0.0
        self._pose_maneuver_consumed = False
        self.car_x = 0.0
        self.car_y = 0.0
        self.have_odom = False

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_timer(0.1, self.drive_loop)

        self.get_logger().info(
            f'Gazebo Track Driver basladi | camera={camera_topic} scan={scan_topic} cmd={cmd_vel_topic}\n'
            f'  On dur mesafesi  : {self.front_emergency_distance} m\n'
            f'  On yavasla mesaf.: {self.front_slow_distance} m\n'
            f'  Sert min mesafe  : {self.hard_stop_distance} m\n'
            f'  Konum manevrasi  : {"acik" if self.pose_maneuver_enabled else "kapali"}'
        )
        apply_log_level(self)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.car_x = float(p.x)
        self.car_y = float(p.y)
        self.have_odom = True

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

    def _start_timed_avoidance(self, now):
        self._avoidance_phase = 'escape_left'
        self._avoidance_phase_start = now
        self._avoidance_active = False
        self._return_active = False
        self._avoidance_dir = 1.0
        self.get_logger().warn(
            'ENGEL GORULDU: 12s SOL seride cik, 24s ortala, 20s SAG seride don!',
            throttle_duration_sec=0.5)

    def _start_pose_maneuver(self, now, distance):
        self._avoidance_phase = 'pose_turn_left'
        self._avoidance_phase_start = now
        self._avoidance_active = False
        self._return_active = False
        self._avoidance_dir = 1.0
        self._pose_maneuver_consumed = True
        self.get_logger().warn(
            f'KONUM DONUSU BASLADI: 20s sola/donus, 20s ortalama '
            f'(x={self.car_x:.2f}, y={self.car_y:.2f}, hedefe={distance:.2f}m)',
            throttle_duration_sec=0.5)

    def _check_pose_maneuver(self, now):
        if (
            not self.pose_maneuver_enabled
            or self._pose_maneuver_consumed
            or self._avoidance_phase != 'normal'
            or not self.have_odom
        ):
            return

        distance = math.hypot(self.car_x - self.pose_maneuver_x, self.car_y - self.pose_maneuver_y)
        if distance <= self.pose_maneuver_radius:
            self._start_pose_maneuver(now, distance)

    def _advance_timed_avoidance(self, now):
        elapsed = now - self._avoidance_phase_start
        if self._avoidance_phase == 'pose_turn_left':
            phase_duration = self.pose_maneuver_turn_duration
        elif self._avoidance_phase == 'pose_center':
            phase_duration = self.pose_maneuver_center_duration
        elif self._avoidance_phase == 'center_left':
            phase_duration = self.avoidance_center_duration
        elif self._avoidance_phase == 'return_right':
            phase_duration = self.avoidance_return_duration
        else:
            phase_duration = self.avoidance_phase_duration
        if elapsed + 1e-6 < phase_duration:
            return elapsed

        if self._avoidance_phase == 'escape_left':
            self._avoidance_phase = 'center_left'
            self._avoidance_phase_start = now
            self.get_logger().info('SOL serit ortalama fazi basladi.')
            return 0.0

        if self._avoidance_phase == 'pose_turn_left':
            self._avoidance_phase = 'pose_center'
            self._avoidance_phase_start = now
            self.get_logger().info('Konum donusu ortalama fazi basladi.')
            return 0.0

        if self._avoidance_phase == 'pose_center':
            self._avoidance_phase = 'normal'
            self._avoidance_phase_start = 0.0
            self._avoidance_dir = 0.0
            self.get_logger().info('Konum donusu bitti, normal serit takibe donuldu.')
            return 0.0

        if self._avoidance_phase == 'center_left':
            self._avoidance_phase = 'return_right'
            self._avoidance_phase_start = now
            self.get_logger().info('SAG seride donus fazi basladi.')
            return 0.0

        if self._avoidance_phase == 'return_right':
            self._avoidance_phase = 'normal'
            self._avoidance_phase_start = 0.0
            self._avoidance_dir = 0.0
            self.get_logger().info('Engel kacma bitti, normal serit takibe donuldu.')
            return 0.0

        return elapsed

    def _lane_follow_angular(self):
        lane_angular = -self.last_deviation * self.lane_gain
        angular = lane_angular + self.barrier_angular_correction()
        return self.scale_angular(angular)

    def _obstacle_escape_cmd(self, twist):
        twist.linear.x = max(self.avoidance_speed, self.min_speed)
        twist.angular.z = self.scale_angular(abs(self.obstacle_escape_angular))

    def _obstacle_center_cmd(self, twist):
        twist.linear.x = max(self.avoidance_center_speed, self.min_speed)
        twist.angular.z = self._lane_follow_angular()

    def _obstacle_return_cmd(self, twist):
        lane_angular = -self.last_deviation * self.lane_gain
        return_angular = -abs(self.obstacle_return_angular) + 0.35 * lane_angular
        twist.linear.x = max(self.return_speed, self.min_speed)
        twist.angular.z = self.scale_angular(return_angular)

    def drive_loop(self):
        twist = Twist()

        if not self.have_image:
            self.cmd_pub.publish(twist)
            return

        now = time.monotonic()
        obstacle_in_avoidance_zone = self.front_distance < self.front_slow_distance

        self._check_pose_maneuver(now)

        if self._avoidance_phase == 'normal' and obstacle_in_avoidance_zone:
            self._start_timed_avoidance(now)

        phase_elapsed = 0.0
        if self._avoidance_phase != 'normal':
            phase_elapsed = self._advance_timed_avoidance(now)

        if self._avoidance_phase in ('escape_left', 'pose_turn_left'):
            self._obstacle_escape_cmd(twist)
            label = 'KONUM DONUSU' if self._avoidance_phase == 'pose_turn_left' else 'SOLLAMA 1/3'
            self.get_logger().warn(
                f'{label}: SOLA cikiliyor sure={phase_elapsed:.1f}s '
                f'on={self.front_distance:.2f}m',
                throttle_duration_sec=0.5)

        elif self._avoidance_phase in ('center_left', 'pose_center'):
            self._obstacle_center_cmd(twist)
            label = 'KONUM ORTALAMA' if self._avoidance_phase == 'pose_center' else 'SOLLAMA 2/3'
            self.get_logger().warn(
                f'{label}: Seritte ortalaniyor sure={phase_elapsed:.1f}s '
                f'dev={self.last_deviation:.2f}',
                throttle_duration_sec=0.5)

        elif self._avoidance_phase == 'return_right':
            self._obstacle_return_cmd(twist)
            self.get_logger().warn(
                f'SOLLAMA 3/3: SAG seride donuluyor sure={phase_elapsed:.1f}s '
                f'dev={self.last_deviation:.2f}',
                throttle_duration_sec=0.5)

        else:
            # Normal suurus: serit takip + bariyer duzeltmesi
            emergency = (
                self.front_distance < self.front_emergency_distance
                or self.nearest_distance < self.hard_stop_distance
            )
            if emergency:
                self.get_logger().warn(
                    f'ACIL DUR (on={self.front_distance:.2f}m min={self.nearest_distance:.2f}m)',
                    throttle_duration_sec=0.5)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return

            angular = self._lane_follow_angular()
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
