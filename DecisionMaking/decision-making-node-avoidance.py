#!/usr/bin/env python3
import json
import math
import sys
import time
from enum import Enum
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, Int32, String

from utils.ros_logger import apply_log_level


class VehicleState(Enum):
    NORMAL = "normal"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    POSE_TURN_ALIGN = "pose_turn_align"
    STOP_SIGN_WAIT = "stop_sign_wait"
    TRAFFIC_LIGHT_STOP = "traffic_light_stop"
    OBSTACLE_ESCAPE = "obstacle_escape"
    OBSTACLE_CENTER = "obstacle_center"
    OBSTACLE_RETURN = "obstacle_return"
    OBSTACLE_RIGHT_ALIGN = "obstacle_right_align"
    BUS_STOP_CHECK = "bus_stop_check"
    BUS_STOP_ENTER = "bus_stop_enter"
    BUS_STOP_ALIGN = "bus_stop_align"
    BUS_STOP_WAIT = "bus_stop_wait"
    BUS_STOP_EXIT = "bus_stop_exit"
    BUS_STOP_EXIT_ALIGN = "bus_stop_exit_align"


class DecisionMakingNode(Node):
    """Perception-driven vehicle behavior state machine."""

    LEFT_SIGNS = {
        "ileriden-sola-mecburi-yon",
        "sola-don",
        "sola-mecburi-yon",
        "ileri-ve-sola-mecburi-yon",
    }
    RIGHT_SIGNS = {
        "ileriden-saga-mecburi-yon",
        "saga-mecburi-yon",
        "ileri-ve-saga-mecburi-yon",
    }

    def __init__(self):
        super().__init__("decision_making_node")

        self.declare_parameter("base_speed", 0.40)
        self.declare_parameter("min_speed", 0.22)
        self.declare_parameter("lane_steering_gain", 1.05)
        self.declare_parameter("normal_lane_correct_heading", False)
        self.declare_parameter("normal_lane_gain_multiplier", 1.0)
        self.declare_parameter("avoidance_heading_gain", 1.60)
        self.declare_parameter("max_steering_angle", 1.25)

        self.declare_parameter("turn_speed", 0.28)
        self.declare_parameter("turn_steering_angle", 0.42)
        self.declare_parameter("turn_min_duration", 1.0)
        self.declare_parameter("turn_max_duration", 5.0)
        self.declare_parameter("turn_intent_timeout", 15.0)
        self.declare_parameter("turn_action_cooldown", 12.0)
        self.declare_parameter("left_turn_speed", 0.18)
        self.declare_parameter("left_turn_steering_angle", 1.10)
        self.declare_parameter("left_turn_duration", 40.0)
        self.declare_parameter("left_turn_start_delay", 4.0)
        self.declare_parameter("right_turn_speed", 0.24)
        self.declare_parameter("right_turn_steering_angle", 0.55)
        self.declare_parameter("right_turn_duration", 5.0)
        self.declare_parameter("right_turn_start_delay", 0.0)
        self.declare_parameter("intersection_clear_frames", 5)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_turn_enabled", False)
        self.declare_parameter("pose_turn_direction", "left")
        self.declare_parameter("pose_turn_x", 0.0)
        self.declare_parameter("pose_turn_y", 0.0)
        self.declare_parameter("pose_turn_radius", 1.2)
        self.declare_parameter("pose_turn_align_duration", 0.0)
        self.declare_parameter("pose_turn_align_speed", 0.32)
        self.declare_parameter("pose_turn_align_gain_multiplier", 1.25)
        self.declare_parameter("post_turn_obstacle_ignore_duration", 10.0)

        self.declare_parameter("stop_sign_duration", 3.0)
        self.declare_parameter("sign_intent_area_ratio", 0.003)
        self.declare_parameter("sign_action_area_ratio", 0.010)
        self.declare_parameter("traffic_light_area_ratio", 0.002)
        self.declare_parameter("traffic_stop_pose_enabled", False)
        self.declare_parameter("traffic_stop_x", 0.0)
        self.declare_parameter("traffic_stop_y", 0.0)
        self.declare_parameter("traffic_stop_radius", 1.2)
        self.declare_parameter("sign_action_cooldown", 12.0)
        self.declare_parameter("sign_turn_enabled", True)

        self.declare_parameter("obstacle_trigger_distance", 3.0)
        self.declare_parameter("emergency_stop_distance", 0.40)
        self.declare_parameter("avoidance_escape_duration", 5.0)
        self.declare_parameter("avoidance_center_duration", 10.0)
        self.declare_parameter("avoidance_return_duration", 6.0)
        self.declare_parameter("avoidance_speed", 0.35)
        self.declare_parameter("avoidance_center_speed", 0.40)
        self.declare_parameter("avoidance_steering_angle", 0.85)
        self.declare_parameter("obstacle_escape_steering_angle", 0.85)
        self.declare_parameter("obstacle_return_steering_angle", 0.55)
        self.declare_parameter("avoidance_center_tolerance", 0.12)
        self.declare_parameter("avoidance_heading_tolerance", 0.18)
        self.declare_parameter("avoidance_center_stable_frames", 6)
        self.declare_parameter("avoidance_center_max_extra_duration", 10.0)
        self.declare_parameter("avoidance_center_gain_multiplier", 1.25)
        self.declare_parameter("avoidance_right_align_duration", 0.0)
        self.declare_parameter("avoidance_right_align_timeout", 12.0)
        self.declare_parameter("avoidance_return_turn_duration", 8.0)
        self.declare_parameter("avoidance_return_counter_duration", 0.0)
        self.declare_parameter("avoidance_return_counter_ratio", 0.0)
        self.declare_parameter("avoidance_return_align_gain_multiplier", 1.40)
        self.declare_parameter("avoidance_right_align_turn_duration", 0.0)
        self.declare_parameter("avoidance_right_align_turn_angle", 0.60)

        self.declare_parameter("bus_stop_check_duration", 1.2)
        self.declare_parameter("bus_stop_clear_distance", 1.25)
        self.declare_parameter("bus_stop_clear_frames", 5)
        self.declare_parameter("bus_stop_entry_duration", 2.6)
        self.declare_parameter("bus_stop_align_duration", 1.8)
        self.declare_parameter("bus_stop_wait_duration", 5.0)
        self.declare_parameter("bus_stop_exit_duration", 2.6)
        self.declare_parameter("bus_stop_exit_align_duration", 1.8)
        self.declare_parameter("bus_stop_speed", 0.22)
        self.declare_parameter("bus_stop_steering_angle", 0.38)
        self.declare_parameter("range_fresh_timeout", 1.0)

        self.base_speed = self._float_param("base_speed")
        self.min_speed = self._float_param("min_speed")
        self.lane_steering_gain = self._float_param("lane_steering_gain")
        self.normal_lane_correct_heading = self._bool_param(
            "normal_lane_correct_heading")
        self.normal_lane_gain_multiplier = self._float_param(
            "normal_lane_gain_multiplier")
        self.avoidance_heading_gain = self._float_param(
            "avoidance_heading_gain")
        self.max_steering_angle = self._float_param("max_steering_angle")

        self.turn_speed = self._float_param("turn_speed")
        self.turn_steering_angle = self._float_param("turn_steering_angle")
        self.turn_min_duration = self._float_param("turn_min_duration")
        self.turn_max_duration = self._float_param("turn_max_duration")
        self.turn_intent_timeout = self._float_param("turn_intent_timeout")
        self.turn_action_cooldown = self._float_param("turn_action_cooldown")
        self.left_turn_speed = self._float_param("left_turn_speed")
        self.left_turn_steering_angle = self._float_param(
            "left_turn_steering_angle")
        self.left_turn_duration = self._float_param("left_turn_duration")
        self.left_turn_start_delay = self._float_param("left_turn_start_delay")
        self.right_turn_speed = self._float_param("right_turn_speed")
        self.right_turn_steering_angle = self._float_param(
            "right_turn_steering_angle")
        self.right_turn_duration = self._float_param("right_turn_duration")
        self.right_turn_start_delay = self._float_param(
            "right_turn_start_delay")
        self.intersection_clear_frames = self._int_param("intersection_clear_frames")
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.pose_turn_enabled = self._bool_param("pose_turn_enabled")
        self.pose_turn_direction = (
            str(self.get_parameter("pose_turn_direction").value)
            .strip()
            .lower()
        )
        if self.pose_turn_direction not in {"left", "right"}:
            self.pose_turn_direction = "left"
        self.pose_turn_x = self._float_param("pose_turn_x")
        self.pose_turn_y = self._float_param("pose_turn_y")
        self.pose_turn_radius = self._float_param("pose_turn_radius")
        self.pose_turn_align_duration = self._float_param(
            "pose_turn_align_duration")
        self.pose_turn_align_speed = self._float_param("pose_turn_align_speed")
        self.pose_turn_align_gain_multiplier = self._float_param(
            "pose_turn_align_gain_multiplier")
        self.post_turn_obstacle_ignore_duration = self._float_param(
            "post_turn_obstacle_ignore_duration")

        self.stop_sign_duration = self._float_param("stop_sign_duration")
        self.sign_intent_area_ratio = self._float_param("sign_intent_area_ratio")
        self.sign_action_area_ratio = self._float_param("sign_action_area_ratio")
        self.traffic_light_area_ratio = self._float_param("traffic_light_area_ratio")
        self.traffic_stop_pose_enabled = self._bool_param(
            "traffic_stop_pose_enabled")
        self.traffic_stop_x = self._float_param("traffic_stop_x")
        self.traffic_stop_y = self._float_param("traffic_stop_y")
        self.traffic_stop_radius = self._float_param("traffic_stop_radius")
        self.sign_action_cooldown = self._float_param("sign_action_cooldown")
        self.sign_turn_enabled = self._bool_param("sign_turn_enabled")

        self.obstacle_trigger_distance = self._float_param("obstacle_trigger_distance")
        self.emergency_stop_distance = self._float_param("emergency_stop_distance")
        self.avoidance_escape_duration = self._float_param("avoidance_escape_duration")
        self.avoidance_center_duration = self._float_param("avoidance_center_duration")
        self.avoidance_return_duration = self._float_param("avoidance_return_duration")
        self.avoidance_speed = self._float_param("avoidance_speed")
        self.avoidance_center_speed = self._float_param("avoidance_center_speed")
        self.avoidance_steering_angle = self._float_param("avoidance_steering_angle")
        self.obstacle_escape_steering_angle = self._float_param(
            "obstacle_escape_steering_angle")
        self.obstacle_return_steering_angle = self._float_param(
            "obstacle_return_steering_angle")
        self.avoidance_center_tolerance = self._float_param(
            "avoidance_center_tolerance")
        self.avoidance_heading_tolerance = self._float_param(
            "avoidance_heading_tolerance")
        self.avoidance_center_stable_frames = self._int_param(
            "avoidance_center_stable_frames")
        self.avoidance_center_max_extra_duration = self._float_param(
            "avoidance_center_max_extra_duration")
        self.avoidance_center_gain_multiplier = self._float_param(
            "avoidance_center_gain_multiplier")
        self.avoidance_right_align_duration = self._float_param(
            "avoidance_right_align_duration")
        self.avoidance_right_align_timeout = self._float_param(
            "avoidance_right_align_timeout")
        self.avoidance_return_turn_duration = self._float_param(
            "avoidance_return_turn_duration")
        self.avoidance_return_counter_duration = self._float_param(
            "avoidance_return_counter_duration")
        self.avoidance_return_counter_ratio = self._float_param(
            "avoidance_return_counter_ratio")
        self.avoidance_return_align_gain_multiplier = self._float_param(
            "avoidance_return_align_gain_multiplier")
        self.avoidance_right_align_turn_duration = self._float_param(
            "avoidance_right_align_turn_duration")
        self.avoidance_right_align_turn_angle = self._float_param(
            "avoidance_right_align_turn_angle")

        self.bus_stop_check_duration = self._float_param("bus_stop_check_duration")
        self.bus_stop_clear_distance = self._float_param("bus_stop_clear_distance")
        self.bus_stop_clear_frames = self._int_param("bus_stop_clear_frames")
        self.bus_stop_entry_duration = self._float_param("bus_stop_entry_duration")
        self.bus_stop_align_duration = self._float_param("bus_stop_align_duration")
        self.bus_stop_wait_duration = self._float_param("bus_stop_wait_duration")
        self.bus_stop_exit_duration = self._float_param("bus_stop_exit_duration")
        self.bus_stop_exit_align_duration = self._float_param(
            "bus_stop_exit_align_duration")
        self.bus_stop_speed = self._float_param("bus_stop_speed")
        self.bus_stop_steering_angle = self._float_param(
            "bus_stop_steering_angle")
        self.range_fresh_timeout = self._float_param("range_fresh_timeout")

        self.state = VehicleState.NORMAL
        self.state_started_at = time.monotonic()
        self.lateral_deviation = 0.0
        self.heading_deviation = 0.0
        self.intersection_direction = 0
        self.intersection_clear_count = 0
        self.pending_turn = None
        self.pending_turn_expires_at = 0.0
        self.pending_turn_ready_at = 0.0
        self.pose_turn_consumed = False
        self.pose_turn_align_pending = False
        self.obstacle_ignore_until = 0.0
        self.have_odom = False
        self.car_x = 0.0
        self.car_y = 0.0

        self.obstacle_detected = False
        self.obstacle_distance = float("inf")
        self.obstacle_is_right = True
        self.obstacle_centered_count = 0
        self.obstacle_right_centered_count = 0
        self.barrier_safety_stop = False
        self.barrier_correction = 0.0
        self.right_barrier_distance = float("inf")

        self.right_front_range = float("inf")
        self.right_rear_range = float("inf")
        self.right_front_range_time = 0.0
        self.right_rear_range_time = 0.0
        self.bus_stop_clear_count = 0
        self.bus_stop_requested = False
        self.stop_requested = False
        self.last_action_times = {}

        self.red_light_active = False
        self.traffic_light_detection_active = False
        self.traffic_stop_consumed = False
        self.last_sign_summary = "none"

        self.create_subscription(
            Float32, "/lane/lateral_deviation", self.lateral_callback, 10)
        self.create_subscription(
            Float32, "/lane/heading_deviation", self.heading_callback, 10)
        self.create_subscription(
            Int32, "/lane/intersection_direction", self.intersection_callback, 10)
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(
            String, "/sign_detection/events", self.sign_events_callback, 10)
        self.create_subscription(
            Bool, "/obstacle_detected", self.obstacle_detected_callback, 10)
        self.create_subscription(
            Float32, "/obstacle_distance", self.obstacle_distance_callback, 10)
        self.create_subscription(
            Bool, "/obstacle_side", self.obstacle_side_callback, 10)
        self.create_subscription(
            Bool, "/barrier/safety_stop", self.barrier_safety_stop_callback, 10)
        self.create_subscription(
            Float32,
            "/barrier/lateral_correction",
            self.barrier_correction_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/barrier/right_distance",
            self.right_barrier_callback,
            10,
        )
        self.create_subscription(
            Range,
            "/ultrasonic_sensor_2_sag_on",
            self.right_front_range_callback,
            10,
        )
        self.create_subscription(
            Range,
            "/ultrasonic_sensor_3_sag_arka",
            self.right_rear_range_callback,
            10,
        )

        self.ackermann_pub = self.create_publisher(
            AckermannDrive, "/ackermann_cmd", 10)
        self.speed_pub = self.create_publisher(Float32, "/speed", 10)
        self.new_lateral_pub = self.create_publisher(
            Float32, "/lane/lateral_new_deviation", 10)
        self.vehicle_state_pub = self.create_publisher(
            Float32, "/vehicle_state", 10)
        self.state_name_pub = self.create_publisher(
            String, "/decision/state", 10)

        self.create_timer(0.1, self.decision_loop)
        self.get_logger().info(
            "DecisionMaking aktif: levha + kavsak + LiDAR + ultrasonik "
            f"durum makinesi | konum donusu="
            f"{'acik' if self.pose_turn_enabled else 'kapali'} | "
            f"levha donusu={'acik' if self.sign_turn_enabled else 'kapali'}")
        apply_log_level(self)

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    def _int_param(self, name):
        return int(self.get_parameter(name).value)

    def _bool_param(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def lateral_callback(self, msg):
        self.lateral_deviation = float(np.clip(msg.data, -1.0, 1.0))

    def heading_callback(self, msg):
        self.heading_deviation = float(np.clip(msg.data, -1.0, 1.0))

    def intersection_callback(self, msg):
        self.intersection_direction = int(msg.data)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.car_x = float(p.x)
        self.car_y = float(p.y)
        self.have_odom = True

    def obstacle_detected_callback(self, msg):
        self.obstacle_detected = bool(msg.data)

    def obstacle_distance_callback(self, msg):
        self.obstacle_distance = (
            float(msg.data) if msg.data > 0.0 else float("inf"))

    def obstacle_side_callback(self, msg):
        """Engel tarafı yalnızca teşhis içindir; sollama yönü daima soldur."""
        if self.state in {
            VehicleState.NORMAL,
            VehicleState.TRAFFIC_LIGHT_STOP,
            VehicleState.STOP_SIGN_WAIT,
        }:
            self.obstacle_is_right = bool(msg.data)

    def barrier_safety_stop_callback(self, msg):
        self.barrier_safety_stop = bool(msg.data)

    def barrier_correction_callback(self, msg):
        self.barrier_correction = float(np.clip(msg.data, -0.35, 0.35))

    def right_barrier_callback(self, msg):
        self.right_barrier_distance = (
            float(msg.data) if msg.data > 0.0 else float("inf"))

    def right_front_range_callback(self, msg):
        self.right_front_range = self._valid_range(msg)
        self.right_front_range_time = time.monotonic()

    def right_rear_range_callback(self, msg):
        self.right_rear_range = self._valid_range(msg)
        self.right_rear_range_time = time.monotonic()

    @staticmethod
    def _valid_range(msg):
        value = float(msg.range)
        if not np.isfinite(value) or value < msg.min_range:
            return float("inf")
        return value

    def sign_events_callback(self, msg):
        try:
            detections = json.loads(msg.data or "[]")
        except (TypeError, ValueError, json.JSONDecodeError):
            return

        if not isinstance(detections, list):
            return

        now = time.monotonic()
        visible_names = []
        red_seen = False
        green_seen = False

        for detection in detections:
            if not isinstance(detection, dict):
                continue
            name = str(detection.get("class", "")).lower()
            area = float(detection.get("area_ratio", 0.0))
            visible_names.append(name)

            if area >= self.traffic_light_area_ratio:
                red_seen |= name in {"kirmizi-isik", "sari-isik"}
                green_seen |= name == "yesil-isik"

            if area >= self.sign_intent_area_ratio:
                if name in self.LEFT_SIGNS and name != "sola-donulmez":
                    if (
                        self.sign_turn_enabled
                        and
                        not self._pose_turn_controls_direction("left")
                        and self._turn_ready("left", now)
                    ):
                        self._remember_turn("left", now)
                elif name in self.RIGHT_SIGNS and name != "saga-donulmez":
                    if (
                        self.sign_turn_enabled
                        and
                        not self._pose_turn_controls_direction("right")
                        and self._turn_ready("right", now)
                    ):
                        self._remember_turn("right", now)
                elif name == "sola-donulmez" and self.pending_turn == "left":
                    self.pending_turn = None
                    self.pending_turn_ready_at = 0.0
                elif name == "saga-donulmez" and self.pending_turn == "right":
                    self.pending_turn = None
                    self.pending_turn_ready_at = 0.0

            if area >= self.sign_action_area_ratio:
                if name == "dur" and self._action_ready("dur", now):
                    self.stop_requested = True
                elif name in {"durak", "park"} and self._action_ready(
                    "durak", now
                ):
                    self.bus_stop_requested = True

        light_seen = red_seen or green_seen
        if light_seen and not self.traffic_light_detection_active:
            self.traffic_light_detection_active = True
            self.get_logger().info(
                "Sign Detection trafik isigini gordu: isik karari aktif.")

        if self.traffic_light_detection_active and green_seen:
            self.red_light_active = False
        elif self.traffic_light_detection_active and red_seen:
            self.red_light_active = True

        self.last_sign_summary = ",".join(sorted(set(visible_names))) or "none"

    def _remember_turn(self, direction, now):
        is_new_turn = (
            self.pending_turn != direction
            or now >= self.pending_turn_expires_at
        )
        if is_new_turn:
            delay = self._turn_start_delay(direction)
            self.pending_turn_ready_at = now + delay
            self.get_logger().info(
                f"Yon levhasi hafizaya alindi: {direction.upper()} "
                f"(baslama gecikmesi {delay:.1f}s)")
        self.pending_turn = direction
        self.pending_turn_expires_at = now + self.turn_intent_timeout

    def _turn_start_delay(self, direction):
        if direction == "left":
            return self.left_turn_start_delay
        if direction == "right":
            return self.right_turn_start_delay
        return 0.0

    def _action_ready(self, action, now):
        return (
            now - self.last_action_times.get(action, -float("inf"))
            >= self.sign_action_cooldown
        )

    def _turn_ready(self, direction, now):
        return (
            now - self.last_action_times.get(f"turn_{direction}", -float("inf"))
            >= self.turn_action_cooldown
        )

    def _pose_turn_controls_direction(self, direction):
        return (
            self.pose_turn_enabled
            and not self.pose_turn_consumed
            and self.pose_turn_direction == direction
        )

    def _check_pose_turn(self, now):
        if (
            not self.pose_turn_enabled
            or self.pose_turn_consumed
            or not self.have_odom
            or self.state != VehicleState.NORMAL
        ):
            return False

        distance = math.hypot(
            self.car_x - self.pose_turn_x,
            self.car_y - self.pose_turn_y,
        )
        if distance > self.pose_turn_radius:
            return False

        direction = self.pose_turn_direction
        target = (
            VehicleState.TURN_LEFT
            if direction == "left"
            else VehicleState.TURN_RIGHT
        )
        self.pose_turn_consumed = True
        self.pose_turn_align_pending = True
        self.pending_turn = None
        self.pending_turn_expires_at = 0.0
        self.pending_turn_ready_at = 0.0
        self.last_action_times[f"turn_{direction}"] = now
        self._transition(
            target,
            f"konum tetigi {direction}: "
            f"x={self.car_x:.2f}, y={self.car_y:.2f}, "
            f"hedefe={distance:.2f}m",
        )
        return True

    def _traffic_stop_distance(self):
        if not self.have_odom:
            return float("inf")
        return math.hypot(
            self.car_x - self.traffic_stop_x,
            self.car_y - self.traffic_stop_y,
        )

    def _check_traffic_light_stop(self):
        if not self.traffic_stop_pose_enabled:
            if self.red_light_active:
                self._transition(
                    VehicleState.TRAFFIC_LIGHT_STOP,
                    "kirmizi/sari isik",
                )
                return True
            return False

        if self.traffic_stop_consumed:
            return False

        distance = self._traffic_stop_distance()
        if distance > self.traffic_stop_radius:
            return False

        if self.red_light_active:
            self._transition(
                VehicleState.TRAFFIC_LIGHT_STOP,
                f"trafik isigi konumu: kirmizi/sari "
                f"(hedefe={distance:.2f}m)",
            )
            return True

        if self.traffic_light_detection_active:
            self.traffic_stop_consumed = True
            self.get_logger().info(
                f"Trafik isigi konumu: yesil/serbest gecis "
                f"(hedefe={distance:.2f}m)")

        return False

    def _transition(self, new_state, reason):
        if new_state == self.state:
            return
        old_state = self.state
        self.state = new_state
        self.state_started_at = time.monotonic()
        self.intersection_clear_count = 0
        if new_state == VehicleState.OBSTACLE_CENTER:
            self.obstacle_centered_count = 0
        if new_state in {
            VehicleState.OBSTACLE_RETURN,
            VehicleState.OBSTACLE_RIGHT_ALIGN,
        }:
            self.obstacle_right_centered_count = 0
        self.get_logger().warn(
            f"KARAR: {old_state.value} -> {new_state.value} | {reason}")

    def _elapsed(self, now):
        return now - self.state_started_at

    def _intersection_supports_pending_turn(self):
        if self.pending_turn == "left":
            return self.intersection_direction in {1, 4}
        if self.pending_turn == "right":
            return self.intersection_direction in {2, 4}
        return False

    def _right_stop_area_clear(self, now):
        front_fresh = (
            now - self.right_front_range_time <= self.range_fresh_timeout)
        rear_fresh = (
            now - self.right_rear_range_time <= self.range_fresh_timeout)

        if front_fresh and rear_fresh:
            return (
                self.right_front_range >= self.bus_stop_clear_distance
                and self.right_rear_range >= self.bus_stop_clear_distance
            )
        return self.right_barrier_distance >= self.bus_stop_clear_distance

    def _lane_is_centered_for_overtake(self):
        return (
            abs(self.lateral_deviation) <= self.avoidance_center_tolerance
            and abs(self.heading_deviation) <= self.avoidance_heading_tolerance
        )

    def _update_overtake_center_count(self, right_lane=False):
        if self._lane_is_centered_for_overtake():
            if right_lane:
                self.obstacle_right_centered_count += 1
            else:
                self.obstacle_centered_count += 1
        elif right_lane:
            self.obstacle_right_centered_count = 0
        else:
            self.obstacle_centered_count = 0

    def _start_obstacle_avoidance(self):
        self._transition(
            VehicleState.OBSTACLE_ESCAPE,
            f"engel {self.obstacle_distance:.2f}m, sag seritten SOL seride kacis",
        )

    def _start_post_turn_obstacle_ignore(self, now):
        if self.post_turn_obstacle_ignore_duration <= 0.0:
            return
        self.obstacle_ignore_until = max(
            self.obstacle_ignore_until,
            now + self.post_turn_obstacle_ignore_duration,
        )
        self.get_logger().warn(
            f"Donus sonrasi hayalet engel filtresi: "
            f"{self.post_turn_obstacle_ignore_duration:.1f}s",
            throttle_duration_sec=1.0,
        )

    def _obstacle_response_allowed(self, now):
        return now >= self.obstacle_ignore_until

    def _barrier_stop_allowed(self, now):
        if not self._obstacle_response_allowed(now):
            return False
        return self.state not in {
            VehicleState.TURN_LEFT,
            VehicleState.TURN_RIGHT,
            VehicleState.POSE_TURN_ALIGN,
            VehicleState.OBSTACLE_ESCAPE,
            VehicleState.OBSTACLE_CENTER,
            VehicleState.OBSTACLE_RETURN,
            VehicleState.OBSTACLE_RIGHT_ALIGN,
        }

    def update_state(self, now):
        if self.pending_turn and now >= self.pending_turn_expires_at:
            self.get_logger().info("Yon levhasi zaman asimina ugradi.")
            self.pending_turn = None
            self.pending_turn_ready_at = 0.0

        elapsed = self._elapsed(now)

        if self.state == VehicleState.TURN_LEFT:
            if elapsed >= self.left_turn_duration:
                self.pending_turn = None
                self.pending_turn_ready_at = 0.0
                self._start_post_turn_obstacle_ignore(now)
                if (
                    self.pose_turn_align_pending
                    and self.pose_turn_align_duration > 0.0
                ):
                    self._transition(
                        VehicleState.POSE_TURN_ALIGN,
                        "sol donus bitti, serit takiple toparlanma",
                    )
                else:
                    self.pose_turn_align_pending = False
                    self._transition(
                        VehicleState.NORMAL,
                        "sol donus tamamlandi",
                    )
            return

        if self.state == VehicleState.TURN_RIGHT:
            if elapsed >= self.right_turn_duration:
                self.pending_turn = None
                self.pending_turn_ready_at = 0.0
                self._start_post_turn_obstacle_ignore(now)
                if (
                    self.pose_turn_align_pending
                    and self.pose_turn_align_duration > 0.0
                ):
                    self._transition(
                        VehicleState.POSE_TURN_ALIGN,
                        "sag donus bitti, serit takiple toparlanma",
                    )
                else:
                    self.pose_turn_align_pending = False
                    self._transition(
                        VehicleState.NORMAL,
                        "sag donus tamamlandi",
                    )
            return

        if self.state == VehicleState.POSE_TURN_ALIGN:
            if elapsed >= self.pose_turn_align_duration:
                self.pose_turn_align_pending = False
                self._transition(
                    VehicleState.NORMAL,
                    "konum donusu serit takibi tamamlandi",
                )
            return

        if self.state == VehicleState.STOP_SIGN_WAIT:
            if elapsed >= self.stop_sign_duration:
                self._transition(VehicleState.NORMAL, "dur levhasi beklemesi bitti")
            return

        if self.state == VehicleState.TRAFFIC_LIGHT_STOP:
            if not self.red_light_active:
                self.traffic_stop_consumed = True
                self._transition(VehicleState.NORMAL, "yesil isik")
            return

        if self.state == VehicleState.OBSTACLE_ESCAPE:
            if elapsed >= self.avoidance_escape_duration:
                self._transition(
                    VehicleState.OBSTACLE_CENTER,
                    "diger seritte merkezleme",
                )
            return

        if self.state == VehicleState.OBSTACLE_CENTER:
            if elapsed >= self.avoidance_center_duration:
                self._transition(
                    VehicleState.OBSTACLE_RETURN,
                    "sol serit merkezleme suresi bitti, sag seride geri donus",
                )
            return

        if self.state == VehicleState.OBSTACLE_RETURN:
            return_total_duration = (
                self.avoidance_return_turn_duration
                + self.avoidance_return_counter_duration
            )
            if elapsed >= return_total_duration:
                self._transition(
                    VehicleState.OBSTACLE_RIGHT_ALIGN,
                    "sag seritte serit takibiyle hizalanma",
                )
            return

        if self.state == VehicleState.OBSTACLE_RIGHT_ALIGN:
            self._update_overtake_center_count(right_lane=True)
            centered = (
                self.obstacle_right_centered_count
                >= self.avoidance_center_stable_frames
            )
            if elapsed >= self.avoidance_right_align_duration and centered:
                self._transition(VehicleState.NORMAL, "sollama tamamlandi")
            elif elapsed >= self.avoidance_right_align_timeout:
                self._transition(
                    VehicleState.NORMAL,
                    "sag serit hizalanma zaman asimi, normal takip",
                )
            return

        if self.state == VehicleState.BUS_STOP_CHECK:
            if self._right_stop_area_clear(now):
                self.bus_stop_clear_count += 1
            else:
                self.bus_stop_clear_count = 0
            if elapsed >= self.bus_stop_check_duration:
                self.last_action_times["durak"] = now
                if self.bus_stop_clear_count >= self.bus_stop_clear_frames:
                    self._transition(
                        VehicleState.BUS_STOP_ENTER,
                        "durak alani bos, park manevrasi",
                    )
                else:
                    self._transition(
                        VehicleState.NORMAL,
                        "durak alani dolu, yola devam",
                    )
            return

        if self.state == VehicleState.BUS_STOP_ENTER:
            if elapsed >= self.bus_stop_entry_duration:
                self._transition(
                    VehicleState.BUS_STOP_ALIGN,
                    "durak icinde hizalanma",
                )
            return

        if self.state == VehicleState.BUS_STOP_ALIGN:
            if elapsed >= self.bus_stop_align_duration:
                self._transition(
                    VehicleState.BUS_STOP_WAIT,
                    f"{self.bus_stop_wait_duration:.0f}s durak beklemesi",
                )
            return

        if self.state == VehicleState.BUS_STOP_WAIT:
            if elapsed >= self.bus_stop_wait_duration:
                self._transition(
                    VehicleState.BUS_STOP_EXIT,
                    "duraktan seride cikis",
                )
            return

        if self.state == VehicleState.BUS_STOP_EXIT:
            if elapsed >= self.bus_stop_exit_duration:
                self._transition(
                    VehicleState.BUS_STOP_EXIT_ALIGN,
                    "seritte yeniden hizalanma",
                )
            return

        if self.state == VehicleState.BUS_STOP_EXIT_ALIGN:
            if elapsed >= self.bus_stop_exit_align_duration:
                self._transition(VehicleState.NORMAL, "durak gorevi tamamlandi")
            return

        if self._check_traffic_light_stop():
            return
        elif self.stop_requested:
            self.stop_requested = False
            self.last_action_times["dur"] = now
            self._transition(VehicleState.STOP_SIGN_WAIT, "dur levhasi")
        elif self.bus_stop_requested:
            self.bus_stop_requested = False
            self.bus_stop_clear_count = 0
            self._transition(
                VehicleState.BUS_STOP_CHECK,
                "durak alani doluluk kontrolu",
            )
        elif self._check_pose_turn(now):
            return
        elif (
            self.obstacle_detected
            and self.obstacle_distance <= self.obstacle_trigger_distance
            and self._obstacle_response_allowed(now)
        ):
            self._start_obstacle_avoidance()
        elif self._intersection_supports_pending_turn():
            direction = self.pending_turn
            if now < self.pending_turn_ready_at:
                self.get_logger().warn(
                    f"{direction} donusu bekletiliyor: "
                    f"{self.pending_turn_ready_at - now:.1f}s",
                    throttle_duration_sec=0.5,
                )
                return
            if not self._turn_ready(direction, now):
                self.pending_turn = None
                self.pending_turn_ready_at = 0.0
                return
            target = (
                VehicleState.TURN_LEFT
                if direction == "left"
                else VehicleState.TURN_RIGHT
            )
            self.last_action_times[f"turn_{direction}"] = now
            self._transition(
                target,
                f"{direction} yon levhasi + kavsak algilandi",
            )

    def lane_steering(self, correct_heading=False, gain_multiplier=1.0):
        corrected_deviation = self.lateral_deviation + self.barrier_correction
        steering = (
            -corrected_deviation
            * self.lane_steering_gain
            * gain_multiplier
        )
        if correct_heading:
            steering -= (
                self.heading_deviation * self.avoidance_heading_gain)
        return float(np.clip(
            steering,
            -self.max_steering_angle,
            self.max_steering_angle,
        ))

    def obstacle_escape_command(self):
        # Varsayilan surus seridi sagdir. Engel hangi tarafta raporlanirsa
        # raporlansin sollama her zaman sol seride yapilir.
        return (
            self.avoidance_speed,
            abs(self.obstacle_escape_steering_angle),
        )

    def obstacle_center_command(self):
        return (
            self.avoidance_center_speed,
            self.lane_steering(
                correct_heading=True,
                gain_multiplier=self.avoidance_center_gain_multiplier,
            ),
        )

    def obstacle_return_steering(self):
        elapsed = self._elapsed(time.monotonic())
        if elapsed < self.avoidance_return_turn_duration:
            steering = -abs(self.obstacle_return_steering_angle)
        elif self.avoidance_return_counter_duration <= 0.0:
            steering = 0.0
        else:
            steering = (
                abs(self.obstacle_return_steering_angle)
                * self.avoidance_return_counter_ratio
            )
        return float(np.clip(
            steering,
            -self.max_steering_angle,
            self.max_steering_angle,
        ))

    def obstacle_return_command(self):
        return self.avoidance_speed, self.obstacle_return_steering()

    def obstacle_right_align_steering(self):
        elapsed = self._elapsed(time.monotonic())
        if elapsed < self.avoidance_right_align_turn_duration:
            return -abs(self.avoidance_right_align_turn_angle)

        return self.lane_steering(
            correct_heading=True,
            gain_multiplier=self.avoidance_return_align_gain_multiplier,
        )

    def obstacle_right_align_command(self):
        return (
            self.avoidance_center_speed,
            self.obstacle_right_align_steering(),
        )

    def command_for_state(self):
        lane_steering = self.lane_steering(
            correct_heading=self.normal_lane_correct_heading,
            gain_multiplier=self.normal_lane_gain_multiplier,
        )

        if self.state == VehicleState.TURN_LEFT:
            return self.left_turn_speed, abs(self.left_turn_steering_angle)
        if self.state == VehicleState.TURN_RIGHT:
            return self.right_turn_speed, -abs(self.right_turn_steering_angle)
        if self.state == VehicleState.POSE_TURN_ALIGN:
            return (
                self.pose_turn_align_speed,
                self.lane_steering(
                    correct_heading=True,
                    gain_multiplier=self.pose_turn_align_gain_multiplier,
                ),
            )
        if self.state in {
            VehicleState.STOP_SIGN_WAIT,
            VehicleState.TRAFFIC_LIGHT_STOP,
            VehicleState.BUS_STOP_WAIT,
        }:
            return 0.0, 0.0
        if self.state == VehicleState.OBSTACLE_ESCAPE:
            return self.obstacle_escape_command()
        if self.state == VehicleState.OBSTACLE_CENTER:
            return self.obstacle_center_command()
        if self.state == VehicleState.OBSTACLE_RETURN:
            return self.obstacle_return_command()
        if self.state == VehicleState.OBSTACLE_RIGHT_ALIGN:
            return self.obstacle_right_align_command()
        if self.state == VehicleState.BUS_STOP_CHECK:
            return self.min_speed, lane_steering
        if self.state == VehicleState.BUS_STOP_ENTER:
            return self.bus_stop_speed, -abs(self.bus_stop_steering_angle)
        if self.state == VehicleState.BUS_STOP_ALIGN:
            return self.bus_stop_speed, abs(self.bus_stop_steering_angle) * 0.75
        if self.state == VehicleState.BUS_STOP_EXIT:
            return self.bus_stop_speed, abs(self.bus_stop_steering_angle)
        if self.state == VehicleState.BUS_STOP_EXIT_ALIGN:
            return self.bus_stop_speed, -abs(self.bus_stop_steering_angle) * 0.75
        return self.base_speed, lane_steering

    def decision_loop(self):
        try:
            now = time.monotonic()
            self.update_state(now)
            speed, steering = self.command_for_state()

            if (
                (self.barrier_safety_stop and self._barrier_stop_allowed(now))
                or (
                    self.obstacle_detected
                    and self.obstacle_distance <= self.emergency_stop_distance
                    and self._obstacle_response_allowed(now)
                    and self.state not in {
                        VehicleState.OBSTACLE_ESCAPE,
                        VehicleState.OBSTACLE_CENTER,
                        VehicleState.OBSTACLE_RETURN,
                        VehicleState.OBSTACLE_RIGHT_ALIGN,
                    }
                )
            ):
                speed = 0.0
                steering = 0.0

            steering = float(np.clip(
                steering,
                -self.max_steering_angle,
                self.max_steering_angle,
            ))

            command = AckermannDrive()
            command.speed = float(max(speed, 0.0))
            command.steering_angle = steering
            self.ackermann_pub.publish(command)
            self.speed_pub.publish(Float32(data=command.speed))
            self.new_lateral_pub.publish(
                Float32(data=self.lateral_deviation))
            self.vehicle_state_pub.publish(
                Float32(data=float(list(VehicleState).index(self.state))))
            self.state_name_pub.publish(String(data=self.state.value))

            if self.state in {VehicleState.TURN_LEFT, VehicleState.TURN_RIGHT}:
                self.get_logger().warn(
                    f"DONUS: state={self.state.value} "
                    f"sure={self._elapsed(now):.1f}s "
                    f"speed={command.speed:.2f} steer={steering:.2f} "
                    f"intersection={self.intersection_direction}",
                    throttle_duration_sec=0.5,
                )

            if self.state in {
                VehicleState.OBSTACLE_ESCAPE,
                VehicleState.OBSTACLE_CENTER,
                VehicleState.OBSTACLE_RETURN,
                VehicleState.OBSTACLE_RIGHT_ALIGN,
            }:
                self.get_logger().warn(
                    f"SOLLAMA: state={self.state.value} "
                    f"sure={self._elapsed(now):.1f}s "
                    f"speed={command.speed:.2f} steer={steering:.2f} "
                    f"lane={self.lateral_deviation:.2f} "
                    f"heading={self.heading_deviation:.2f}",
                    throttle_duration_sec=0.5,
                )

            self.get_logger().debug(
                f"state={self.state.value} speed={command.speed:.2f} "
                f"steer={steering:.2f} lane={self.lateral_deviation:.2f} "
                f"intersection={self.intersection_direction} "
                f"turn={self.pending_turn} sign={self.last_sign_summary}")
        except Exception as exc:
            self.get_logger().error(f"Decision loop error: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
