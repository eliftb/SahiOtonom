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
    BUS_STOP_FINAL_ALIGN = "bus_stop_final_align"
    BUS_STOP_LANE_ALIGN = "bus_stop_lane_align"
    PARK_SEARCH = "park_search"
    PARK_ROUTE = "park_route"
    PARK_SETTLE = "park_settle"
    PARK_ENTER = "park_enter"
    PARK_ALIGN = "park_align"
    PARKED = "parked"


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
        self.declare_parameter("pose_turn_right_speed", 0.18)
        self.declare_parameter("pose_turn_right_steering_angle", 0.64)
        self.declare_parameter("pose_turn_right_duration", 32.0)
        self.declare_parameter("intersection_clear_frames", 5)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_turn_enabled", False)
        self.declare_parameter("pose_turn_direction", "left")
        self.declare_parameter("pose_turn_x", 0.0)
        self.declare_parameter("pose_turn_y", 0.0)
        self.declare_parameter("pose_turn_radius", 1.2)
        self.declare_parameter("pose_turn_points", "")
        self.declare_parameter("pose_turn_align_duration", 0.0)
        self.declare_parameter("pose_turn_left_align_duration", 0.0)
        self.declare_parameter("pose_turn_right_align_duration", 0.0)
        self.declare_parameter("pose_turn_align_speed", 0.32)
        self.declare_parameter("pose_turn_align_gain_multiplier", 1.25)
        self.declare_parameter("post_turn_obstacle_ignore_duration", 10.0)

        self.declare_parameter("stop_sign_duration", 3.0)
        self.declare_parameter("sign_intent_area_ratio", 0.003)
        self.declare_parameter("sign_action_area_ratio", 0.010)
        self.declare_parameter("traffic_light_area_ratio", 0.002)
        self.declare_parameter("traffic_red_lost_release_delay", 0.0)
        self.declare_parameter("traffic_green_fresh_duration", 1.5)
        self.declare_parameter("traffic_stop_pose_enabled", False)
        self.declare_parameter("traffic_stop_x", 0.0)
        self.declare_parameter("traffic_stop_y", 0.0)
        self.declare_parameter("traffic_stop_radius", 1.2)
        self.declare_parameter("traffic_stop_points", "")
        self.declare_parameter("sign_action_cooldown", 12.0)
        self.declare_parameter("sign_turn_enabled", True)

        self.declare_parameter("obstacle_trigger_distance", 3.0)
        self.declare_parameter("emergency_stop_distance", 0.40)
        self.declare_parameter("obstacle_ignore_traffic_stop_radius", 8.0)
        self.declare_parameter("max_overtake_count", 0)
        self.declare_parameter("obstacle_start_lateral_tolerance", 0.35)
        self.declare_parameter("obstacle_start_heading_tolerance", 0.18)
        self.declare_parameter("obstacle_start_allow_intersection", False)
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
        self.declare_parameter("avoidance_right_align_duration", 0.8)
        self.declare_parameter("avoidance_right_align_timeout", 12.0)
        self.declare_parameter("avoidance_return_turn_duration", 8.0)
        self.declare_parameter("avoidance_return_counter_duration", 0.0)
        self.declare_parameter("avoidance_return_counter_ratio", 0.0)
        self.declare_parameter("avoidance_return_align_gain_multiplier", 1.40)
        self.declare_parameter("avoidance_right_align_turn_duration", 0.5)
        self.declare_parameter("avoidance_right_align_turn_angle", 0.20)

        self.declare_parameter("bus_stop_check_duration", 1.2)
        self.declare_parameter("bus_stop_clear_distance", 1.25)
        self.declare_parameter("bus_stop_clear_frames", 5)
        self.declare_parameter("bus_stop_entry_duration", 25.0)
        self.declare_parameter("bus_stop_align_duration", 5.0)
        self.declare_parameter("bus_stop_wait_duration", 18.0)
        self.declare_parameter("bus_stop_exit_duration", 3.0)
        self.declare_parameter("bus_stop_exit_align_duration", 25.0)
        self.declare_parameter("bus_stop_final_align_duration", 15.0)
        self.declare_parameter("bus_stop_lane_align_duration", 8.0)
        self.declare_parameter("bus_stop_lane_align_timeout", 18.0)
        self.declare_parameter("bus_stop_lane_align_speed", 0.22)
        self.declare_parameter("bus_stop_speed", 0.22)
        self.declare_parameter("bus_stop_steering_angle", 0.35)
        self.declare_parameter("bus_stop_entry_steering_angle", 0.33)
        self.declare_parameter("bus_stop_align_gain_multiplier", 1.35)
        self.declare_parameter("bus_stop_align_steering_limit", 0.22)
        self.declare_parameter("bus_stop_target_steering_gain", 0.55)
        self.declare_parameter("bus_stop_target_steering_limit", 0.70)
        self.declare_parameter("bus_stop_target_steer_sign", 1.0)
        self.declare_parameter("bus_stop_target_slow_distance", 2.0)
        self.declare_parameter("bus_stop_pose_enabled", False)
        self.declare_parameter("bus_stop_pose_x", 0.0)
        self.declare_parameter("bus_stop_pose_y", 0.0)
        self.declare_parameter("bus_stop_pose_radius", 1.2)
        self.declare_parameter("bus_stop_wait_pose_enabled", False)
        self.declare_parameter("bus_stop_wait_pose_x", 0.0)
        self.declare_parameter("bus_stop_wait_pose_y", 0.0)
        self.declare_parameter("bus_stop_wait_pose_radius", 1.2)
        self.declare_parameter("parking_enabled", False)
        self.declare_parameter("parking_pose_enabled", False)
        self.declare_parameter("parking_pose_x", 0.0)
        self.declare_parameter("parking_pose_y", 0.0)
        self.declare_parameter("parking_pose_radius", 1.2)
        self.declare_parameter("parking_search_timeout", 30.0)
        self.declare_parameter("parking_clear_distance", 0.85)
        self.declare_parameter("parking_clear_frames", 3)
        self.declare_parameter("parking_search_speed", 0.18)
        self.declare_parameter("parking_speed", 0.22)
        self.declare_parameter("parking_entry_duration", 70.0)
        self.declare_parameter("parking_align_duration", 4.0)
        self.declare_parameter("parking_entry_steering_angle", 0.55)
        self.declare_parameter("parking_align_steering_angle", 0.35)
        self.declare_parameter("parking_target_enabled", False)
        self.declare_parameter("parking_target_x", 0.0)
        self.declare_parameter("parking_target_y", 0.0)
        self.declare_parameter("parking_target_radius", 0.75)
        self.declare_parameter("parking_route_points", "")
        self.declare_parameter("parking_route_default_radius", 0.85)
        self.declare_parameter("parking_route_steering_gain", 0.95)
        self.declare_parameter("parking_route_steering_limit", 0.85)
        self.declare_parameter("parking_route_slow_distance", 2.2)
        self.declare_parameter("parking_min_speed", 0.10)
        self.declare_parameter("parking_settle_duration", 1.5)
        self.declare_parameter("parking_disable_obstacles", True)
        self.declare_parameter("parking_script_enabled", False)
        self.declare_parameter("parking_pre_align_duration", 0.0)
        self.declare_parameter("parking_pre_align_speed", 0.20)
        self.declare_parameter("parking_pre_align_gain_multiplier", 1.05)
        self.declare_parameter("parking_pre_align_steering_limit", 0.18)
        self.declare_parameter("parking_script_straight_duration", 9.0)
        self.declare_parameter("parking_script_shift_duration", 3.5)
        self.declare_parameter("parking_script_final_duration", 55.0)
        self.declare_parameter("parking_script_shift_steering", 0.10)
        self.declare_parameter("parking_script_right_duration", 0.0)
        self.declare_parameter("parking_script_right_steering", 0.05)
        self.declare_parameter("parking_script_shift_speed", 0.20)
        self.declare_parameter("parking_target_start_distance", 15.0)
        self.declare_parameter("parking_target_timeout", 120.0)
        self.declare_parameter("parking_target_steering_gain", 0.95)
        self.declare_parameter("parking_target_steering_limit", 0.80)
        self.declare_parameter("parking_target_steer_sign", -1.0)
        self.declare_parameter("parking_target_slow_distance", 3.0)
        self.declare_parameter("parking_target_passed_forward_margin", 0.40)
        self.declare_parameter("parking_target_passed_lateral_tolerance", 1.60)
        self.declare_parameter("parking_final_yaw_enabled", False)
        self.declare_parameter("parking_final_yaw", 0.0)
        self.declare_parameter("parking_final_yaw_tolerance", 0.10)
        self.declare_parameter("parking_final_yaw_gain", 0.80)
        self.declare_parameter("parking_align_speed", 0.22)
        self.declare_parameter("parking_line_align_gain_multiplier", 1.20)
        self.declare_parameter("parking_line_align_weight", 0.80)
        self.declare_parameter("parking_heading_align_tolerance", 0.08)
        self.declare_parameter("parking_heading_align_weight", 0.92)
        self.declare_parameter("parking_heading_align_speed_ratio", 0.75)
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
        self.pose_turn_right_speed = self._float_param("pose_turn_right_speed")
        self.pose_turn_right_steering_angle = self._float_param(
            "pose_turn_right_steering_angle")
        self.pose_turn_right_duration = self._float_param(
            "pose_turn_right_duration")
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
        self.pose_turn_points = self._parse_pose_turn_points(
            str(self.get_parameter("pose_turn_points").value or ""))
        self.pose_turn_align_duration = self._float_param(
            "pose_turn_align_duration")
        self.pose_turn_left_align_duration = self._float_param(
            "pose_turn_left_align_duration")
        self.pose_turn_right_align_duration = self._float_param(
            "pose_turn_right_align_duration")
        self.pose_turn_align_speed = self._float_param("pose_turn_align_speed")
        self.pose_turn_align_gain_multiplier = self._float_param(
            "pose_turn_align_gain_multiplier")
        self.post_turn_obstacle_ignore_duration = self._float_param(
            "post_turn_obstacle_ignore_duration")

        self.stop_sign_duration = self._float_param("stop_sign_duration")
        self.sign_intent_area_ratio = self._float_param("sign_intent_area_ratio")
        self.sign_action_area_ratio = self._float_param("sign_action_area_ratio")
        self.traffic_light_area_ratio = self._float_param("traffic_light_area_ratio")
        self.traffic_red_lost_release_delay = self._float_param(
            "traffic_red_lost_release_delay")
        self.traffic_green_fresh_duration = self._float_param(
            "traffic_green_fresh_duration")
        self.traffic_stop_pose_enabled = self._bool_param(
            "traffic_stop_pose_enabled")
        self.traffic_stop_x = self._float_param("traffic_stop_x")
        self.traffic_stop_y = self._float_param("traffic_stop_y")
        self.traffic_stop_radius = self._float_param("traffic_stop_radius")
        self.traffic_stop_points = self._parse_traffic_stop_points(
            str(self.get_parameter("traffic_stop_points").value or ""))
        self.sign_action_cooldown = self._float_param("sign_action_cooldown")
        self.sign_turn_enabled = self._bool_param("sign_turn_enabled")

        self.obstacle_trigger_distance = self._float_param("obstacle_trigger_distance")
        self.emergency_stop_distance = self._float_param("emergency_stop_distance")
        self.obstacle_ignore_traffic_stop_radius = self._float_param(
            "obstacle_ignore_traffic_stop_radius")
        self.max_overtake_count = self._int_param("max_overtake_count")
        self.obstacle_start_lateral_tolerance = self._float_param(
            "obstacle_start_lateral_tolerance")
        self.obstacle_start_heading_tolerance = self._float_param(
            "obstacle_start_heading_tolerance")
        self.obstacle_start_allow_intersection = self._bool_param(
            "obstacle_start_allow_intersection")
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
        self.bus_stop_final_align_duration = self._float_param(
            "bus_stop_final_align_duration")
        self.bus_stop_lane_align_duration = self._float_param(
            "bus_stop_lane_align_duration")
        self.bus_stop_lane_align_timeout = self._float_param(
            "bus_stop_lane_align_timeout")
        self.bus_stop_lane_align_speed = self._float_param(
            "bus_stop_lane_align_speed")
        self.bus_stop_speed = self._float_param("bus_stop_speed")
        self.bus_stop_steering_angle = self._float_param(
            "bus_stop_steering_angle")
        self.bus_stop_entry_steering_angle = self._float_param(
            "bus_stop_entry_steering_angle")
        self.bus_stop_align_gain_multiplier = self._float_param(
            "bus_stop_align_gain_multiplier")
        self.bus_stop_align_steering_limit = self._float_param(
            "bus_stop_align_steering_limit")
        self.bus_stop_target_steering_gain = self._float_param(
            "bus_stop_target_steering_gain")
        self.bus_stop_target_steering_limit = self._float_param(
            "bus_stop_target_steering_limit")
        self.bus_stop_target_steer_sign = self._float_param(
            "bus_stop_target_steer_sign")
        self.bus_stop_target_slow_distance = self._float_param(
            "bus_stop_target_slow_distance")
        self.bus_stop_pose_enabled = self._bool_param("bus_stop_pose_enabled")
        self.bus_stop_pose_x = self._float_param("bus_stop_pose_x")
        self.bus_stop_pose_y = self._float_param("bus_stop_pose_y")
        self.bus_stop_pose_radius = self._float_param("bus_stop_pose_radius")
        self.bus_stop_wait_pose_enabled = self._bool_param(
            "bus_stop_wait_pose_enabled")
        self.bus_stop_wait_pose_x = self._float_param("bus_stop_wait_pose_x")
        self.bus_stop_wait_pose_y = self._float_param("bus_stop_wait_pose_y")
        self.bus_stop_wait_pose_radius = self._float_param(
            "bus_stop_wait_pose_radius")
        self.parking_enabled = self._bool_param("parking_enabled")
        self.parking_pose_enabled = self._bool_param("parking_pose_enabled")
        self.parking_pose_x = self._float_param("parking_pose_x")
        self.parking_pose_y = self._float_param("parking_pose_y")
        self.parking_pose_radius = self._float_param("parking_pose_radius")
        self.parking_search_timeout = self._float_param("parking_search_timeout")
        self.parking_clear_distance = self._float_param("parking_clear_distance")
        self.parking_clear_frames = self._int_param("parking_clear_frames")
        self.parking_search_speed = self._float_param("parking_search_speed")
        self.parking_speed = self._float_param("parking_speed")
        self.parking_entry_duration = self._float_param("parking_entry_duration")
        self.parking_align_duration = self._float_param("parking_align_duration")
        self.parking_entry_steering_angle = self._float_param(
            "parking_entry_steering_angle")
        self.parking_align_steering_angle = self._float_param(
            "parking_align_steering_angle")
        self.parking_target_enabled = self._bool_param(
            "parking_target_enabled")
        self.parking_target_x = self._float_param("parking_target_x")
        self.parking_target_y = self._float_param("parking_target_y")
        self.parking_target_radius = self._float_param(
            "parking_target_radius")
        self.parking_route_default_radius = self._float_param(
            "parking_route_default_radius")
        self.parking_route_points = self._parse_parking_route_points(
            str(self.get_parameter("parking_route_points").value or ""))
        self.parking_route_steering_gain = self._float_param(
            "parking_route_steering_gain")
        self.parking_route_steering_limit = self._float_param(
            "parking_route_steering_limit")
        self.parking_route_slow_distance = self._float_param(
            "parking_route_slow_distance")
        self.parking_min_speed = self._float_param("parking_min_speed")
        self.parking_settle_duration = self._float_param(
            "parking_settle_duration")
        self.parking_disable_obstacles = self._bool_param(
            "parking_disable_obstacles")
        self.parking_script_enabled = self._bool_param(
            "parking_script_enabled")
        self.parking_pre_align_duration = self._float_param(
            "parking_pre_align_duration")
        self.parking_pre_align_speed = self._float_param(
            "parking_pre_align_speed")
        self.parking_pre_align_gain_multiplier = self._float_param(
            "parking_pre_align_gain_multiplier")
        self.parking_pre_align_steering_limit = self._float_param(
            "parking_pre_align_steering_limit")
        self.parking_script_straight_duration = self._float_param(
            "parking_script_straight_duration")
        self.parking_script_shift_duration = self._float_param(
            "parking_script_shift_duration")
        self.parking_script_final_duration = self._float_param(
            "parking_script_final_duration")
        self.parking_script_shift_steering = self._float_param(
            "parking_script_shift_steering")
        self.parking_script_right_duration = self._float_param(
            "parking_script_right_duration")
        self.parking_script_right_steering = self._float_param(
            "parking_script_right_steering")
        self.parking_script_shift_speed = self._float_param(
            "parking_script_shift_speed")
        self.parking_target_start_distance = self._float_param(
            "parking_target_start_distance")
        self.parking_target_timeout = self._float_param(
            "parking_target_timeout")
        self.parking_target_steering_gain = self._float_param(
            "parking_target_steering_gain")
        self.parking_target_steering_limit = self._float_param(
            "parking_target_steering_limit")
        self.parking_target_steer_sign = self._float_param(
            "parking_target_steer_sign")
        self.parking_target_slow_distance = self._float_param(
            "parking_target_slow_distance")
        self.parking_target_passed_forward_margin = self._float_param(
            "parking_target_passed_forward_margin")
        self.parking_target_passed_lateral_tolerance = self._float_param(
            "parking_target_passed_lateral_tolerance")
        self.parking_final_yaw_enabled = self._bool_param(
            "parking_final_yaw_enabled")
        self.parking_final_yaw = self._float_param("parking_final_yaw")
        self.parking_final_yaw_tolerance = self._float_param(
            "parking_final_yaw_tolerance")
        self.parking_final_yaw_gain = self._float_param(
            "parking_final_yaw_gain")
        self.parking_align_speed = self._float_param("parking_align_speed")
        self.parking_line_align_gain_multiplier = self._float_param(
            "parking_line_align_gain_multiplier")
        self.parking_line_align_weight = self._float_param(
            "parking_line_align_weight")
        self.parking_heading_align_tolerance = self._float_param(
            "parking_heading_align_tolerance")
        self.parking_heading_align_weight = self._float_param(
            "parking_heading_align_weight")
        self.parking_heading_align_speed_ratio = self._float_param(
            "parking_heading_align_speed_ratio")
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
        self.active_pose_turn_direction = None
        self.active_pose_turn_align_duration = 0.0
        self.obstacle_ignore_until = 0.0
        self.have_odom = False
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0

        self.obstacle_detected = False
        self.obstacle_distance = float("inf")
        self.obstacle_is_right = True
        self.overtake_count = 0
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
        self.bus_stop_pose_consumed = False
        self.bus_stop_task_completed = False
        self.obstacle_response_disabled = False
        self.parking_clear_count = 0
        self.parking_pose_consumed = False
        self.parking_requested = False
        self.parking_route_index = 0
        self.stop_requested = False
        self.last_action_times = {}

        self.red_light_active = False
        self.traffic_light_detection_active = False
        self.traffic_stop_consumed = False
        self.active_traffic_stop_point = None
        self.last_red_light_seen_at = 0.0
        self.last_green_light_seen_at = 0.0
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
            f"{'acik' if self.pose_turn_enabled else 'kapali'} "
            f"({len(self.pose_turn_points)} nokta) | "
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

    def _parse_pose_turn_points(self, raw_points):
        points = []
        if raw_points:
            for idx, item in enumerate(raw_points.split(";"), start=1):
                fields = [part.strip() for part in item.split(":")]
                if len(fields) == 3:
                    direction = self.pose_turn_direction
                    x_str, y_str, radius_str = fields
                elif len(fields) == 4:
                    direction, x_str, y_str, radius_str = fields
                    direction = direction.lower()
                else:
                    self.get_logger().warn(
                        f"Gecersiz pose_turn_points girdisi atlandi: {item}")
                    continue

                if direction not in {"left", "right"}:
                    self.get_logger().warn(
                        f"Gecersiz konum donusu yonu atlandi: {direction}")
                    continue

                try:
                    points.append({
                        "name": f"pose_turn_{idx}",
                        "direction": direction,
                        "x": float(x_str),
                        "y": float(y_str),
                        "radius": float(radius_str),
                        "consumed": False,
                    })
                except ValueError:
                    self.get_logger().warn(
                        f"Gecersiz pose_turn_points sayilari atlandi: {item}")

        if not points and self.pose_turn_enabled:
            points.append({
                "name": "pose_turn_legacy",
                "direction": self.pose_turn_direction,
                "x": self.pose_turn_x,
                "y": self.pose_turn_y,
                "radius": self.pose_turn_radius,
                "consumed": False,
            })

        return points

    def _parse_traffic_stop_points(self, raw_points):
        points = []
        if raw_points:
            for idx, item in enumerate(raw_points.split(";"), start=1):
                fields = [part.strip() for part in item.split(":")]
                if len(fields) != 3:
                    self.get_logger().warn(
                        f"Gecersiz traffic_stop_points girdisi atlandi: {item}")
                    continue

                try:
                    x, y, radius = (float(value) for value in fields)
                except ValueError:
                    self.get_logger().warn(
                        f"Gecersiz traffic_stop_points sayilari atlandi: {item}")
                    continue

                points.append({
                    "name": f"traffic_stop_{idx}",
                    "x": x,
                    "y": y,
                    "radius": radius,
                    "consumed": False,
                })

        if not points and self.traffic_stop_pose_enabled:
            points.append({
                "name": "traffic_stop_legacy",
                "x": self.traffic_stop_x,
                "y": self.traffic_stop_y,
                "radius": self.traffic_stop_radius,
                "consumed": False,
            })

        return points

    def _parse_parking_route_points(self, raw_points):
        points = []
        if raw_points:
            for idx, item in enumerate(raw_points.split(";"), start=1):
                fields = [part.strip() for part in item.split(":")]
                if len(fields) == 2:
                    x_str, y_str = fields
                    radius = self.parking_route_default_radius
                elif len(fields) == 3:
                    x_str, y_str, radius_str = fields
                    try:
                        radius = float(radius_str)
                    except ValueError:
                        self.get_logger().warn(
                            f"Gecersiz parking_route_points yari capi "
                            f"atlandi: {item}")
                        continue
                else:
                    self.get_logger().warn(
                        f"Gecersiz parking_route_points girdisi atlandi: {item}")
                    continue

                try:
                    points.append({
                        "name": f"park_route_{idx}",
                        "x": float(x_str),
                        "y": float(y_str),
                        "radius": radius,
                    })
                except ValueError:
                    self.get_logger().warn(
                        f"Gecersiz parking_route_points sayilari atlandi: "
                        f"{item}")

        return points

    def lateral_callback(self, msg):
        self.lateral_deviation = float(np.clip(msg.data, -1.0, 1.0))

    def heading_callback(self, msg):
        self.heading_deviation = float(np.clip(msg.data, -1.0, 1.0))

    def intersection_callback(self, msg):
        self.intersection_direction = int(msg.data)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.car_x = float(p.x)
        self.car_y = float(p.y)
        self.car_yaw = self._yaw_from_quaternion(q)
        self.have_odom = True

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

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
                elif (
                    name == "durak"
                    and not self.bus_stop_task_completed
                    and self._action_ready("durak", now)
                ):
                    self.bus_stop_requested = True
                elif (
                    name == "park"
                    and self.parking_enabled
                    and self._action_ready("park", now)
                ):
                    self.parking_requested = True

        light_seen = red_seen or green_seen
        if light_seen and not self.traffic_light_detection_active:
            self.traffic_light_detection_active = True
            self.get_logger().info(
                "Sign Detection trafik isigini gordu: isik karari aktif.")

        if self.traffic_light_detection_active and red_seen:
            self.last_red_light_seen_at = now
            self.red_light_active = True
        elif self.traffic_light_detection_active and green_seen:
            self.last_green_light_seen_at = now
            self.red_light_active = False

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
            and any(
                not point["consumed"] and point["direction"] == direction
                for point in self.pose_turn_points
            )
        )

    def _check_pose_turn(self, now):
        if (
            not self.pose_turn_enabled
            or self.pose_turn_consumed
            or not self.have_odom
            or self.state != VehicleState.NORMAL
        ):
            return False

        selected = None
        selected_distance = float("inf")
        for point in self.pose_turn_points:
            if point["consumed"]:
                continue
            distance = math.hypot(
                self.car_x - point["x"],
                self.car_y - point["y"],
            )
            if distance <= point["radius"]:
                selected = point
                selected_distance = distance
                break

        if selected is None:
            return False

        direction = selected["direction"]
        target = (
            VehicleState.TURN_LEFT
            if direction == "left"
            else VehicleState.TURN_RIGHT
        )
        selected["consumed"] = True
        self.pose_turn_consumed = all(
            point["consumed"] for point in self.pose_turn_points)
        self.pose_turn_align_pending = True
        self.active_pose_turn_direction = direction
        self.active_pose_turn_align_duration = (
            self._pose_turn_align_duration(direction))
        self.pending_turn = None
        self.pending_turn_expires_at = 0.0
        self.pending_turn_ready_at = 0.0
        self.last_action_times[f"turn_{direction}"] = now
        self._transition(
            target,
            f"konum tetigi {selected['name']} {direction}: "
            f"x={self.car_x:.2f}, y={self.car_y:.2f}, "
            f"hedefe={selected_distance:.2f}m",
        )
        return True

    def _traffic_stop_distance(self, point=None):
        if not self.have_odom:
            return float("inf")
        if point is None:
            return math.hypot(
                self.car_x - self.traffic_stop_x,
                self.car_y - self.traffic_stop_y,
            )
        return math.hypot(
            self.car_x - point["x"],
            self.car_y - point["y"],
        )

    def _next_traffic_stop_point(self):
        if not self.have_odom:
            return None, float("inf")

        for point in self.traffic_stop_points:
            if point["consumed"]:
                continue
            distance = self._traffic_stop_distance(point)
            if distance <= point["radius"]:
                return point, distance

        return None, float("inf")

    def _consume_active_traffic_stop(self):
        if self.active_traffic_stop_point is not None:
            self.active_traffic_stop_point["consumed"] = True
            self.active_traffic_stop_point = None

        if self.traffic_stop_points:
            self.traffic_stop_consumed = all(
                point["consumed"] for point in self.traffic_stop_points)
        else:
            self.traffic_stop_consumed = True

    def _green_light_fresh(self, now):
        return (
            self.last_green_light_seen_at > 0.0
            and now - self.last_green_light_seen_at
            <= self.traffic_green_fresh_duration
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

        point, distance = self._next_traffic_stop_point()
        if point is None:
            return False

        now = time.monotonic()
        if self._green_light_fresh(now) and not self.red_light_active:
            point["consumed"] = True
            self.traffic_stop_consumed = all(
                item["consumed"] for item in self.traffic_stop_points)
            self.get_logger().info(
                f"{point['name']}: yesil/serbest gecis "
                f"(hedefe={distance:.2f}m)")
            return False

        self.active_traffic_stop_point = point
        reason = "kirmizi/sari" if self.red_light_active else "isik karari bekleniyor"
        self._transition(
            VehicleState.TRAFFIC_LIGHT_STOP,
            f"{point['name']}: {reason} "
            f"(hedefe={distance:.2f}m)",
        )
        return True

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

    def _finish_bus_stop_task(self):
        self.bus_stop_task_completed = True
        self.obstacle_response_disabled = True
        self.bus_stop_requested = False
        self.bus_stop_clear_count = 0
        self.obstacle_detected = False
        self.obstacle_distance = float("inf")
        self._transition(
            VehicleState.NORMAL,
            "durak gorevi tamamlandi; durak ve engel tepkileri kapatildi",
        )

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

    def _right_parking_area_clear(self, now):
        front_fresh = (
            now - self.right_front_range_time <= self.range_fresh_timeout)
        rear_fresh = (
            now - self.right_rear_range_time <= self.range_fresh_timeout)

        if front_fresh and rear_fresh:
            return (
                self.right_front_range >= self.parking_clear_distance
                and self.right_rear_range >= self.parking_clear_distance
            )
        return self.right_barrier_distance >= self.parking_clear_distance

    def _parking_route_active(self):
        return bool(self.parking_route_points) and self.have_odom

    def _parking_current_target(self):
        if self._parking_route_active():
            index = min(
                self.parking_route_index,
                len(self.parking_route_points) - 1,
            )
            point = self.parking_route_points[index]
            return point["x"], point["y"], point["radius"], point["name"]

        if self.parking_target_enabled and self.have_odom:
            return (
                self.parking_target_x,
                self.parking_target_y,
                self.parking_target_radius,
                "park_target",
            )

        return None

    def _parking_final_target(self):
        if self.parking_route_points and self.have_odom:
            point = self.parking_route_points[-1]
            return point["x"], point["y"], point["radius"], point["name"]

        if self.parking_target_enabled and self.have_odom:
            return (
                self.parking_target_x,
                self.parking_target_y,
                self.parking_target_radius,
                "park_target",
            )

        return None

    def _parking_target_active(self):
        return self._parking_current_target() is not None

    def _parking_target_delta(self):
        target = self._parking_current_target()
        if target is None:
            return 0.0, 0.0, float("inf")

        target_x, target_y, _, _ = target
        dx = target_x - self.car_x
        dy = target_y - self.car_y
        return dx, dy, math.hypot(dx, dy)

    def _parking_target_body_delta(self):
        if not self._parking_target_active():
            return 0.0, 0.0, float("inf")

        dx, dy, distance = self._parking_target_delta()
        cos_yaw = math.cos(self.car_yaw)
        sin_yaw = math.sin(self.car_yaw)
        forward = cos_yaw * dx + sin_yaw * dy
        left = -sin_yaw * dx + cos_yaw * dy
        return forward, left, distance

    def _parking_final_target_delta(self):
        target = self._parking_final_target()
        if target is None:
            return 0.0, 0.0, float("inf")

        target_x, target_y, _, _ = target
        dx = target_x - self.car_x
        dy = target_y - self.car_y
        return dx, dy, math.hypot(dx, dy)

    def _parking_final_target_body_delta(self):
        target = self._parking_final_target()
        if target is None:
            return 0.0, 0.0, float("inf")

        dx, dy, distance = self._parking_final_target_delta()
        cos_yaw = math.cos(self.car_yaw)
        sin_yaw = math.sin(self.car_yaw)
        forward = cos_yaw * dx + sin_yaw * dy
        left = -sin_yaw * dx + cos_yaw * dy
        return forward, left, distance

    def _parking_target_distance(self):
        if not self._parking_target_active():
            return float("inf")
        _, _, distance = self._parking_target_delta()
        return distance

    def _parking_target_reached(self):
        target = self._parking_current_target()
        if target is None:
            return False
        _, _, radius, _ = target
        return (
            self._parking_target_active()
            and self._parking_target_distance() <= radius
        )

    def _parking_script_total_duration(self):
        return (
            max(self.parking_script_straight_duration, 0.0)
            + max(self.parking_script_shift_duration, 0.0)
            + max(self.parking_script_right_duration, 0.0)
            + max(self.parking_script_final_duration, 0.0)
        )

    def _parking_script_phase(self, elapsed=None):
        if elapsed is None:
            elapsed = self._elapsed(time.monotonic())

        straight_1 = max(self.parking_script_straight_duration, 0.0)
        shift = max(self.parking_script_shift_duration, 0.0)
        right = max(self.parking_script_right_duration, 0.0)
        pre_align = min(
            max(self.parking_pre_align_duration, 0.0),
            straight_1,
        )
        if elapsed < pre_align:
            return "lane_align"
        if elapsed < straight_1:
            return "straight_1"
        if elapsed < straight_1 + shift:
            return "shift_left"
        if elapsed < straight_1 + shift + right:
            return "shift_right"
        return "straight_2"

    def _parking_script_done(self, elapsed):
        total_duration = self._parking_script_total_duration()
        target = self._parking_final_target()
        if target is None:
            return elapsed >= total_duration

        _, _, radius, _ = target
        _, _, distance = self._parking_final_target_body_delta()
        if distance <= radius:
            return True

        return elapsed >= total_duration

    def _update_parking_route_progress(self, elapsed=None):
        if self.parking_script_enabled:
            if elapsed is None:
                elapsed = self._elapsed(time.monotonic())
            return self._parking_script_done(elapsed)

        if not self._parking_target_active():
            return False

        while self._parking_target_reached():
            if (
                self._parking_route_active()
                and self.parking_route_index < len(self.parking_route_points) - 1
            ):
                reached = self.parking_route_points[self.parking_route_index]
                self.parking_route_index += 1
                target = self.parking_route_points[self.parking_route_index]
                self.get_logger().warn(
                    f"PARK: {reached['name']} gecildi, "
                    f"siradaki hedef={target['name']} "
                    f"({target['x']:.2f}, {target['y']:.2f})",
                    throttle_duration_sec=0.5,
                )
                continue
            return True

        return False

    def _parking_target_yaw_error(self):
        if not self._parking_target_active():
            return 0.0
        dx, dy, _ = self._parking_target_delta()
        target_yaw = math.atan2(dy, dx)
        return self._normalize_angle(target_yaw - self.car_yaw)

    def _parking_final_yaw_error(self):
        return self._normalize_angle(self.parking_final_yaw - self.car_yaw)

    def _check_bus_stop_pose(self):
        if (
            not self.bus_stop_pose_enabled
            or self.bus_stop_task_completed
            or self.bus_stop_pose_consumed
            or not self.have_odom
            or self.state != VehicleState.NORMAL
        ):
            return False

        distance = math.hypot(
            self.car_x - self.bus_stop_pose_x,
            self.car_y - self.bus_stop_pose_y,
        )
        if distance > self.bus_stop_pose_radius:
            return False

        self.bus_stop_pose_consumed = True
        self.bus_stop_requested = False
        self.bus_stop_clear_count = 0
        self._transition(
            VehicleState.BUS_STOP_ENTER,
            f"durak konumu tetiklendi: "
            f"x={self.car_x:.2f}, y={self.car_y:.2f}, "
            f"hedefe={distance:.2f}m, saga kayis",
        )
        return True

    def _check_parking_pose(self):
        if (
            not self.parking_enabled
            or not self.parking_pose_enabled
            or self.parking_pose_consumed
            or not self.have_odom
            or self.state != VehicleState.NORMAL
        ):
            return False

        distance = math.hypot(
            self.car_x - self.parking_pose_x,
            self.car_y - self.parking_pose_y,
        )
        if distance > self.parking_pose_radius:
            return False

        self.parking_pose_consumed = True
        self.parking_clear_count = 0
        self.parking_route_index = 0
        self.parking_requested = False
        self.last_action_times["park"] = time.monotonic()
        self._transition(
            VehicleState.PARK_ROUTE,
            f"PARK_ROUTE basladi: "
            f"x={self.car_x:.2f}, y={self.car_y:.2f}, "
            f"hedefe={distance:.2f}m",
        )
        return True

    def _bus_stop_wait_pose_reached(self):
        if not self.bus_stop_wait_pose_enabled or not self.have_odom:
            return False

        distance = math.hypot(
            self.car_x - self.bus_stop_wait_pose_x,
            self.car_y - self.bus_stop_wait_pose_y,
        )
        return distance <= self.bus_stop_wait_pose_radius

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
        self.overtake_count += 1
        self._transition(
            VehicleState.OBSTACLE_ESCAPE,
            f"engel {self.obstacle_distance:.2f}m, sag seritten SOL seride kacis "
            f"(sollama {self.overtake_count}/"
            f"{self.max_overtake_count if self.max_overtake_count > 0 else '∞'})",
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

    def _pose_turn_align_duration(self, direction):
        if direction == "left":
            return self.pose_turn_left_align_duration
        if direction == "right":
            return self.pose_turn_right_align_duration
        return self.pose_turn_align_duration

    def _turn_duration(self, direction):
        if (
            direction == "right"
            and self.active_pose_turn_direction == "right"
            and self.pose_turn_align_pending
        ):
            return self.pose_turn_right_duration
        if direction == "right":
            return self.right_turn_duration
        if direction == "left":
            return self.left_turn_duration
        return self.turn_max_duration

    def _obstacle_response_allowed(self, now):
        return now >= self.obstacle_ignore_until

    def _parking_control_state(self):
        return self.state in {
            VehicleState.PARK_SEARCH,
            VehicleState.PARK_ROUTE,
            VehicleState.PARK_SETTLE,
            VehicleState.PARK_ENTER,
            VehicleState.PARK_ALIGN,
            VehicleState.PARKED,
        }

    def _near_traffic_stop_zone(self):
        """Isik direkleri lidar tarafindan hayalet engel olarak gorulebilir;
        isik dur noktalarinin cevresinde sollama baslatilmaz."""
        if (
            not self.have_odom
            or self.obstacle_ignore_traffic_stop_radius <= 0.0
        ):
            return False
        for point in self.traffic_stop_points:
            distance = math.hypot(
                self.car_x - point["x"],
                self.car_y - point["y"],
            )
            if distance <= self.obstacle_ignore_traffic_stop_radius:
                return True
        return False

    def _obstacle_start_allowed(self, now):
        if not self._obstacle_response_allowed(now):
            return False
        if (
            self.max_overtake_count > 0
            and self.overtake_count >= self.max_overtake_count
        ):
            self.get_logger().warn(
                "Engel tetigi yoksayildi: sollama hakki doldu "
                f"({self.overtake_count}/{self.max_overtake_count}).",
                throttle_duration_sec=2.0,
            )
            return False
        if self._near_traffic_stop_zone():
            self.get_logger().warn(
                "Engel tetigi yoksayildi: trafik isigi bolgesi "
                f"(x={self.car_x:.2f}, y={self.car_y:.2f}).",
                throttle_duration_sec=1.0,
            )
            return False
        if (
            not self.obstacle_start_allow_intersection
            and self.intersection_direction != 0
        ):
            self.get_logger().warn(
                "Engel tetigi bekletildi: kavsak/ayrim bolgesinde.",
                throttle_duration_sec=1.0,
            )
            return False
        if abs(self.lateral_deviation) > self.obstacle_start_lateral_tolerance:
            self.get_logger().warn(
                "Engel tetigi bekletildi: arac serit merkezinde degil "
                f"(lane={self.lateral_deviation:.2f}).",
                throttle_duration_sec=1.0,
            )
            return False
        if abs(self.heading_deviation) > self.obstacle_start_heading_tolerance:
            self.get_logger().warn(
                "Engel tetigi bekletildi: arac acisi henuz oturmadi "
                f"(heading={self.heading_deviation:.2f}).",
                throttle_duration_sec=1.0,
            )
            return False
        return True

    def _barrier_stop_allowed(self, now):
        if not self._obstacle_response_allowed(now):
            return False
        if self.parking_disable_obstacles and self._parking_control_state():
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
            if elapsed >= self._turn_duration("left"):
                self.pending_turn = None
                self.pending_turn_ready_at = 0.0
                self._start_post_turn_obstacle_ignore(now)
                if (
                    self.pose_turn_align_pending
                    and self.active_pose_turn_align_duration > 0.0
                ):
                    self._transition(
                        VehicleState.POSE_TURN_ALIGN,
                        "sol donus bitti, serit takiple toparlanma",
                    )
                else:
                    self.pose_turn_align_pending = False
                    self.active_pose_turn_direction = None
                    self.active_pose_turn_align_duration = 0.0
                    self._transition(
                        VehicleState.NORMAL,
                        "sol donus tamamlandi",
                    )
            return

        if self.state == VehicleState.TURN_RIGHT:
            if elapsed >= self._turn_duration("right"):
                self.pending_turn = None
                self.pending_turn_ready_at = 0.0
                self._start_post_turn_obstacle_ignore(now)
                if (
                    self.pose_turn_align_pending
                    and self.active_pose_turn_align_duration > 0.0
                ):
                    self._transition(
                        VehicleState.POSE_TURN_ALIGN,
                        "sag donus bitti, serit takiple toparlanma",
                    )
                else:
                    self.pose_turn_align_pending = False
                    self.active_pose_turn_direction = None
                    self.active_pose_turn_align_duration = 0.0
                    self._transition(
                        VehicleState.NORMAL,
                        "sag donus tamamlandi",
                    )
            return

        if self.state == VehicleState.POSE_TURN_ALIGN:
            if elapsed >= self.active_pose_turn_align_duration:
                self.pose_turn_align_pending = False
                self.active_pose_turn_direction = None
                self.active_pose_turn_align_duration = 0.0
                self._start_post_turn_obstacle_ignore(now)
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
            if self._green_light_fresh(now) and not self.red_light_active:
                self._consume_active_traffic_stop()
                self._transition(VehicleState.NORMAL, "yesil isik")
            elif (
                self.traffic_red_lost_release_delay > 0.0
                and self.last_red_light_seen_at > 0.0
                and now - self.last_red_light_seen_at
                >= self.traffic_red_lost_release_delay
            ):
                self.red_light_active = False
                self._consume_active_traffic_stop()
                self._transition(
                    VehicleState.NORMAL,
                    "kirmizi kayboldu, 5s sonra gecis",
                )
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
            min_align_duration = max(
                self.avoidance_right_align_duration,
                self.avoidance_right_align_turn_duration,
            )
            if elapsed >= min_align_duration and centered:
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
                    "durak icinde araci duzleme",
                )
            return

        if self.state == VehicleState.BUS_STOP_ALIGN:
            if elapsed >= self.bus_stop_align_duration:
                self._transition(
                    VehicleState.BUS_STOP_EXIT,
                    "durak cikisi: once duz ilerle",
                )
            return

        if self.state == VehicleState.BUS_STOP_WAIT:
            if elapsed >= self.bus_stop_wait_duration:
                self._transition(
                    VehicleState.BUS_STOP_FINAL_ALIGN,
                    "bekleme bitti, ikinci acili serit donusu",
                )
            return

        if self.state == VehicleState.BUS_STOP_EXIT:
            if elapsed >= self.bus_stop_exit_duration:
                self._transition(
                    VehicleState.BUS_STOP_EXIT_ALIGN,
                    "seride ilk acili donus",
                )
            return

        if self.state == VehicleState.BUS_STOP_EXIT_ALIGN:
            if elapsed >= self.bus_stop_exit_align_duration:
                if self.bus_stop_wait_duration > 0.0:
                    self._transition(
                        VehicleState.BUS_STOP_WAIT,
                        f"ilk {self.bus_stop_exit_align_duration:.0f}s "
                        "acili donus bitti, "
                        f"{self.bus_stop_wait_duration:.0f}s bekle",
                    )
                else:
                    self._transition(
                        VehicleState.BUS_STOP_FINAL_ALIGN,
                        f"ikinci {self.bus_stop_final_align_duration:.0f}s "
                        "acili serit donusu",
                    )
            return

        if self.state == VehicleState.BUS_STOP_FINAL_ALIGN:
            if elapsed >= self.bus_stop_final_align_duration:
                if self.bus_stop_lane_align_duration > 0.0:
                    self._transition(
                        VehicleState.BUS_STOP_LANE_ALIGN,
                        "son donus bitti, serit takibiyle toparlan",
                    )
                else:
                    self._finish_bus_stop_task()
            return

        if self.state == VehicleState.BUS_STOP_LANE_ALIGN:
            centered = self._lane_is_centered_for_overtake()
            if elapsed >= self.bus_stop_lane_align_duration and centered:
                self._finish_bus_stop_task()
            elif (
                self.bus_stop_lane_align_timeout > 0.0
                and elapsed >= self.bus_stop_lane_align_timeout
            ):
                self._finish_bus_stop_task()
            return

        if self.state in {
            VehicleState.PARK_SEARCH,
            VehicleState.PARK_ENTER,
            VehicleState.PARK_ALIGN,
        }:
            self._transition(
                VehicleState.PARK_ROUTE,
                "eski park state'i sade waypoint rotasina alindi",
            )
            return

        if self.state == VehicleState.PARK_ROUTE:
            if not self._parking_target_active():
                self.get_logger().warn(
                    "PARK_ROUTE: odom veya park hedefi bekleniyor.",
                    throttle_duration_sec=1.0,
                )
                return

            if self._update_parking_route_progress(elapsed):
                self._transition(
                    VehicleState.PARK_SETTLE,
                    "son park noktasina ulasildi, park sabitleniyor",
                )
            return

        if self.state == VehicleState.PARK_SETTLE:
            if elapsed >= max(self.parking_settle_duration, 0.0):
                self._transition(
                    VehicleState.PARKED,
                    "park tamamlandi",
                )
            return

        if self.state == VehicleState.PARKED:
            return

        if self._check_traffic_light_stop():
            return
        elif self.stop_requested:
            self.stop_requested = False
            self.last_action_times["dur"] = now
            self._transition(VehicleState.STOP_SIGN_WAIT, "dur levhasi")
        elif self.bus_stop_requested and not self.bus_stop_task_completed:
            self.bus_stop_requested = False
            self.bus_stop_clear_count = 0
            self._transition(
                VehicleState.BUS_STOP_CHECK,
                "durak alani doluluk kontrolu",
            )
        elif self.bus_stop_requested:
            self.bus_stop_requested = False
        elif self._check_bus_stop_pose():
            return
        elif self._check_parking_pose():
            return
        elif self.parking_requested and self.parking_enabled:
            self.parking_requested = False
            self.parking_clear_count = 0
            self.parking_route_index = 0
            self.last_action_times["park"] = now
            self._transition(
                VehicleState.PARK_ROUTE,
                "park tabelasi algilandi, waypoint rotasi basladi",
            )
        elif self.parking_requested:
            self.parking_requested = False
        elif self._check_pose_turn(now):
            return
        elif (
            not self.obstacle_response_disabled
            and self.obstacle_detected
            and self.obstacle_distance <= self.obstacle_trigger_distance
            and self._obstacle_start_allowed(now)
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

    def bus_stop_align_command(self):
        return self.bus_stop_speed, 0.0

    def bus_stop_enter_command(self):
        return self.bus_stop_speed, -abs(self.bus_stop_entry_steering_angle)

    def bus_stop_exit_command(self):
        return self.bus_stop_speed, 0.0

    def bus_stop_exit_align_command(self):
        return self.bus_stop_speed, abs(self.bus_stop_steering_angle)

    def bus_stop_final_align_command(self):
        return self.bus_stop_exit_align_command()

    def bus_stop_lane_align_command(self):
        steering = self.lane_steering(
            correct_heading=True,
            gain_multiplier=self.bus_stop_align_gain_multiplier,
        )
        steering = float(np.clip(
            steering,
            -self.bus_stop_align_steering_limit,
            self.bus_stop_align_steering_limit,
        ))
        return self.bus_stop_lane_align_speed, steering

    def bus_stop_target_command(self):
        if not self.bus_stop_wait_pose_enabled or not self.have_odom:
            return self.bus_stop_align_command()

        dx = self.bus_stop_wait_pose_x - self.car_x
        dy = self.bus_stop_wait_pose_y - self.car_y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self.car_yaw)
        steering = yaw_error * self.bus_stop_target_steering_gain
        steering *= self.bus_stop_target_steer_sign
        steering = float(np.clip(
            steering,
            -self.bus_stop_target_steering_limit,
            self.bus_stop_target_steering_limit,
        ))

        speed = self.bus_stop_speed
        if distance <= self.bus_stop_target_slow_distance:
            speed = min(speed, self.bus_stop_speed * 0.70)
        if abs(yaw_error) > 1.35:
            speed = min(speed, self.bus_stop_speed * 0.75)
        return speed, steering

    def parking_search_command(self):
        return self.parking_search_speed, 0.0

    def parking_target_command(self, allow_slowdown=True):
        if not self._parking_target_active():
            return self.parking_speed, -abs(self.parking_entry_steering_angle)

        forward, left, distance = self._parking_target_body_delta()

        lookahead = max(abs(forward), 1.0)
        steering = (left / lookahead) * self.parking_target_steering_gain
        steering *= self.parking_target_steer_sign
        steering = float(np.clip(
            steering,
            -self.parking_target_steering_limit,
            self.parking_target_steering_limit,
        ))

        speed = self.parking_speed
        if allow_slowdown:
            if distance <= self.parking_target_slow_distance:
                speed = min(speed, self.parking_speed * 0.60)
            if distance <= max(self.parking_target_radius * 2.0, 0.75):
                speed = min(speed, self.parking_speed * 0.45)
            if abs(left) > 2.0 or forward < 0.4:
                speed = min(speed, self.parking_speed * 0.75)
        return speed, steering

    def parking_route_command(self):
        if self.parking_script_enabled:
            return self.parking_script_command()

        target = self._parking_current_target()
        if target is None:
            return self.parking_speed, 0.0

        _, _, radius, _ = target
        forward, left, distance = self._parking_target_body_delta()
        lookahead = max(abs(forward), 0.85)
        steering = math.atan2(left, lookahead) * self.parking_route_steering_gain
        steering = float(np.clip(
            steering,
            -self.parking_route_steering_limit,
            self.parking_route_steering_limit,
        ))

        speed = self.parking_speed
        if distance <= self.parking_route_slow_distance:
            speed = min(
                speed,
                max(self.parking_min_speed, self.parking_speed * 0.62),
            )
        if distance <= max(radius * 2.0, 0.90):
            speed = min(
                speed,
                max(self.parking_min_speed, self.parking_speed * 0.45),
            )
        if forward < 0.35:
            speed = min(
                speed,
                max(self.parking_min_speed, self.parking_speed * 0.50),
            )

        return speed, steering

    def parking_script_command(self):
        elapsed = self._elapsed(time.monotonic())
        phase = self._parking_script_phase(elapsed)

        speed = self.parking_speed
        steering = 0.0
        if phase == "lane_align":
            speed = min(self.parking_speed, self.parking_pre_align_speed)
            steering = self.lane_steering(
                correct_heading=True,
                gain_multiplier=self.parking_pre_align_gain_multiplier,
            )
            steering = float(np.clip(
                steering,
                -self.parking_pre_align_steering_limit,
                self.parking_pre_align_steering_limit,
            ))
        elif phase == "shift_left":
            speed = min(self.parking_speed, self.parking_script_shift_speed)
            steering = abs(self.parking_script_shift_steering)
        elif phase == "shift_right":
            speed = min(self.parking_speed, self.parking_script_shift_speed)
            steering = -abs(self.parking_script_right_steering)

        target = self._parking_final_target()
        if target is not None:
            _, _, radius, _ = target
            forward, _, distance = self._parking_final_target_body_delta()
            if distance <= self.parking_route_slow_distance:
                speed = min(
                    speed,
                    max(self.parking_min_speed, self.parking_speed * 0.60),
                )
            if distance <= max(radius * 2.0, 0.90):
                speed = min(
                    speed,
                    max(self.parking_min_speed, self.parking_speed * 0.45),
                )
            if phase != "straight_2" and forward < 0.35:
                speed = min(
                    speed,
                    max(self.parking_min_speed, self.parking_speed * 0.50),
                )

        return speed, float(np.clip(
            steering,
            -self.parking_route_steering_limit,
            self.parking_route_steering_limit,
        ))

    def parking_yaw_then_target_command(self, turn_speed):
        if not (self._parking_target_active() and self.parking_final_yaw_enabled):
            return None

        yaw_error = self._parking_final_yaw_error()
        if abs(yaw_error) > self.parking_final_yaw_tolerance:
            steering = yaw_error * self.parking_final_yaw_gain
            steering *= self.parking_target_steer_sign
            steering = float(np.clip(
                steering,
                -self.parking_target_steering_limit,
                self.parking_target_steering_limit,
            ))
            return turn_speed, steering

        return self.parking_target_command(allow_slowdown=False)

    def parking_enter_command(self):
        if self._parking_route_active():
            return self.parking_route_command()
        if self._parking_target_active():
            return self.parking_target_command(allow_slowdown=False)
        return self.parking_speed, -abs(self.parking_entry_steering_angle)

    def parking_line_align_command(self):
        target_speed, target_steering = self.parking_target_command(
            allow_slowdown=False)
        line_steering = self.lane_steering(
            correct_heading=True,
            gain_multiplier=self.parking_line_align_gain_multiplier,
        )
        line_weight = float(np.clip(
            self.parking_line_align_weight,
            0.0,
            1.0,
        ))
        if abs(self.heading_deviation) > self.parking_heading_align_tolerance:
            line_weight = max(
                line_weight,
                float(np.clip(self.parking_heading_align_weight, 0.0, 1.0)),
            )

        steering = (
            line_steering * line_weight
            + target_steering * (1.0 - line_weight)
        )
        steering = float(np.clip(
            steering,
            -self.parking_target_steering_limit,
            self.parking_target_steering_limit,
        ))
        speed = min(target_speed, self.parking_align_speed)
        if abs(self.heading_deviation) > self.parking_heading_align_tolerance:
            speed *= float(np.clip(
                self.parking_heading_align_speed_ratio,
                0.20,
                1.0,
            ))
        return speed, steering

    def parking_align_command(self):
        if self._parking_route_active():
            return self.parking_route_command()
        yaw_target = self.parking_yaw_then_target_command(
            self.parking_align_speed)
        if yaw_target is not None:
            return yaw_target
        if self._parking_target_active():
            return self.parking_line_align_command()
        return self.parking_speed, abs(self.parking_align_steering_angle)

    def command_for_state(self):
        lane_steering = self.lane_steering(
            correct_heading=self.normal_lane_correct_heading,
            gain_multiplier=self.normal_lane_gain_multiplier,
        )

        if self.state == VehicleState.TURN_LEFT:
            return self.left_turn_speed, abs(self.left_turn_steering_angle)
        if self.state == VehicleState.TURN_RIGHT:
            if (
                self.active_pose_turn_direction == "right"
                and self.pose_turn_align_pending
            ):
                return (
                    self.pose_turn_right_speed,
                    -abs(self.pose_turn_right_steering_angle),
                )
            return self.right_turn_speed, -abs(self.right_turn_steering_angle)
        if self.state == VehicleState.POSE_TURN_ALIGN:
            if self.active_pose_turn_direction == "right":
                return (
                    self.pose_turn_align_speed,
                    self.lane_steering(
                        correct_heading=self.normal_lane_correct_heading,
                        gain_multiplier=self.normal_lane_gain_multiplier,
                    ),
                )
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
            return self.bus_stop_enter_command()
        if self.state == VehicleState.BUS_STOP_ALIGN:
            return self.bus_stop_align_command()
        if self.state == VehicleState.BUS_STOP_EXIT:
            return self.bus_stop_exit_command()
        if self.state == VehicleState.BUS_STOP_EXIT_ALIGN:
            return self.bus_stop_exit_align_command()
        if self.state == VehicleState.BUS_STOP_FINAL_ALIGN:
            return self.bus_stop_final_align_command()
        if self.state == VehicleState.BUS_STOP_LANE_ALIGN:
            return self.bus_stop_lane_align_command()
        if self.state in {
            VehicleState.PARK_SEARCH,
            VehicleState.PARK_ENTER,
            VehicleState.PARK_ALIGN,
            VehicleState.PARK_ROUTE,
        }:
            return self.parking_route_command()
        if self.state in {VehicleState.PARK_SETTLE, VehicleState.PARKED}:
            return 0.0, 0.0
        return self.base_speed, lane_steering

    def decision_loop(self):
        try:
            now = time.monotonic()
            self.update_state(now)
            speed, steering = self.command_for_state()

            if (
                not self.obstacle_response_disabled
                and (
                    (
                        self.barrier_safety_stop
                        and self._barrier_stop_allowed(now)
                    )
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
                        and not (
                            self.parking_disable_obstacles
                            and self._parking_control_state()
                        )
                    )
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

            if self.state in {
                VehicleState.BUS_STOP_ENTER,
                VehicleState.BUS_STOP_ALIGN,
                VehicleState.BUS_STOP_WAIT,
                VehicleState.BUS_STOP_EXIT,
                VehicleState.BUS_STOP_EXIT_ALIGN,
                VehicleState.BUS_STOP_FINAL_ALIGN,
                VehicleState.BUS_STOP_LANE_ALIGN,
            }:
                self.get_logger().warn(
                    f"DURAK: state={self.state.value} "
                    f"sure={self._elapsed(now):.1f}s "
                    f"speed={command.speed:.2f} steer={steering:.2f} "
                    f"lane={self.lateral_deviation:.2f} "
                    f"heading={self.heading_deviation:.2f}",
                    throttle_duration_sec=0.5,
                )

            if self.state in {
                VehicleState.PARK_SEARCH,
                VehicleState.PARK_ROUTE,
                VehicleState.PARK_SETTLE,
                VehicleState.PARK_ENTER,
                VehicleState.PARK_ALIGN,
                VehicleState.PARKED,
            }:
                target_text = ""
                if self.parking_script_enabled:
                    target = self._parking_final_target()
                    if target is not None:
                        forward, left, distance = (
                            self._parking_final_target_body_delta())
                        target_name = target[3]
                        target_text = (
                            f" phase={self._parking_script_phase(self._elapsed(now))}"
                            f" hedef={target_name}"
                            f" target={distance:.2f}m"
                            f" forward={forward:.2f} left={left:.2f}"
                        )
                elif self._parking_target_active():
                    forward, left, distance = self._parking_target_body_delta()
                    target = self._parking_current_target()
                    target_name = target[3] if target is not None else "park_target"
                    target_text = (
                        f" hedef={target_name}"
                        f" target={distance:.2f}m"
                        f" forward={forward:.2f} left={left:.2f}"
                    )
                    if self.parking_final_yaw_enabled:
                        target_text += (
                            f" yaw_err={self._parking_final_yaw_error():.2f}"
                        )
                self.get_logger().warn(
                    f"PARK: state={self.state.value} "
                    f"sure={self._elapsed(now):.1f}s "
                    f"speed={command.speed:.2f} steer={steering:.2f} "
                    f"right_front={self.right_front_range:.2f} "
                    f"right_rear={self.right_rear_range:.2f} "
                    f"lane={self.lateral_deviation:.2f} "
                    f"heading={self.heading_deviation:.2f}"
                    f"{target_text}",
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
