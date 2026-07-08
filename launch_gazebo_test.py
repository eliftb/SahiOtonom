"""
Gazebo Simülasyon Launch Script — Alzada X Car
===============================================
Alzada X Car modeline göre yapılandırılmıştır:
  - /serit_takip_kamerasi/image_raw → Şerit takip kamerası
  - /zed2i_rgb/image_raw            → ZED RGB kamera (levha tespiti)
  - /lidar                → LiDAR engel tespiti
  - /cmd_vel              → Gazebo modeli kabul eder

Başlatılan node'lar:
  1. ObstacleDetection   — /lidar → /obstacle_detected, /barrier/safety_stop
  2. SignDetection       — /zed2i_rgb/image_raw → /sign_detection/events
  3. GazeboLaneDetection — /serit_takip_kamerasi/image_raw → /lane/*
  4. DecisionMaking      — algılama topic'leri → /ackermann_cmd
  5. ActuatorBridge      — /ackermann_cmd → /cmd_vel

Kullanım:
    python3 launch_gazebo_test.py
"""

import subprocess
import time
import os
import sys
import pathlib
import signal

BASE_DIR = pathlib.Path(__file__).parent


def make_env(extra_dirs=None):
    """Her subprocess için PYTHONPATH'i ayarlar."""
    env = os.environ.copy()
    paths = [str(BASE_DIR)]
    if extra_dirs:
        paths += [str(d) for d in extra_dirs]
    existing = env.get('PYTHONPATH', '')
    if existing:
        paths.append(existing)
    env['PYTHONPATH'] = ':'.join(paths)
    return env


def build_cmd(script_path, ros_args=None, remaps=None):
    """Subprocess komutunu oluşturur; ROS2 parametre ve remap desteğiyle."""
    cmd = [sys.executable, script_path]
    has_ros = (ros_args and len(ros_args) > 0) or (remaps and len(remaps) > 0)
    if has_ros:
        cmd += ['--ros-args']
        for arg in (ros_args or []):
            cmd += ['-p', arg]
        for remap in (remaps or []):
            cmd += ['--remap', remap]
    return cmd


NODE_LAUNCH_ORDER = [
    {
        "name": "GAZEBO OBSTACLE DETECTION",
        "file_path": str(BASE_DIR / 'ObstacleDetection' / 'obstacle_detection.py'),
        "delay_after": 1,
        "ros_args": [
            'scan_topic:=/lidar',
            'obstacle_threshold:=3.0',
            'obstacle_min_distance:=0.35',
            'front_angle_range_deg:=20.0',
            'obstacle_min_points:=3',
            'obstacle_confirm_frames:=3',
            'obstacle_clear_frames:=4',
            'barrier_hard_stop_distance:=0.12',
        ],
        "remaps": [],
        "extra_dirs": [],
        "note": "/lidar → engel tespiti (Gazebo modu)",
    },
    {
        "name": "SIGN DETECTION",
        "file_path": str(BASE_DIR / 'SignDetection' / 'run_tracker.py'),
        "required_files": [
            BASE_DIR / 'SignDetection' / 'UltraConservative_BEST_mAP0.9248_20250801_115028.pt',
        ],
        "delay_after": 3,
        "ros_args": [
            'history_length:=5',
            'min_confidence_frames:=2',
            'left_turn_confirm_frames:=2',
        ],
        "remaps": [],
        "extra_dirs": [BASE_DIR / 'SignDetection'],
        "note": "/zed2i_rgb/image_raw → /sign_detection/output",
    },
    {
        "name": "GAZEBO LANE DETECTION",
        "file_path": str(BASE_DIR / 'Gazebo' / 'gazebo_lane_detection.py'),
        "delay_after": 1,
        "ros_args": [
            'camera_topic:=/serit_takip_kamerasi/image_raw',
            'white_only:=false',
            'roi_ratio:=0.45',
            'history_size:=5',
            'intersection_history_size:=5',
            'intersection_min_frames:=3',
        ],
        "remaps": [],
        "extra_dirs": [BASE_DIR / 'Gazebo'],
        "note": "/serit_takip_kamerasi/image_raw → /lane/lateral_deviation + kavşak",
    },
    {
        "name": "DECISION MAKING",
        "file_path": str(
            BASE_DIR / 'DecisionMaking' / 'decision-making-node-avoidance.py'),
        "delay_after": 1,
        "ros_args": [
            'base_speed:=0.40',
            'min_speed:=0.22',
            'lane_steering_gain:=1.05',
            'normal_lane_correct_heading:=false',
            'normal_lane_gain_multiplier:=1.0',
            'avoidance_heading_gain:=1.60',
            'max_steering_angle:=1.25',
            'turn_speed:=0.28',
            'turn_steering_angle:=0.42',
            'turn_min_duration:=1.0',
            'turn_max_duration:=5.0',
            'turn_intent_timeout:=15.0',
            'turn_action_cooldown:=12.0',
            'left_turn_speed:=0.18',
            'left_turn_steering_angle:=1.10',
            'left_turn_duration:=40.0',
            'left_turn_start_delay:=4.0',
            'right_turn_speed:=0.24',
            'right_turn_steering_angle:=0.55',
            'right_turn_duration:=5.0',
            'right_turn_start_delay:=0.0',
            'pose_turn_right_speed:=0.18',
            'pose_turn_right_steering_angle:=0.64',
            'pose_turn_right_duration:=32.0',
            'odom_topic:=/odom',
            'pose_turn_enabled:=true',
            'pose_turn_direction:=left',
            'pose_turn_x:=23.244656',
            'pose_turn_y:=50.5',
            'pose_turn_radius:=1.2',
            (
                'pose_turn_points:=left:23.244656:50.5:1.2'
                ';left:-45.126837:51.595236:1.2'
                ';left:-46.217237:3.639416:0.6'
                ';right:-0.604681:0.827706:0.7'
            ),
            'pose_turn_align_duration:=0.0',
            'pose_turn_left_align_duration:=0.0',
            'pose_turn_right_align_duration:=3.0',
            'pose_turn_align_speed:=0.22',
            'pose_turn_align_gain_multiplier:=1.0',
            'post_turn_obstacle_ignore_duration:=12.0',
            'stop_sign_duration:=3.0',
            'traffic_stop_pose_enabled:=true',
            'traffic_stop_x:=2.838900',
            'traffic_stop_y:=51.849778',
            'traffic_stop_radius:=1.2',
            (
                'traffic_stop_points:=2.838900:51.849778:1.2'
                ';-31.574237:1.016936:1.2'
            ),
            'traffic_light_area_ratio:=0.001',
            'traffic_red_lost_release_delay:=0.0',
            'traffic_green_fresh_duration:=1.5',
            'sign_turn_enabled:=false',
            'obstacle_trigger_distance:=3.0',
            'emergency_stop_distance:=0.40',
            'obstacle_start_lateral_tolerance:=0.55',
            'obstacle_start_heading_tolerance:=0.24',
            'obstacle_start_allow_intersection:=false',
            'avoidance_speed:=0.35',
            'avoidance_escape_duration:=5.0',
            'avoidance_center_duration:=10.0',
            'avoidance_center_speed:=0.45',
            'obstacle_escape_steering_angle:=0.85',
            'obstacle_return_steering_angle:=0.55',
            'avoidance_return_duration:=6.0',
            'avoidance_center_tolerance:=0.12',
            'avoidance_heading_tolerance:=0.18',
            'avoidance_center_stable_frames:=6',
            'avoidance_center_max_extra_duration:=10.0',
            'avoidance_center_gain_multiplier:=1.25',
            'avoidance_right_align_duration:=0.8',
            'avoidance_right_align_timeout:=12.0',
            'avoidance_return_turn_duration:=8.0',
            'avoidance_return_counter_duration:=0.0',
            'avoidance_return_counter_ratio:=0.0',
            'avoidance_return_align_gain_multiplier:=1.40',
            'avoidance_right_align_turn_duration:=0.5',
            'avoidance_right_align_turn_angle:=0.20',
            'bus_stop_check_duration:=1.2',
            'bus_stop_clear_distance:=1.25',
            'bus_stop_wait_duration:=18.0',
            'bus_stop_entry_duration:=25.0',
            'bus_stop_align_duration:=5.0',
            'bus_stop_exit_duration:=3.0',
            'bus_stop_exit_align_duration:=25.0',
            'bus_stop_final_align_duration:=15.0',
            'bus_stop_lane_align_duration:=8.0',
            'bus_stop_lane_align_timeout:=18.0',
            'bus_stop_lane_align_speed:=0.22',
            'bus_stop_speed:=0.22',
            'bus_stop_steering_angle:=0.35',
            'bus_stop_entry_steering_angle:=0.33',
            'bus_stop_align_gain_multiplier:=1.05',
            'bus_stop_align_steering_limit:=0.20',
            'bus_stop_target_steering_gain:=0.60',
            'bus_stop_target_steering_limit:=0.75',
            'bus_stop_target_steer_sign:=1.0',
            'bus_stop_target_slow_distance:=2.0',
            'bus_stop_pose_enabled:=true',
            'bus_stop_pose_x:=-16.549737',
            'bus_stop_pose_y:=51.813236',
            'bus_stop_pose_radius:=1.3',
            'bus_stop_wait_pose_enabled:=true',
            'bus_stop_wait_pose_x:=-21.725026',
            'bus_stop_wait_pose_y:=54.715588',
            'bus_stop_wait_pose_radius:=1.4',
            'parking_enabled:=true',
            'parking_pose_enabled:=false',
            'parking_pose_x:=1.071523',
            'parking_pose_y:=-7.404127',
            'parking_pose_radius:=2.0',
            'parking_search_timeout:=30.0',
            'parking_clear_distance:=0.85',
            'parking_clear_frames:=3',
            'parking_search_speed:=0.30',
            'parking_speed:=0.22',
            'parking_entry_duration:=70.0',
            'parking_align_duration:=4.0',
            'parking_entry_steering_angle:=0.55',
            'parking_align_steering_angle:=0.35',
            'parking_target_enabled:=true',
            'parking_target_x:=3.552593',
            'parking_target_y:=-35.650000',
            'parking_target_radius:=0.50',
            'parking_route_default_radius:=0.85',
            'parking_route_steering_gain:=0.95',
            'parking_route_steering_limit:=0.55',
            'parking_route_slow_distance:=2.2',
            'parking_min_speed:=0.10',
            'parking_settle_duration:=1.5',
            'parking_disable_obstacles:=true',
            'parking_script_enabled:=true',
            'parking_pre_align_duration:=0.0',
            'parking_pre_align_speed:=0.20',
            'parking_pre_align_gain_multiplier:=1.05',
            'parking_pre_align_steering_limit:=0.18',
            'parking_script_straight_duration:=9.0',
            'parking_script_shift_duration:=3.0',
            'parking_script_final_duration:=58.0',
            'parking_script_shift_steering:=0.07',
            'parking_script_right_duration:=0.0',
            'parking_script_right_steering:=0.0',
            'parking_script_shift_speed:=0.20',
            (
                'parking_route_points:=1.071523:-7.404127:2.00'
                ';1.277140:-11.817414:1.15'
                ';1.560242:-13.820760:1.15'
                ';1.972861:-17.866455:1.25'
                ';2.926002:-25.751907:1.35'
                ';3.716382:-32.844867:1.35'
                ';3.552593:-35.650000:0.50'
            ),
            'parking_target_start_distance:=15.0',
            'parking_target_timeout:=120.0',
            'parking_target_steering_gain:=0.75',
            'parking_target_steering_limit:=0.48',
            'parking_target_steer_sign:=-1.0',
            'parking_target_slow_distance:=2.2',
            'parking_target_passed_forward_margin:=0.80',
            'parking_target_passed_lateral_tolerance:=0.45',
            'parking_final_yaw_enabled:=false',
            'parking_final_yaw:=0.0',
            'parking_final_yaw_tolerance:=0.10',
            'parking_final_yaw_gain:=0.75',
            'parking_align_speed:=0.30',
            'parking_line_align_gain_multiplier:=1.20',
            'parking_line_align_weight:=0.80',
            'parking_heading_align_tolerance:=0.08',
            'parking_heading_align_weight:=0.92',
            'parking_heading_align_speed_ratio:=0.75',
        ],
        "remaps": [],
        "extra_dirs": [BASE_DIR / 'DecisionMaking'],
        "note": "levha + kavşak + LiDAR + ultrasonik → /ackermann_cmd",
    },
    {
        "name": "GAZEBO ACTUATOR BRIDGE",
        "file_path": str(BASE_DIR / 'Gazebo' / 'gazebo_actuator_bridge.py'),
        "delay_after": 1,
        "ros_args": [
            'cmd_vel_topic:=/cmd_vel',
            'control_mode:=simple',
            'wheelbase:=0.3',
            'max_speed:=1.0',
            'max_steering_angle:=1.25',
            'steer_sign:=1.0',
            'angular_gain:=1.0',
            'max_angular_z:=1.25',
        ],
        "remaps": [],
        "extra_dirs": [BASE_DIR / 'Gazebo'],
        "note": "/ackermann_cmd → /cmd_vel",
    },
]


if __name__ == '__main__':
    processes = []

    print("=" * 60)
    print("  ALZADA X CAR — GAZEBO SİMÜLASYON MODU")
    print("  Şerit Kamerası : /serit_takip_kamerasi/image_raw")
    print("  Levha Kamerası : /zed2i_rgb/image_raw")
    print("  LiDAR          : /lidar")
    print("  Kontrol        : /cmd_vel")
    print("=" * 60)
    print()

    try:
        for node_info in NODE_LAUNCH_ORDER:
            name     = node_info["name"]
            path     = node_info["file_path"]
            delay    = node_info["delay_after"]
            ros_args = node_info.get("ros_args", [])
            remaps   = node_info.get("remaps", [])
            note     = node_info.get("note", "")
            extra    = node_info.get("extra_dirs", [])

            if not os.path.exists(path):
                print(f"\n⚠️  '{name}' dosyası bulunamadı, atlanıyor: {path}")
                continue

            missing_required = [
                str(required_file)
                for required_file in node_info.get("required_files", [])
                if not pathlib.Path(required_file).exists()
            ]
            if missing_required:
                print(f"\n⚠️  '{name}' gerekli dosya eksik olduğu için atlanıyor:")
                for missing_file in missing_required:
                    print(f"   - {missing_file}")
                continue

            print(f"\n▶  {name}")
            print(f"   {note}")

            cmd = build_cmd(path, ros_args, remaps)
            env = make_env(extra)

            p = subprocess.Popen(cmd, env=env)
            processes.append(p)
            print(f"   PID {p.pid} | {delay}s bekleniyor...")
            time.sleep(delay)

        print("\n" + "=" * 60)
        print("  Tüm node'lar çalışıyor. Durdurmak için CTRL+C")
        print("=" * 60)

        for p in processes:
            p.wait()

    except KeyboardInterrupt:
        print("\n--- KAPATMA SİNYALİ ---")
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        time.sleep(2)
        for p in processes:
            if p.poll() is None:
                p.kill()
        print("Tüm işlemler durduruldu.")
