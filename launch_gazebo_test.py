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
  2. SignDetection        — /zed2i_rgb/image_raw → /sign_detection/output
  3. TrackDriver          — /serit_takip_kamerasi/image_raw + /lidar → /cmd_vel

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
        "ros_args": [],
        "remaps": [],
        "extra_dirs": [BASE_DIR / 'SignDetection'],
        "note": "/zed2i_rgb/image_raw → /sign_detection/output",
    },
    {
        "name": "GAZEBO TRACK DRIVER",
        "file_path": str(BASE_DIR / 'Gazebo' / 'gazebo_track_driver.py'),
        "delay_after": 1,
        "ros_args": [
            'camera_topic:=/serit_takip_kamerasi/image_raw',
            'scan_topic:=/lidar',
            'cmd_vel_topic:=/cmd_vel',
            'base_speed:=0.28',
            'min_speed:=0.14',
            'lane_gain:=1.05',
            'barrier_gain:=0.50',
            'max_angular_z:=1.25',
            'left_turn_gain:=1.0',
            'right_turn_gain:=1.0',
            'front_emergency_distance:=0.40',
            'front_slow_distance:=3.0',
            'front_angle_range_deg:=70.0',
            'hard_stop_distance:=0.30',
            'emergency_turn_speed:=0.0',
            'avoidance_speed:=0.22',
            'avoidance_phase_duration:=12.0',
            'avoidance_center_duration:=24.0',
            'avoidance_center_speed:=0.22',
            'avoidance_turn_angular:=0.85',
            'avoidance_return_angular:=0.85',
            'avoidance_return_duration:=20.0',
            'return_hold_duration:=10.0',
            'return_min_duration:=1.2',
            'return_center_tolerance:=0.05',
            'return_speed:=0.18',
            'return_turn_ratio:=1.0',
            'return_max_angular_z:=2.0',
            'return_turn_sign:=-1.0',
            'pose_maneuver_enabled:=true',
            'pose_maneuver_x:=23.243183',
            'pose_maneuver_y:=47.263049',
            'pose_maneuver_radius:=1.5',
            'pose_maneuver_turn_duration:=20.0',
            'pose_maneuver_center_duration:=20.0',
            'barrier_safe_distance:=0.85',
            'barrier_influence_distance:=1.6',
        ],
        "remaps": [],
        "extra_dirs": [BASE_DIR / 'Gazebo'],
        "note": "/serit_takip_kamerasi/image_raw + /lidar → /cmd_vel",
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
