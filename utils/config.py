"""
utils/config.py
---------------
.env dosyasından proje geneli yapılandırma değerlerini okur ve
tip güvenli dataclass'lar olarak döndürür.

Kullanım:
    from utils.config import get_serial_config, get_pid_config, get_camera_config

    serial = get_serial_config()
    print(serial.speed_port)   # '/dev/ttyACM1'

    pid = get_pid_config()
    kp = pid.kp                # 0.05
"""

import os
from dataclasses import dataclass
from utils import load_env


# ── Serial / UART ─────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class SerialConfig:
    speed_port:    str   # İtki / fren portu
    steering_port: str   # Direksiyon portu
    stop_port:     str   # Stop sinyali portu
    baud_rate:     int


def get_serial_config() -> SerialConfig:
    load_env()
    return SerialConfig(
        speed_port    = os.environ.get('SERIAL_SPEED_PORT',    '/dev/ttyACM1'),
        steering_port = os.environ.get('SERIAL_STEERING_PORT', '/dev/ttyACM0'),
        stop_port     = os.environ.get('SERIAL_STOP_PORT',     '/dev/ttyACM2'),
        baud_rate     = int(os.environ.get('SERIAL_BAUD_RATE', '38400')),
    )


# ── PID Kontrolör ─────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class PIDConfig:
    kp:              float  # Oransal kazanç
    ki:              float  # İntegral kazanç
    kd:              float  # Türevsel kazanç
    integral_clamp:  float  # İntegral windup sınırı (±)


def get_pid_config() -> PIDConfig:
    load_env()
    return PIDConfig(
        kp             = float(os.environ.get('PID_KP',             '0.05')),
        ki             = float(os.environ.get('PID_KI',             '0.5')),
        kd             = float(os.environ.get('PID_KD',             '1.0')),
        integral_clamp = float(os.environ.get('PID_INTEGRAL_CLAMP', '0.3')),
    )


# ── Kamera ────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class CameraConfig:
    resolution: str   # 'HD720' | 'HD1080' | 'HD2K' | 'VGA'
    fps:        int


def get_camera_config() -> CameraConfig:
    load_env()
    return CameraConfig(
        resolution = os.environ.get('CAMERA_RESOLUTION', 'HD720'),
        fps        = int(os.environ.get('CAMERA_FPS', '30')),
    )


# ── OpenCV Görüntü ────────────────────────────────────────────────────────────

def is_cv_display_enabled() -> bool:
    """
    CV_DISPLAY=true  → imshow pencereleri açılır (geliştirme)
    CV_DISPLAY=false → headless, pencere açılmaz (yarışma / sunucu)
    """
    load_env()
    return os.environ.get('CV_DISPLAY', 'true').strip().lower() == 'true'


# ── Web Arayüzü ───────────────────────────────────────────────────────────────

def get_backend_port() -> int:
    load_env()
    return int(os.environ.get('BACKEND_PORT', '3001'))
