"""
utils/ros_logger.py
-------------------
Proje genelinde loglama seviyesini .env dosyasından okuyarak ROS2 node'larına uygular.

Kullanım:
    from utils.ros_logger import apply_log_level
    apply_log_level(self)   # __init__ sonunda çağır

.env'de geçerli değerler:
    LOG_LEVEL=DEBUG   → tüm loglar görünür (yüksek frekanslı callback logları dahil)
    LOG_LEVEL=INFO    → varsayılan, rutin durum logları görünür
    LOG_LEVEL=WARN    → yalnızca uyarı ve hatalar
    LOG_LEVEL=ERROR   → yalnızca hatalar
"""

import os
from rclpy.logging import LoggingSeverity
from utils import load_env

_LEVEL_MAP = {
    'DEBUG':   LoggingSeverity.DEBUG,
    'INFO':    LoggingSeverity.INFO,
    'WARN':    LoggingSeverity.WARN,
    'WARNING': LoggingSeverity.WARN,
    'ERROR':   LoggingSeverity.ERROR,
    'FATAL':   LoggingSeverity.FATAL,
}


def get_log_level() -> LoggingSeverity:
    load_env()
    level_str = os.environ.get('LOG_LEVEL', 'INFO').upper()
    return _LEVEL_MAP.get(level_str, LoggingSeverity.INFO)


def apply_log_level(node) -> str:
    """
    Node'un log seviyesini .env LOG_LEVEL değerine göre ayarlar.
    Uygulanan seviye adını (string) döndürür.
    """
    level = get_log_level()
    node.get_logger().set_level(level)
    level_name = os.environ.get('LOG_LEVEL', 'INFO').upper()
    node.get_logger().info(f'Log seviyesi: {level_name}')
    return level_name
