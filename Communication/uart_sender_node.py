import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
import serial
import time
import numpy as np
from utils.ros_logger import apply_log_level
from utils.config import get_serial_config, get_pid_config

class UartSenderNode(Node):
    def __init__(self):
        super().__init__('uart_sender_node')

        # .env'den serial port ve baud rate varsayılanlarını al
        serial_cfg = get_serial_config()

        self.declare_parameter('speed_port',    serial_cfg.speed_port)
        self.declare_parameter('steering_port', serial_cfg.steering_port)
        self.declare_parameter('stop_port',     serial_cfg.stop_port)
        self.declare_parameter('baud_rate',     serial_cfg.baud_rate)
        self.declare_parameter('lateral_deviation_topic', '/lane/lateral_deviation')
        self.declare_parameter('use_ackermann_speed', False)

        self.SPEED_PORT    = self.get_parameter('speed_port').value
        self.STEERING_PORT = self.get_parameter('steering_port').value
        self.STOP_PORT     = self.get_parameter('stop_port').value
        self.BAUD_RATE     = self.get_parameter('baud_rate').value
        lateral_topic      = self.get_parameter('lateral_deviation_topic').value
        self.use_ackermann_speed = bool(self.get_parameter('use_ackermann_speed').value)

        self.get_logger().info(f'Hız Portu: {self.SPEED_PORT}')
        self.get_logger().info(f'Stop Portu: {self.STOP_PORT}')
        self.get_logger().info(f'Direksiyon Portu: {self.STEERING_PORT}')
        self.get_logger().info(f'Lateral Deviation Topic: {lateral_topic}')
        self.get_logger().info(f'Ackermann hız fallback: {self.use_ackermann_speed}')

        # .env'den PID kazançlarını al
        pid_cfg = get_pid_config()
        self.kp = pid_cfg.kp
        self.ki = pid_cfg.ki
        self.kd = pid_cfg.kd
        self._pid_integral_clamp = pid_cfg.integral_clamp

        self.get_logger().info(
            f'PID kazançları — kp={self.kp} ki={self.ki} kd={self.kd} '
            f'clamp=±{self._pid_integral_clamp}'
        )

        self.prev_error = 0.0
        self.integral = 0.0
        self.max_steering_angle = 0.5
        self._pid_last_time = None  # dt hesabı için

        # Lateral deviation ve manuel ackermann komutları
        self.current_lateral_deviation = 0.0
        self.manual_speed = 0.0

        # Hız portunu aç
        try:
            self.speed_serial = serial.Serial(self.SPEED_PORT, self.BAUD_RATE, timeout=0.1)
            time.sleep(1)
            self.get_logger().info(f'✅ Hız portu (FREN/İTKİ) açıldı.')
        except Exception as e:
            self.get_logger().error(f'❌ Hız portu açılamadı: {e}')
            self.speed_serial = None

        # Stop portunu aç
        try:
            self.stop_serial = serial.Serial(self.STOP_PORT, self.BAUD_RATE, timeout=0.1)
            time.sleep(1)
            self.get_logger().info(f'✅ Stop portu (FREN) açıldı.')
        except Exception as e:
            self.get_logger().error(f'❌ Stop portu açılamadı: {e}')
            self.stop_serial = None

        # Direksiyon portunu aç
        try:
            self.steering_serial = serial.Serial(self.STEERING_PORT, self.BAUD_RATE, timeout=0.1)
            time.sleep(1)
            self.get_logger().info(f'✅ Direksiyon portu açıldı.')
        except Exception as e:
            self.get_logger().error(f'❌ Direksiyon portu açılamadı: {e}')
            self.steering_serial = None

        # Lateral deviation subscriber (topic parametreden gelir)
        self.lateral_sub = self.create_subscription(
            Float32, lateral_topic, self.lateral_deviation_callback, 10)

        # Manuel ackermann komutları
        self.ackermann_sub = self.create_subscription(
            AckermannDrive, '/ackermann_cmd', self.ackermann_callback, 10)

        # Decision making node'dan hız verisi. /ackermann_cmd de aynı hızı taşıdığı
        # için UART'a varsayılan olarak yalnızca /speed yazılır.
        self.speed_sub = self.create_subscription(
            Float32, '/speed', self.speed_callback, 10)

        self.get_logger().info('🦾 UART Gönderici başlatıldı. Komut bekleniyor...')
        apply_log_level(self)

    def _write_speed_signal(self, speed: float, source: str):
        """Hız değerini speed ve stop portlarına güvenli şekilde yazar."""
        speed_signal = self.speed_to_digital_signal(speed)

        if self.stop_serial and self.stop_serial.is_open:
            stop_signal = 0 if speed_signal == 1 else 1
            self.stop_serial.write(stop_signal.to_bytes(1, byteorder='little'))

        if self.speed_serial and self.speed_serial.is_open:
            self.speed_serial.write(speed_signal.to_bytes(1, byteorder='little'))
            self.get_logger().debug(f'📡 {source} | {speed:.2f} m/s → sinyal: {speed_signal}')

    def speed_callback(self, msg: Float32):
        """Decision making node'undan gelen speed verisini işle."""
        try:
            self._write_speed_signal(float(msg.data), "HIZ")
        except Exception as e:
            self.get_logger().error(f'Speed callback hatası: {e}')

    def lateral_deviation_to_steering_angle(self, lateral_deviation):
        """
        PID kontrolör kullanarak lateral deviation'ı direksiyon açısına dönüştürür.
        dt (delta time) hesabıyla integral ve türev terimleri zamana göre normalize edilir.
        """
        current_time = time.time()
        if self._pid_last_time is None:
            # İlk çağrıda I ve D terimleri anlamlı değil; sadece P kullan
            self._pid_last_time = current_time
            self.prev_error = lateral_deviation
            return float(np.clip(self.kp * lateral_deviation, -self.max_steering_angle, self.max_steering_angle))
        dt = current_time - self._pid_last_time
        if dt <= 0.0:
            dt = 1e-4
        self._pid_last_time = current_time

        error = lateral_deviation

        # P terimi
        p_term = self.kp * error

        # I terimi — dt ile normalize, windup koruması
        self.integral += error * dt
        self.integral = max(-self._pid_integral_clamp, min(self.integral, self._pid_integral_clamp))
        i_term = self.ki * self.integral

        # D terimi — dt ile normalize
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        pid_output = -(p_term + i_term + d_term)
        steering_angle = max(-self.max_steering_angle, min(pid_output, self.max_steering_angle))
        return steering_angle

    def lateral_deviation_callback(self, msg: Float32):
        """Lateral deviation mesajını alır ve direksiyon kontrolü yapar.
        Normal modda /lane/lateral_deviation, kaçınma modunda launch parametresiyle
        /lane/lateral_new_deviation dinlenir.
        """
        self.current_lateral_deviation = msg.data
        steering_angle = self.lateral_deviation_to_steering_angle(self.current_lateral_deviation)

        if self.steering_serial and self.steering_serial.is_open:
            angle_byte = self.angle_to_byte(steering_angle)
            self.steering_serial.write(angle_byte.to_bytes(1, byteorder='little'))
            self.get_logger().debug(
                f'🎯 LATERAL | dev={self.current_lateral_deviation:.3f} '
                f'steer={steering_angle:.3f} rad byte={angle_byte} '
                f'[{"SAĞ→SOL" if self.current_lateral_deviation < 0 else "SOL→SAĞ" if self.current_lateral_deviation > 0 else "MERKEZ"}]'
            )

    def speed_to_digital_signal(self, speed_ms):
        """Gelen hız değerine göre 1 (İleri Git) veya 0 (Dur/Fren) döndürür."""
        return 1 if speed_ms > 0.1 else 0

    def angle_to_byte(self, angle_rad):
        """Direksiyon açısını oransal olarak (0-255) çevirir.
        -max_steering_angle → 0, 0 → 127, +max_steering_angle → 255"""
        angle_rad = max(-self.max_steering_angle, min(angle_rad, self.max_steering_angle))
        byte_value = int((angle_rad + self.max_steering_angle) / (2 * self.max_steering_angle) * 255)
        return max(0, min(byte_value, 255))

    def ackermann_callback(self, msg: AckermannDrive):
        """Ackermann komutunu alır; hız yazımı sadece fallback açıksa yapılır."""
        try:
            self.manual_speed = float(msg.speed)
            if self.use_ackermann_speed:
                self._write_speed_signal(self.manual_speed, "ACKERMANN")

        except Exception as e:
            self.get_logger().error(f'UART gönderme hatası: {e}')

    def _safe_shutdown_outputs(self):
        """Node kapanırken motorları güvenli duruma getir."""
        try:
            if hasattr(self, 'speed_serial') and self.speed_serial and self.speed_serial.is_open:
                self.speed_serial.write((0).to_bytes(1, 'little'))
                self.speed_serial.close()
            if hasattr(self, 'steering_serial') and self.steering_serial and self.steering_serial.is_open:
                self.steering_serial.write((127).to_bytes(1, 'little'))
                self.steering_serial.close()
            if hasattr(self, 'stop_serial') and self.stop_serial and self.stop_serial.is_open:
                self.stop_serial.write((1).to_bytes(1, 'little'))
                self.stop_serial.close()
        except Exception:
            pass

    def destroy_node(self):
        self._safe_shutdown_outputs()
        super().destroy_node()

    def __del__(self):
        self._safe_shutdown_outputs()

def main(args=None):
    rclpy.init(args=args)
    node = UartSenderNode()
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
