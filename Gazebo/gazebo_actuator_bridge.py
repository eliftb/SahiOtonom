"""
Gazebo Actuator Bridge
======================
UART sender node'unun (uart_sender_node.py) yerini alır.
/ackermann_cmd ve /speed topic'lerini Gazebo'nun anlayacağı
geometry_msgs/Twist (/cmd_vel) komutlarına dönüştürür.

Parametreler:
    cmd_vel_topic   (str)   : Gazebo'daki kontrol topic'i   (varsayılan: /cmd_vel)
    wheelbase       (float) : Araç dingil mesafesi (m)      (varsayılan: 0.3)
    max_speed       (float) : Maksimum hız (m/s)            (varsayılan: 2.0)
    angular_gain    (float) : Gazebo dönüş komutu çarpanı   (varsayılan: 1.0)
    max_angular_z   (float) : Maksimum angular.z sınırı     (varsayılan: 1.2)
    control_mode    (str)   : "ackermann" veya "simple"
        - ackermann : Gerçek Ackermann kinematik dönüşümü (angular.z hesaplanır)
        - simple    : angular.z = steering_angle (küçük açılarda yaklaşık)

Topic haritası (dinlenen):
    /ackermann_cmd  (ackermann_msgs/AckermannDrive) — direksiyon + hız
    /speed          (std_msgs/Float32)              — decision making'den hız
    /lane/lateral_deviation (std_msgs/Float32)      — doğrudan sapma değeri

Topic haritası (yayınlanan):
    {cmd_vel_topic}  (geometry_msgs/Twist)          — Gazebo araç kontrolü
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from utils.ros_logger import apply_log_level


class GazeboActuatorBridge(Node):
    """
    Ackermann komutlarını Gazebo /cmd_vel (Twist) mesajlarına dönüştürür.

    Gerçek UART node'unun (uart_sender_node.py) simülasyon karşılığıdır.
    Seri port bağlantısı gerekmez; tüm komutlar ROS2 topic üzerinden iletilir.
    """

    def __init__(self):
        super().__init__('gazebo_actuator_bridge')

        # --- Parametreler ---
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('control_mode', 'ackermann')
        self.declare_parameter('angular_dead_zone', 0.03)
        self.declare_parameter('steer_sign', -1.0)  # -1.0 → yön ters çevir, 1.0 → normal
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('max_angular_z', 1.2)

        self._cmd_vel_topic    = self.get_parameter('cmd_vel_topic').value
        self._wheelbase        = self.get_parameter('wheelbase').value
        self._max_speed        = self.get_parameter('max_speed').value
        self._max_steer        = self.get_parameter('max_steering_angle').value
        self._control_mode     = self.get_parameter('control_mode').value
        self._dead_zone        = self.get_parameter('angular_dead_zone').value
        self._steer_sign       = float(self.get_parameter('steer_sign').value)
        self._angular_gain     = float(self.get_parameter('angular_gain').value)
        self._max_angular_z    = float(self.get_parameter('max_angular_z').value)

        # --- Durum ---
        self._speed   = 0.0
        self._steer   = 0.0   # radyan

        # --- Publisher ---
        self.cmd_vel_pub = self.create_publisher(
            Twist, self._cmd_vel_topic, 10)

        # --- Subscribers ---
        self.create_subscription(
            AckermannDrive, '/ackermann_cmd', self._ackermann_callback, 10)
        self.create_subscription(
            Float32, '/speed', self._speed_callback, 10)
        self.create_subscription(
            Float32, '/lane/lateral_deviation', self._deviation_callback, 10)

        # 10 Hz yayın timer'ı
        self.create_timer(0.1, self._publish_cmd_vel)

        self.get_logger().info(
            f'Gazebo Actuator Bridge başlatıldı.\n'
            f'  Hedef topic  : {self._cmd_vel_topic}\n'
            f'  Kontrol modu : {self._control_mode}\n'
            f'  Dingil mesaf.: {self._wheelbase} m\n'
            f'  Maks. hız    : {self._max_speed} m/s'
        )
        apply_log_level(self)

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _ackermann_callback(self, msg: AckermannDrive):
        self._speed = float(np.clip(msg.speed, -self._max_speed, self._max_speed))
        self._steer = float(np.clip(msg.steering_angle, -self._max_steer, self._max_steer))
        self.get_logger().debug(
            f'Ackermann → hız={self._speed:.2f} m/s | direksiyon={self._steer:.3f} rad')

    def _speed_callback(self, msg: Float32):
        """Decision making node'dan gelen ham hız (direksiyon dokunmaz)."""
        self._speed = float(np.clip(msg.data, -self._max_speed, self._max_speed))

    def _deviation_callback(self, msg: Float32):
        """
        Lateral deviation'dan basit orantısal direksiyon.
        /ackermann_cmd yoksa veya zayıfsa bu devreye girer.
        """
        pass  # DecisionMaking zaten /ackermann_cmd üretiyor; ekstra işlem yok

    # ------------------------------------------------------------------ #
    #  Yayın döngüsü                                                       #
    # ------------------------------------------------------------------ #

    def _publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self._speed

        if abs(self._speed) < 1e-6:
            angular_z = 0.0
        elif self._control_mode == 'ackermann':
            angular_z = self._speed * np.tan(self._steer) / self._wheelbase
        else:
            angular_z = float(self._steer)

        # Yön düzeltmesi + dead zone
        angular_z *= self._steer_sign * self._angular_gain
        if abs(angular_z) < self._dead_zone:
            angular_z = 0.0

        twist.angular.z = float(np.clip(angular_z, -self._max_angular_z, self._max_angular_z))
        self.cmd_vel_pub.publish(twist)

        self.get_logger().debug(
            f'cmd_vel → linear.x={twist.linear.x:.2f} | angular.z={twist.angular.z:.3f}')

    # ------------------------------------------------------------------ #
    #  Güvenli kapanış                                                     #
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        try:
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            self.get_logger().info('Araç durduruldu (cmd_vel sıfırlandı).')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboActuatorBridge()
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
