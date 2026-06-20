"""
Gazebo Camera Bridge
====================
ZED2i kameranın yerini alır. Gazebo'nun yayınladığı kamera topic'ini okur
ve mevcut node'ların beklediği /zed2i_rgb/image_raw topic'ine yayınlar.

Parametre:
    gazebo_camera_topic (str): Gazebo'daki kamera topic'i
                               Varsayılan: /camera/image_raw

Örnek kullanım — farklı topic için:
    ros2 run ... --ros-args -p gazebo_camera_topic:=/my_robot/camera/image_raw
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from utils.ros_logger import apply_log_level


class GazeboCameraBridge(Node):
    """
    Gazebo kamera topic'ini okuyup /zed2i_rgb/image_raw olarak yeniden yayınlar.

    Gerçek ZED2i node'unun (zedi2connect_port.py) simülasyon karşılığıdır.
    Encoding dönüşümünü otomatik yapar; LaneDetection ve SignDetection node'ları
    bu bridge'i fark etmeden çalışmaya devam eder.
    """

    def __init__(self):
        super().__init__('gazebo_camera_bridge')

        self.declare_parameter('gazebo_camera_topic', '/camera/image_raw')
        self.declare_parameter('target_width', 0)   # 0 = resize yok
        self.declare_parameter('target_height', 0)

        gazebo_topic = self.get_parameter('gazebo_camera_topic').value
        self._target_w = self.get_parameter('target_width').value
        self._target_h = self.get_parameter('target_height').value

        self.bridge = CvBridge()
        self._frame_count = 0

        self.subscription = self.create_subscription(
            Image, gazebo_topic, self._image_callback, 10)

        self.publisher = self.create_publisher(
            Image, '/zed2i_rgb/image_raw', 10)

        self.get_logger().info(
            f'Gazebo Kamera Bridge başlatıldı.\n'
            f'  Kaynak : {gazebo_topic}\n'
            f'  Hedef  : /zed2i_rgb/image_raw\n'
            f'  Resize : '
            f'{"KAPALI" if not (self._target_w and self._target_h) else f"{self._target_w}x{self._target_h}"}'
        )
        apply_log_level(self)

    def _image_callback(self, msg: Image):
        try:
            # Gazebo'dan gelen encoding'e göre BGR'ye çevir
            encoding = msg.encoding.lower()
            if encoding in ('rgb8', 'rgb'):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif encoding in ('bgr8', 'bgr'):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif encoding in ('bgra8', 'rgba8'):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif encoding in ('mono8', '8uc1'):
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                # Bilinmeyen encoding — ham dönüşüm dene
                self.get_logger().warn(
                    f'Bilinmeyen encoding: {msg.encoding} — BGR8 olarak deneniyor.',
                    throttle_duration_sec=10.0)
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # İsteğe bağlı resize
            if self._target_w and self._target_h:
                frame = cv2.resize(
                    frame, (self._target_w, self._target_h),
                    interpolation=cv2.INTER_LINEAR)

            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_msg.header = msg.header
            self.publisher.publish(out_msg)

            self._frame_count += 1
            if self._frame_count % 100 == 0:
                self.get_logger().info(f'{self._frame_count} kare yayınlandı.')

        except Exception as e:
            self.get_logger().error(f'Kare işleme hatası: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
