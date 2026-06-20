import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2
from utils.ros_logger import apply_log_level
from utils.config import get_camera_config

# .env'deki CAMERA_RESOLUTION string'ini sl.RESOLUTION enum'una eşler
_RESOLUTION_MAP = {
    'HD720':  sl.RESOLUTION.HD720,
    'HD1080': sl.RESOLUTION.HD1080,
    'HD2K':   sl.RESOLUTION.HD2K,
    'VGA':    sl.RESOLUTION.VGA,
}

class ZEDPublisherNode(Node):
    def __init__(self):
        super().__init__('zed_publisher_node')
        self.publisher_ = self.create_publisher(Image, '/zed2i_rgb/image_raw', 10)
        self.bridge = CvBridge()
        self.get_logger().info("ZEDPublisherNode başlatılıyor...")

        # .env'den kamera ayarlarını al
        cam_cfg = get_camera_config()
        resolution = _RESOLUTION_MAP.get(cam_cfg.resolution.upper(), sl.RESOLUTION.HD720)

        self.get_logger().info(
            f'Kamera ayarları — çözünürlük={cam_cfg.resolution} fps={cam_cfg.fps}'
        )

        # ZED kamera başlat
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = resolution
        init_params.camera_fps = cam_cfg.fps

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"ZED kamerayı açamadı: {err}")
            rclpy.shutdown()
            return

        self.image = sl.Mat()
        # Timer'ı gerçek FPS'e göre hesapla
        self.timer = self.create_timer(1.0 / cam_cfg.fps, self.timer_callback)
        self.get_logger().info("ZED kamera başarıyla başlatıldı.")
        apply_log_level(self)

    def timer_callback(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            frame = self.image.get_data()

            # ZED BGRA formatında döndürür, RGB'ye çevir
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

            msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            self.publisher_.publish(msg)
            self.get_logger().debug("Görüntü yayınlandı.")
        else:
            self.get_logger().warning("ZED'den görüntü alınamadı.")

    def destroy_node(self):
        self.get_logger().info("ZED kamera kapatılıyor...")
        self.zed.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
