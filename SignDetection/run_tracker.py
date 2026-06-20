import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from collections import defaultdict, deque, Counter
import os
import pathlib
from utils.ros_logger import apply_log_level
from utils.config import is_cv_display_enabled

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')
        self.br = CvBridge()

        self.declare_parameter('model_name', 'UltraConservative_BEST_mAP0.9248_20250801_115028.pt')
        
        # history_length: Bir nesnenin sınıfını belirlemek için kaç kare geriye bakılacağı.
        self.declare_parameter('history_length', 10) 
        # min_confidence_frames: history_length içinde bir sınıfın en az kaç kere
        # görülmesi gerektiği ki "stabil" olarak kabul edilsin.
        self.declare_parameter('min_confidence_frames', 4) 

        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.history_length = self.get_parameter('history_length').get_parameter_value().integer_value
        self.min_confidence_frames = self.get_parameter('min_confidence_frames').get_parameter_value().integer_value

        # Her bir takip ID'sinin sınıf geçmişini tutmak için deque kullanıyoruz.
        # deque, maxlen parametresi sayesinde otomatik olarak eski verileri atar.
        self.track_class_history = defaultdict(lambda: deque(maxlen=self.history_length))
        
        try:
            model_path = str(pathlib.Path(__file__).parent / model_name)
            self.model = YOLO(model_path)
            self.get_logger().info(f'✅ Levha modeli yüklendi: {model_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Levha modeli yüklenemedi: {e}')
            raise RuntimeError(f'Model yüklenemedi: {model_path}') from e
            
        self.subscription = self.create_subscription(Image, '/zed2i_rgb/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/sign_detection/output', 10)

        self._cv_display = is_cv_display_enabled()
        self._request_shutdown = False
        self.get_logger().info(
            f'✅ Gelişmiş Levha Tespit Düğümü Başlatıldı. '
            f'Ekran: {"AÇIK" if self._cv_display else "KAPALI (headless)"}'
        )
        apply_log_level(self)

    def image_callback(self, msg):
        try:
            # Görüntüyü ROS'tan alıp OpenCV formatına çeviriyoruz (BGR)
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLO ile takip et (henüz çizim yapma)
            results = self.model.track(frame, persist=True, conf=0.4, iou=0.5, verbose=False)
            
            # Geliştirilmiş fonksiyonumuzla sonuçları işle, stabilize et ve çiz
            annotated_frame = self.draw_stabilized_annotations(results[0], frame.copy())
            
            # Sonucu ekranda göster (CV_DISPLAY=true ise)
            if self._cv_display:
                cv2.imshow("Sign Detection (Stabilized & Correct Color)", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._request_shutdown = True
                    return

            # Sonucu ROS'ta yayınla
            self.publisher.publish(self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8"))
        except Exception as e:
            self.get_logger().error(f'Levha Tespit Callback Hatası: {e}')
            import traceback
            traceback.print_exc()

    def draw_stabilized_annotations(self, result, frame):
        """
        Tespit sonuçlarını alır, sınıflandırmayı stabilize eder ve
        sonuçları manuel olarak OpenCV ile görüntü üzerine çizer.
        """
        # Eğer takip edilen bir nesne yoksa, orijinal görüntüyü direkt döndür.
        if not hasattr(result.boxes, 'id') or result.boxes.id is None:
            return frame

        track_ids = result.boxes.id.int().cpu().tolist()
        boxes = result.boxes.xywh.cpu()
        class_ids = result.boxes.cls.int().cpu().tolist()

        # Her bir tespit edilen nesne için döngü
        for box, track_id, class_id in zip(boxes, track_ids, class_ids):
            # 1. Bu nesnenin sınıflandırma geçmişini güncelle
            self.track_class_history[track_id].append(class_id)
            
            # 2. Geçmişi analiz et
            history = self.track_class_history[track_id]
            if not history:
                continue

            # 3. Geçmişteki en yaygın sınıfı ve görülme sayısını bul
            most_common_class, confidence_count = Counter(history).most_common(1)[0]

            # 4. Sadece yeterince emin olduğumuz (stabil) sınıfları çiz
            if confidence_count >= self.min_confidence_frames:
                x, y, w, h = box
                x1, y1 = int(x - w / 2), int(y - h / 2)
                x2, y2 = int(x + w / 2), int(y + h / 2)
                
                # Stabil sınıfın adını modelden al
                stable_class_name = self.model.names[most_common_class]
                label = f"ID:{track_id} {stable_class_name}"

                # RENK DÜZELTMESİ: OpenCV BGR formatında renk bekle
                color = (255, 165, 0) 

                # Dikdörtgeni çiz
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # Etiket için daha okunaklı bir arka plan çiz
                (w_text, h_text), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, y1 - 20), (x1 + w_text, y1), color, -1)
                cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) # Yazı Beyaz
        
        return frame

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()
        self.get_logger().info("Levha Tespit Düğümü kapatıldı.")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SignDetectionNode()
    except RuntimeError as e:
        rclpy.shutdown()
        return
    try:
        while rclpy.ok() and not node._request_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()