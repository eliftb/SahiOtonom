# utils — Yardımcı Modüller

Bu klasör, tüm ROS2 node'ları tarafından ortak kullanılan yardımcı araçları barındırır.

| Dosya | Görev |
|-------|-------|
| `__init__.py` | Paylaşılan `load_env()` — `.env` dosyasını bir kez yükler |
| `config.py` | Donanım, PID ve kamera ayarlarını tip güvenli döndürür |
| `ros_logger.py` | LOG_LEVEL'i `.env`'den okuyup ROS2 node'larına uygular |

---

## `__init__.py` — Paylaşılan `.env` Yükleyici

`load_env()` fonksiyonu proje kökündeki `.env` dosyasını `os.environ`'a yükler. `config.py` ve `ros_logger.py` bu fonksiyonu ortak kullanır; yani `.env` tek bir süreç içinde yalnızca bir kez okunur.

`python-dotenv` kurulu değilse `KEY=VALUE` formatını elle parse eder — ek kurulum gerekmez.

---

## `config.py` — Yapılandırma Modülü

### Nedir?

Hardcode değerleri koddan çıkarıp `.env` dosyasına taşır. Her kategori için `frozen=True` dataclass döndürür; bu sayede ayarlar salt okunurdur ve yanlışlıkla değiştirilemez.

### Kullanım — Yeni Node'a Nasıl Eklenir?

```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.config import get_serial_config, get_pid_config, get_camera_config

class BenimNode(Node):
    def __init__(self):
        super().__init__('benim_node')

        serial = get_serial_config()
        # serial.speed_port, serial.steering_port, serial.stop_port, serial.baud_rate

        pid = get_pid_config()
        # pid.kp, pid.ki, pid.kd, pid.integral_clamp

        cam = get_camera_config()
        # cam.resolution ('HD720'), cam.fps (30)
```

### Mevcut Fonksiyonlar

| Fonksiyon | Döndürür | `.env` Anahtarları |
|-----------|----------|-------------------|
| `get_serial_config()` | `SerialConfig` | `SERIAL_SPEED_PORT`, `SERIAL_STEERING_PORT`, `SERIAL_STOP_PORT`, `SERIAL_BAUD_RATE` |
| `get_pid_config()` | `PIDConfig` | `PID_KP`, `PID_KI`, `PID_KD`, `PID_INTEGRAL_CLAMP` |
| `get_camera_config()` | `CameraConfig` | `CAMERA_RESOLUTION`, `CAMERA_FPS` |
| `is_cv_display_enabled()` | `bool` | `CV_DISPLAY` |
| `get_backend_port()` | `int` | `BACKEND_PORT` |

### `CV_DISPLAY` — OpenCV Pencere Kontrolü

Yarışma ortamında `cv2.imshow()` çağrıları monitör gerektirdiğinden hata verebilir. `CV_DISPLAY` bayrağı, tüm `imshow` / `waitKey` çağrılarını tek yerden açıp kapatır.

```bash
# .env
CV_DISPLAY=true   # geliştirme — imshow pencereleri açılır
CV_DISPLAY=false  # yarışma / headless — pencere açılmaz
```

```python
from utils.config import is_cv_display_enabled

class BenimNode(Node):
    def __init__(self):
        super().__init__('benim_node')
        self._cv_display = is_cv_display_enabled()

    def image_callback(self, msg):
        # ... işlem ...
        if self._cv_display:
            cv2.imshow('Pencere', frame)
            cv2.waitKey(1)
```

Bu bayrağı kullanan node'lar: `run_tracker.py`, `serit-tespitcopy.py`, `zedi2connect_port.py`.

---

## `ros_logger.py` — Merkezi Loglama Sistemi

### Nedir?

Her ROS2 node'u kendi içinde `self.get_logger()` ile log yazar. Ancak hangi logların ekranda görüneceği — yani **log seviyesi** — varsayılan olarak her node için ayrı ayrı yönetilmek zorundaydı. `ros_logger.py`, bu problemi çözmek için projenin kök dizinindeki `.env` dosyasını okur ve tüm node'lara tek bir yerden log seviyesi uygular.

### Nasıl Çalışır?

```
.env
 ├─ LOG_LEVEL=INFO
 ├─ CV_DISPLAY=false
 ├─ SERIAL_SPEED_PORT=/dev/ttyACM1
 ├─ PID_KP=0.05  ...
 └─ CAMERA_RESOLUTION=HD720  ...
        │
        ▼
utils/__init__.py → load_env()   (tek seferlik yükleme, tüm modüller paylaşır)
        │
        ├─► utils/config.py
        │       ├─ get_serial_config() / get_pid_config() / get_camera_config()
        │       └─ is_cv_display_enabled()  →  imshow çağrılarını açar/kapar
        │
        └─► utils/ros_logger.py
                └─ get_log_level() → LoggingSeverity
                └─ apply_log_level(node) → node.get_logger().set_level(...)
                        │
                        ▼
                Her ROS2 Node (__init__ sonunda)
                 ├─ apply_log_level(self)
                 └─ self._cv_display = is_cv_display_enabled()
```

### Kurulum — Yeni Node'a Nasıl Eklenir?

```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # proje kökünü path'e ekle

from utils.ros_logger import apply_log_level

class Node(Node):
    def __init__(self):
        super().__init__('node')
        # ... diğer kurulum ...
        apply_log_level(self)  # her zaman __init__'in sonunda çağır
```

---

## Log Seviyeleri Rehberi

Log seviyesi `.env` dosyasındaki `LOG_LEVEL` satırı değiştirilerek ayarlanır:

```
# .env
LOG_LEVEL=INFO
```

### Seviyeler ve Anlamları

| Seviye | Değer | Ne Görünür |
|--------|-------|-----------|
| `DEBUG` | En düşük | Her şey — callback başına detaylar, frekans bağımsız tüm loglar |
| `INFO` | Varsayılan | Durum değişimleri, başlangıç bilgileri, önemli olaylar |
| `WARN` | Orta | Yalnızca uyarılar ve hatalar |
| `ERROR` | Yüksek | Yalnızca kritik hatalar |
| `FATAL` | En yüksek | Yalnızca sistem çöküşleri |


---

## Projede Hangi Log Nereye Yazılır?

Bu tablo projedeki tüm node'lardaki log kararlarını belgeler. Yeni log eklerken bu tabloyu referans alın.

### Seviye Kararı — Genel Kural

```
Soru: Bu log kaç kez tetiklenir?
│
├─ Saniyede 1'den fazla (callback başına, timer döngüsü) ──► DEBUG
│
├─ Ara sıra, durum değişince ──────────────────────────────► INFO
│
├─ Beklenmeyen ama kurtarılabilir durum ───────────────────► WARN
│
└─ İşlem durdurucu hata ───────────────────────────────────► ERROR
```

---

### `zedi2connect_port.py` — Kamera Node'u

| Log | Seviye | Neden |
|-----|--------|-------|
| "ZEDPublisherNode başlatılıyor..." | INFO | Tek seferlik başlangıç |
| "ZED kamera başarıyla başlatıldı." | INFO | Tek seferlik donanım onayı |
| "ZED kamerayı açamadı: {err}" | ERROR | Kritik donanım hatası |
| "Görüntü yayınlandı." | **DEBUG** | 30 Hz'de tetiklenir — INFO'da saniyede 30 satır |
| "ZED'den görüntü alınamadı." | WARN | Geçici kayıp, kurtarılabilir |
| "ZED kamera kapatılıyor..." | INFO | Tek seferlik kapanış |

---

### `uart_sender_node.py` — UART Haberleşme Node'u

| Log | Seviye | Neden |
|-----|--------|-------|
| Port açılış logları (Hız/Stop/Direksiyon) | INFO | Tek seferlik donanım onayı |
| "Port açılamadı: {e}" | ERROR | Kritik donanım hatası |
| "UART Gönderici başlatıldı." | INFO | Tek seferlik başlangıç |
| `HIZ \| {speed} → sinyal: {s}` | **DEBUG** | Her `/speed` mesajında (~10 Hz) |
| `LATERAL \| dev=... steer=...` | **DEBUG** | Her `/lane/lateral_deviation` mesajında (~10-20 Hz) |
| `MANUEL HIZ \| ...` | **DEBUG** | Her `/ackermann_cmd` mesajında |
| `MANUEL DİREKSİYON \| ...` | **DEBUG** | Her manuel direksiyon komutunda |
| "Lateral kontrol yeniden etkinleştirildi" | INFO | Durum değişimi, nadir |
| "Speed/UART gönderme hatası: {e}" | ERROR | İşlem hatası |

---

### `engel-tespit.py` — LiDAR Engel Tespiti

| Log | Seviye | Neden |
|-----|--------|-------|
| "LiDAR Engel Dedektörü başlatıldı \| ..." | INFO | Tek seferlik başlangıç |
| "Geçersiz angle_increment değeri." | WARN | LiDAR veri hatası |
| "🚨 Engel GİRDİ: {dist}m ({taraf})" | INFO | **Durum değişimi**: engel yokken var oldu |
| "✅ Engel temizlendi." | INFO | **Durum değişimi**: engel varken yok oldu |
| `Engel: {dist}m ({taraf})` | **DEBUG** | Engel zaten varken her scan'de tekrar (~10 Hz) |

> **Not:** `_prev_obstacle_detected` bayrağıyla durum değişimi takibi yapılır. Aynı durum devam ediyorsa INFO yerine DEBUG yazılır. Bu sayede ekran engel varken her saniye 10 satır logla dolmaz.

---

### `basic-decision-making-node.py` — Karar Alma (Normal Mod)

| Log | Seviye | Neden |
|-----|--------|-------|
| "Decision Making Node başlatıldı." | INFO | Tek seferlik başlangıç |
| "⛔ Acil duruş! Engel mesafesi: {d}m" | WARN | Kritik durum, acil müdahale |
| "⚠️ Yavaşlama! Engel mesafesi: {d}m" | INFO | Hız kısıtlaması başladı |
| `Speed: ... Steering: ... Deviation: ...` | **DEBUG** | 10 Hz timer döngüsü |
| "Decision loop error: {e}" | ERROR | İşlem hatası |

---

### `decision-making-node-avoidance.py` — Karar Alma (Kaçınma Modu)

| Log | Seviye | Neden |
|-----|--------|-------|
| "Gelişmiş Decision Making Node başlatıldı." | INFO | Tek seferlik başlangıç |
| "🚨 Engel tespit edildi! Pozisyon: {taraf}" | INFO | Durum değişimi |
| "✅ Engel kayboldu, normal duruma dönülüyor." | INFO | Durum değişimi |
| "⛔ EMERGENCY STOP başladı! Mesafe: {d}m" | WARN | Kritik durum geçişi |
| "🔄 OBSTACLE AVOIDANCE {yön} moduna geçildi!" | WARN | Kritik manevra başladı |
| "🛣️ Şerit takibi aşamasına geçildi!" | INFO | Faz geçişi |
| "➡️ / ⬅️ Dönüş aşamasına geçildi!" | INFO | Faz geçişi |
| "✅ Engel kaçınma tamamlandı!" | INFO | Manevra bitti |
| `[state] hız=... steer=... dev=...` | **DEBUG** | 10 Hz timer döngüsü |
| "Decision loop error: {e}" | ERROR | İşlem hatası |

---

### `run_tracker.py` — Levha Tespiti (YOLO)

| Log | Seviye | Neden |
|-----|--------|-------|
| "✅ Levha modeli yüklendi: {path}" | INFO | Tek seferlik model yükleme onayı |
| "❌ Levha modeli yüklenemedi: {e}" | ERROR | Kritik başlatma hatası |
| "✅ Gelişmiş Levha Tespit Düğümü Başlatıldı. Ekran: ..." | INFO | Tek seferlik başlangıç |
| "Levha Tespit Callback Hatası: {e}" | ERROR | İşlem hatası |
| "Levha Tespit Düğümü kapatıldı." | INFO | Tek seferlik kapanış |

---

### `serit-tespitcopy.py` — Şerit Tespiti

| Log | Seviye | Neden |
|-----|--------|-------|
| "🚦 Lane Detection Node başlatıldı. Ekran: ..." | INFO | Tek seferlik başlangıç |
| "✅ Model yüklendi." | INFO | Tek seferlik model yükleme onayı |
| `Brightness: ... \| Horizontal: ... \| Direction: ...` | **DEBUG** | Her kare işleminde (~10-20 Hz) |
| "Image callback error: {e}" | ERROR | İşlem hatası |

---

## Yeni Log Yazarken Kontrol Listesi

Projeye yeni bir log satırı eklemeden önce şu soruları sor:

1. **Bu log ne sıklıkla tetiklenir?**
   - Callback veya timer içindeyse büyük ihtimalle `debug()` olmalı.

2. **Kullanıcının bu logu her zaman görmesi gerekiyor mu?**
   - Evet → `info()` veya `warn()`
   - Sadece hata ayıklarken → `debug()`

3. **Bu log bir durum değişimini mi yoksa sürekli durumu mu yansıtıyor?**
   - Durum değişimi (engel girdi, mod değişti, port açıldı) → `info()`
   - Sürekli durum (engel hâlâ var, araç hâlâ ilerliyor) → `debug()`

4. **Sistemin normale dönebileceği bir sorun mu?**
   - Evet → `warn()`
   - Hayır, sistem durmalı → `error()`

---

## `python-dotenv` Olmadan Çalışma

`python-dotenv` kurulu değilse `utils/__init__.py` içindeki `load_env()` `.env` dosyasını elle parse eder. Yorum satırları (`#`) ve boş satırlar atlanır, `KEY=VALUE` formatı beklenir. Yine de paketi kurmanız önerilir:

```bash
pip install python-dotenv
# veya
pip install -r requirements.txt
```
