# SahiOtonom

Otonom araç yazılım sistemi. ZED2i kamera, LiDAR sensörü ve UART haberleşmesiyle şerit takibi ve engel kaçınma yeteneklerine sahip ROS2 tabanlı bir mimari üzerine kurulmuştur.

---

## Proje Yapısı

```
SahiOtonom/
├── Camera/
│   └── zedi2connect_port.py          # ZED2i camera → ROS2 image topic publisher
├── LaneDetection/
│   ├── lane_detection.py             # Lane detection and lateral deviation
│   ├── models/
│   │   └── tusimple_18.pt            # TUSimple lane detection model
│   └── utils/
│       └── utils.py                  # Helper functions
├── SignDetection/
│   ├── run_tracker.py                # Traffic sign detection (YOLO)
│   └── UltraConservative_BEST_...pt  # YOLO sign detection model
├── ObstacleDetection/
│   └── obstacle_detection.py         # LiDAR-based obstacle detection
├── DecisionMaking/
│   ├── basic-decision-making-node.py          # Normal mode decision making
│   └── decision-making-node-avoidance.py      # Obstacle avoidance decision making
├── Communication/
│   └── uart_sender_node.py           # UART communication node
├── utils/
│   ├── __init__.py                   # Paylaşılan load_env() — .env tek seferlik yükler
│   ├── config.py                     # Donanım, PID, kamera ayarları (tip güvenli dataclass)
│   ├── ros_logger.py                 # Merkezi loglama — .env'den LOG_LEVEL okur
│   └── README.md                     # utils rehberi ↗
├── .env                              # Tüm ayarlar: portlar, PID, kamera, loglama (git'e girmez)
├── .env.example                      # .env şablonu
├── launch_all_nodes.py               # Normal mod başlatıcı
├── launch_all_nodes_avoidance.py     # Engel kaçınma modu başlatıcı
├── requirements.txt
├── CONTRIBUTING.md               # Git akışı, branch ve PR kuralları ↗
├── test/
│   └── test_lateral_deviatiton.py
└── archive/                          # Eski/kullanılmayan dosyalar
```

---

## Node Mimarisi

```
ZED2i Kamera
    └─► [zedi2connect_port.py]
            │  /zed2i_rgb/image_raw
            ├─► [serit-tespitcopy.py]
            │       │  /lane/lateral_deviation
            │       │  /lane/lateral_new_deviation   (avoidance modunda)
            │       │  /lane/intersection_direction  (kavşak yönü)
            │       └─► [uart_sender_node.py] ──► UART (Direksiyon)
            │
            └─► [run_tracker.py]
                    │  /sign_detection/output  (annotated görüntü)
                    └─► [decision-making-node.py]

LiDAR
    └─► [engel-tespit.py]
            │  /obstacle_detected   (Bool)
            │  /obstacle_distance   (Float32)
            │  /obstacle_side       (Bool — true: sağ, false: sol)
            └─► [decision-making-node.py]
                    │  /speed
                    └─► [uart_sender_node.py] ──► UART (Hız/Fren)
```

---

## Başlatma

### Normal Mod (Şerit Takibi)

```bash
python3 launch_all_nodes.py
```

Sırasıyla başlatılan node'lar:
1. ZED2i Kamera Yayını (2s bekleme)
2. Şerit Tespit (2s bekleme)
3. Levha Tespit / Görüntü İşleme (3s bekleme)
4. LiDAR Engel Tespiti (3s bekleme)
5. Karar Alma Algoritması (5s bekleme)
6. UART Haberleşme

### Engel Kaçınma Modu

```bash
python3 launch_all_nodes_avoidance.py
```

Normal moddan farkları:
- UART node'u `/lane/lateral_new_deviation` topic'ini dinler
- Karar alma node'u `decision-making-node-avoidance.py` kullanır (durum makinesi: NORMAL → OBSTACLE_DETECTED → AVOIDANCE → NORMAL)

---

## Node'lar

### `Camera/zedi2connect_port.py` — Kamera Yayıncısı
- ZED2i kamerayı başlatır (çözünürlük ve FPS `.env`'den okunur; varsayılan: HD720, 30 FPS)
- BGRA → RGB dönüşümü yaparak `/zed2i_rgb/image_raw` topic'ine yayın yapar

### `LaneDetection/lane_detection.py` — Şerit Tespiti
- TUSimple modeli (`tusimple_18.pt`) ile şerit tespiti yapar
- Lateral deviation değerini `/lane/lateral_deviation` topic'ine yayınlar
- Kavşak yönünü `/lane/intersection_direction` topic'ine yayınlar (0: yok, 1: sol, 2: sağ, 4: düz)
- Model path: `LaneDetection/models/tusimple_18.pt`

### `SignDetection/run_tracker.py` — Levha Tespiti
- YOLO tabanlı trafik levhası tespiti
- Takip stabilizasyonu için geçmiş tabanlı sınıflandırma
- ROS2 parametreleri: `model_name`, `history_length`, `min_confidence_frames`
- Model path: `GoruntuIsleme/<model_name>`

### `ObstacleDetection/obstacle_detection.py` — LiDAR Engel Tespiti
- LiDAR `/scan` topic'ini dinler
- Araç önündeki engelleri tespit eder
- `/obstacle_detected` (Bool), `/obstacle_distance` (Float32) ve `/obstacle_side` (Bool — `true`: sağ, `false`: sol) yayınlar
- ROS2 parametreleri: `obstacle_threshold` (m), `front_angle_range_deg`

### `DecisionMaking/basic-decision-making-node.py` — Karar Alma (Normal Mod)
- Şerit sapması ve engel bilgisine göre hız ve direksiyon açısı hesaplar
- `/ackermann_cmd` ve `/speed` topic'lerine yayın yapar
- ROS2 parametreleri: `base_speed`, `max_steering_angle`, `steering_gain`, `emergency_stop_distance`, `slow_down_distance`

### `DecisionMaking/decision-making-node-avoidance.py` — Karar Alma (Kaçınma Modu)
- Durum makinesi: `NORMAL` → `OBSTACLE_DETECTED` → `EMERGENCY_STOP` → `OBSTACLE_AVOIDANCE_ESCAPE` → `OBSTACLE_AVOIDANCE_FOLLOW_LANE` → `OBSTACLE_AVOIDANCE_RETURN` → `NORMAL`
- `/lane/lateral_new_deviation` topic'ini kullanır

### `Communication/uart_sender_node.py` — UART Haberleşme
- 3 ayrı seri port üzerinden araç elektroniğine komut gönderir:
  - `speed_port`: İtki/fren
  - `steering_port`: Direksiyon
  - `stop_port`: Stop sinyali
- Port adları, baud rate ve PID kazançları `.env` dosyasından okunur (varsayılanlar: `/dev/ttyACM1`, `/dev/ttyACM0`, `/dev/ttyACM2`, 38400 baud)
- PID kontrolör ile lateral deviation'dan direksiyon açısı hesaplar (dt normalize)
- ROS2 parametreleri: `lateral_deviation_topic` (default: `/lane/lateral_deviation`)

---

## ROS2 Topic Haritası

| Topic | Tip | Yayıncı | Dinleyici |
|---|---|---|---|
| `/zed2i_rgb/image_raw` | `sensor_msgs/Image` | zedi2connect_port | serit-tespitcopy, run_tracker |
| `/lane/lateral_deviation` | `std_msgs/Float32` | serit-tespitcopy | uart_sender_node, decision-making |
| `/lane/lateral_new_deviation` | `std_msgs/Float32` | decision-making-avoidance | uart_sender_node (avoidance) |
| `/lane/intersection_direction` | `std_msgs/Int32` | serit-tespitcopy | decision-making |
| `/sign_detection/output` | `sensor_msgs/Image` | run_tracker | — (görsel çıktı) |
| `/obstacle_detected` | `std_msgs/Bool` | engel-tespit | decision-making |
| `/obstacle_distance` | `std_msgs/Float32` | engel-tespit | decision-making |
| `/obstacle_side` | `std_msgs/Bool` | engel-tespit | decision-making-avoidance |
| `/speed` | `std_msgs/Float32` | decision-making | uart_sender_node |
| `/ackermann_cmd` | `ackermann_msgs/AckermannDrive` | decision-making | uart_sender_node |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR sürücüsü | engel-tespit |

---

## UART Port Yapılandırması

| Port | Cihaz | Baud Rate | İşlev |
|---|---|---|---|
| `speed_port` | `/dev/ttyACM1` | 38400 | İtki / Fren |
| `steering_port` | `/dev/ttyACM0` | 38400 | Direksiyon |
| `stop_port` | `/dev/ttyACM2` | 38400 | Stop sinyali |

Port adları ve baud rate `.env` dosyasından yapılandırılabilir (`SERIAL_SPEED_PORT`, `SERIAL_STEERING_PORT`, `SERIAL_STOP_PORT`, `SERIAL_BAUD_RATE`).

---

## Web Arayüzü (Node.js)

`proje(1)/proje/` dizininde, sistemi **tarayıcı üzerinden** başlatıp durdurmak için bir Express.js backend ve React frontend bulunmaktadır.

### Yapı

```
proje/
├── backend/
│   ├── server.js        # Express.js API (port BACKEND_PORT, varsayılan: 3001)
│   └── scripts/
│       └── start_process.sh
└── frontend/            # React + Webpack (port 3000)
```

### Nasıl Çalışır?

`server.js` içindeki Express sunucusu, `GET /start` isteği geldiğinde tüm ROS2 node'larını sırayla `child_process.spawn()` ile başlatır. Her node arasında belirlenen süre kadar bekler. Başlatılan tüm process'lerin PID'leri hafızada tutulur. `GET /stop` isteğinde ise tüm bu PID'lere kill sinyali gönderilerek node'lar durdurulur.

```
Tarayıcı
  └─► React Frontend (localhost:3000)
          │  GET /start  →  Tüm Python node'ları sırayla spawn eder
          │  GET /stop   →  Tüm PID'lere kill gönderir
          └─► Express Backend (0.0.0.0:3001)
```

### Başlatma Sırası (`/start` endpoint'i)

| Adım | Node | Bekleme |
|------|------|---------|
| 1 | Kamera Yayını (`zedi2connect_port.py`) | 5s |
| 2 | UART Haberleşme (`uart_sender_node3.py`) | 2s |
| 3 | Şerit Tespit (`serit-tespitcopy.py`) | 3s |
| 4 | Levha Tespit (`run_tracker.py`) | 3s |
| 5 | LiDAR Engel Tespiti (`engel-tespit.py`) | 2s |
| 6 | Karar Alma (`basic-decision-making-node.py`) | 1s |

### Kurulum ve Çalıştırma

**Backend (port 3001):**
```bash
cd proje\(1\)/proje/backend
npm install
npm start          # node server.js
# veya geliştirme modunda:
npm run dev        # nodemon server.js (otomatik yeniden başlatma)
```

**Frontend (port 3000):**
```bash
cd proje\(1\)/proje/frontend
npm install
npm start          # webpack-dev-server (development)
npm run build      # production build
```

### API Endpoint'leri

| Method | Endpoint | Açıklama |
|--------|----------|----------|
| `GET` | `/start` | Tüm node'ları sırayla başlatır |
| `GET` | `/stop` | Tüm çalışan node'ları durdurur |
| `GET` | `/health` | Backend sağlık kontrolü |

> **Not:** Backend, ağdaki diğer cihazlardan da erişilebilmesi için `0.0.0.0` adresinde dinler.

---

## Loglama

Log seviyesi ve OpenCV ekran görüntüsü proje kökündeki `.env` dosyasından kontrol edilir. Tüm node'lar bu ayarları ortak `utils/ros_logger.py` ve `utils/config.py` modülleri aracılığıyla okur.

```bash
# .env
LOG_LEVEL=DEBUG   # tüm detaylar — her callback logu dahil (geliştirme)
LOG_LEVEL=INFO    # durum değişimleri, başlangıç bilgileri (varsayılan)
LOG_LEVEL=WARN    # yalnızca uyarı + hata
LOG_LEVEL=ERROR   # yalnızca kritik hatalar

CV_DISPLAY=true   # imshow pencereleri açılır (geliştirme / debug)
CV_DISPLAY=false  # pencere açılmaz, headless çalışır (yarışma / sunucu)
```

Hangi node'da hangi logun hangi seviyede yazıldığına dair tam rehber: **[utils/README.md](utils/README.md)**

---

## Kurulum

### 1. ROS2 Humble

```bash
# ROS2 Humble kurulumu (Ubuntu 22.04)
sudo apt update && sudo apt install ros-humble-desktop -y

# Gerekli ROS2 paketleri
sudo apt install ros-humble-cv-bridge ros-humble-ackermann-msgs -y

# Her oturumda ROS2'yi aktive et
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Python Bağımlılıkları

```bash
# Sanal ortam oluştur ve aktive et
python3 -m venv .venv
source .venv/bin/activate

# Paketleri yükle
pip install -r requirements.txt
```

> **Not:** `pyzed` kurulumu için ZED SDK'nın sistemde kurulu olması gerekir.
> ZED SDK: https://www.stereolabs.com/developers/release

### 3. Web Arayüzü (Node.js)

```bash
# Backend bağımlılıkları
cd proje\(1\)/proje/backend
npm install

# Frontend bağımlılıkları
cd ../frontend
npm install
```

---

## Bağımlılıklar

| Paket | Versiyon | Açıklama |
|-------|----------|----------|
| `torch` | ≥ 2.0.0 | Derin öğrenme |
| `ultralytics` | ≥ 8.0.0 | YOLO levha tespiti |
| `opencv-python` | ≥ 4.8.0 | Görüntü işleme |
| `numpy` | ≥ 1.24.0 | Sayısal hesaplama |
| `pyzed` | ≥ 4.0 | ZED2i kamera SDK |
| `pyserial` | ≥ 3.5 | UART haberleşme |
| `cv_bridge` | ROS2 | ROS2 ↔ OpenCV köprüsü |
| `ackermann_msgs` | ROS2 | Ackermann sürüş mesajları |
| `python-dotenv` | ≥ 1.0.0 | `.env` dosyası yükleme |
| `express` | ≥ 4.18 | Node.js backend API |
| `react` | ≥ 18.0 | Web arayüzü |
| `dotenv` (npm) | ≥ 16.0 | Node.js `.env` yükleme |

---

## Katkı ve Git Akışı

Branch isimlendirme, commit formatı, PR şablonu ve inceleme süreci için: **[CONTRIBUTING.md](CONTRIBUTING.md)**
