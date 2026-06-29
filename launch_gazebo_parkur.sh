#!/bin/bash
# Gazebo + alzada_car + merkezi DecisionMaking — parkur.world

source /opt/ros/humble/setup.bash

BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
TRAFFIC_LIGHT_PLUGIN_SRC="$BASE_DIR/Gazebo/traffic_light_cycle_plugin.cc"
TRAFFIC_LIGHT_PLUGIN_DIR="$BASE_DIR/Gazebo/plugins"
TRAFFIC_LIGHT_PLUGIN="$TRAFFIC_LIGHT_PLUGIN_DIR/libTrafficLightCyclePlugin.so"

export GAZEBO_MODEL_PATH=/home/elifnur/Desktop/sahi_otonom/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH="$TRAFFIC_LIGHT_PLUGIN_DIR:$GAZEBO_PLUGIN_PATH"

WORLD="/home/elifnur/Desktop/sahi_otonom/worlds/parkur.world"
MODEL_SDF="/home/elifnur/Desktop/sahi_otonom/models/alzada_car/model.sdf"

if [ ! -f "$TRAFFIC_LIGHT_PLUGIN" ] || [ "$TRAFFIC_LIGHT_PLUGIN_SRC" -nt "$TRAFFIC_LIGHT_PLUGIN" ]; then
    echo "▶ Trafik ışığı eklentisi derleniyor..."
    mkdir -p "$TRAFFIC_LIGHT_PLUGIN_DIR"
    g++ -shared -fPIC "$TRAFFIC_LIGHT_PLUGIN_SRC" \
        -o "$TRAFFIC_LIGHT_PLUGIN" \
        $(pkg-config --cflags --libs gazebo)
    if [ $? -ne 0 ]; then
        echo "HATA: Trafik ışığı eklentisi derlenemedi."
        exit 1
    fi
fi

echo "============================================"
echo "  ALZADA X CAR — GAZEBO PARKUR MODU"
echo "  Dünya  : parkur.world"
echo "  Model  : alzada_car"
echo "============================================"

# 0. Önceki Gazebo süreçlerini temizle (port 11345 çakışmasını / "Address already in use" hatasını önler)
echo ""
echo "▶ Önceki Gazebo süreçleri temizleniyor..."
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f "gazebo.launch.py" 2>/dev/null
pkill -9 -f "spawn_entity.py" 2>/dev/null
pkill -9 -f "launch_gazebo_test.py" 2>/dev/null
pkill -9 -f "gazebo_track_driver.py" 2>/dev/null
pkill -9 -f "decision-making-node-avoidance.py" 2>/dev/null
pkill -9 -f "gazebo_actuator_bridge.py" 2>/dev/null
pkill -9 -f "gazebo_lane_detection.py" 2>/dev/null
pkill -9 -f "obstacle_detection.py" 2>/dev/null
pkill -9 -f "SignDetection/run_tracker.py" 2>/dev/null
sleep 2

# 1. Gazebo'yu arka planda başlat (ros2 launch ile ROS2 plugin'leri otomatik yüklenir)
echo ""
echo "▶ Gazebo başlatılıyor..."
ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD" verbose:=true &
GAZEBO_PID=$!
echo "  Gazebo PID: $GAZEBO_PID"

# Gazebo ve ROS2 plugin'lerinin hazır olması için bekle
echo "  15 saniye bekleniyor..."
sleep 15

# 2. Alzada car'ı sahneye spawn et
echo ""
echo "▶ alzada_car spawn ediliyor..."
ros2 run gazebo_ros spawn_entity.py \
    -file "$MODEL_SDF" \
    -entity alzada_car \
    -x 23.196297 -y -10.799116 -z 0.5 \
    -R 0.0 -P 0.0 -Y 3.096073
echo "  Spawn tamamlandı."
sleep 3

# 3. Algılama + DecisionMaking + actuator zincirini başlat
echo ""
echo "▶ Merkezi otonom kontrol zinciri başlatılıyor..."
cd "$BASE_DIR"
python3 launch_gazebo_test.py &
DRIVER_PID=$!
echo "  Kontrol zinciri PID: $DRIVER_PID"

echo ""
echo "============================================"
echo "  Tüm bileşenler çalışıyor."
echo "  Durdurmak için CTRL+C"
echo "============================================"

# CTRL+C geldiğinde her şeyi temizle
cleanup() {
    echo ""
    echo "--- Kapatılıyor ---"
    kill $DRIVER_PID 2>/dev/null
    kill $GAZEBO_PID 2>/dev/null
    pkill -f "launch_gazebo_test.py" 2>/dev/null
    pkill -f "decision-making-node-avoidance.py" 2>/dev/null
    pkill -f "gazebo_actuator_bridge.py" 2>/dev/null
    pkill -f "gazebo_lane_detection.py" 2>/dev/null
    pkill -f "obstacle_detection.py" 2>/dev/null
    pkill -f "SignDetection/run_tracker.py" 2>/dev/null
    pkill -f gzserver 2>/dev/null
    pkill -f gzclient 2>/dev/null
    echo "Tüm işlemler durduruldu."
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $GAZEBO_PID
