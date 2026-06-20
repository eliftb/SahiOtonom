#!/bin/bash
# Gazebo + alzada_car + track driver — parkur.world

source /opt/ros/humble/setup.bash

export GAZEBO_MODEL_PATH=/home/elifnur/Desktop/sahi_otonom/models:$GAZEBO_MODEL_PATH

WORLD="/home/elifnur/Desktop/sahi_otonom/worlds/parkur.world"
MODEL_SDF="/home/elifnur/Desktop/sahi_otonom/models/alzada_car/model.sdf"
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "============================================"
echo "  ALZADA X CAR — GAZEBO PARKUR MODU"
echo "  Dünya  : parkur.world"
echo "  Model  : alzada_car"
echo "============================================"

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
    -x 0.0 -y 0.0 -z 0.5 \
    -R 0.0 -P 0.0 -Y 0.0
echo "  Spawn tamamlandı."
sleep 3

# 3. Track driver node'unu başlat
echo ""
echo "▶ Gazebo Track Driver başlatılıyor..."
cd "$BASE_DIR"
python3 launch_gazebo_test.py &
DRIVER_PID=$!
echo "  Track Driver PID: $DRIVER_PID"

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
    pkill -f gzserver 2>/dev/null
    pkill -f gzclient 2>/dev/null
    echo "Tüm işlemler durduruldu."
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $GAZEBO_PID
