from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'

lidar = RPLidar(PORT_NAME, baudrate=256000)

try:
    print("Lidar başlatıldı. Ölçümler geliyor...\n")
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            print(f"Açı: {angle:.2f}°, Mesafe: {distance:.2f} mm")
except KeyboardInterrupt:
    print("\nDurduruluyor...")
finally:
    print("Bağlantı kapatılıyor...")
    lidar.stop()
    lidar.disconnect()
