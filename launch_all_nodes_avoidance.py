import multiprocessing
import time
import os
import sys
import importlib.util
import pathlib

BASE_DIR = pathlib.Path(__file__).parent


NODE_LAUNCH_ORDER = [
    {
        "name": "CAMERA (ZED)",
        "file_path": str(BASE_DIR / 'Camera' / 'zedi2connect_port.py'),
        "delay_after": 5
    },
    {
        "name": "COMMUNICATION (UART) - AVOIDANCE",
        "file_path": str(BASE_DIR / 'Communication' / 'uart_sender_node.py'),
        "ros_args": ["--ros-args", "-p", "lateral_deviation_topic:=/lane/lateral_new_deviation"],
        "delay_after": 2
    },
    {
        "name": "LANE DETECTION",
        "file_path": str(BASE_DIR / 'LaneDetection' / 'lane_detection.py'),
        "delay_after": 3
    },
    {
        "name": "SIGN DETECTION",
        "file_path": str(BASE_DIR / 'SignDetection' / 'run_tracker.py'),
        "delay_after": 3
    },
    {
        "name": "LIDAR OBSTACLE DETECTION",
        "file_path": str(BASE_DIR / 'ObstacleDetection' / 'obstacle_detection.py'),
        "delay_after": 2,
        "ros_args": ["--ros-args", "-p", "scan_topic:=/scan"],
    },
    {
        "name": "DECISION MAKING - AVOIDANCE",
        "file_path": str(BASE_DIR / 'DecisionMaking' / 'decision-making-node-avoidance.py'),
        "delay_after": 1
    }
]

def run_script(script_path, ros_args=None):
    """Verilen tam yoldaki Python script'ini çalıştırır."""
    try:
        script_dir = os.path.dirname(script_path)
        sys.path.insert(0, script_dir)

        print(f"[{os.getpid()}] Proses başlatıldı: {os.path.basename(script_path)}")

        # ROS2 argümanlarını sys.argv'e aktar
        if ros_args:
            original_argv = sys.argv[:]
            sys.argv = [script_path] + ros_args

        module_name = os.path.splitext(os.path.basename(script_path))[0]
        spec = importlib.util.spec_from_file_location(module_name, script_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)

        if hasattr(module, 'main'):
            module.main()
        else:
            print(f"HATA: {script_path} içinde 'main' fonksiyonu bulunamadı.", file=sys.stderr)

        # Eklenen yolu ve argv'yi temizle
        sys.path.pop(0)
        if ros_args:
            sys.argv = original_argv

    except Exception as e:
        print(f"[{os.getpid()}] HATA ({os.path.basename(script_path)}): {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    processes = []

    print("--- OTONOM SİSTEM SIRAYLA BAŞLATILIYOR (Avoidance Versiyonu) ---")

    try:
        for node_info in NODE_LAUNCH_ORDER:
            node_name = node_info["name"]
            path = node_info["file_path"]
            delay = node_info["delay_after"]
            ros_args = node_info.get("ros_args")

            if not os.path.exists(path):
                print(f"\n‼️ UYARI: '{node_name}' için dosya bulunamadı, atlanıyor: {path}")
                continue

            print(f"\n▶️  '{node_name}' başlatılıyor...")
            process = multiprocessing.Process(target=run_script, args=(path, ros_args))
            processes.append(process)
            process.start()

            print(f"    PID {process.pid} ile başlatıldı. Sonraki düğüm için {delay} saniye bekleniyor...")
            time.sleep(delay)

        print("\n✅ Tüm düğümler başarıyla ve sırayla başlatıldı.")
        print("Kapatmak için terminalde CTRL+C tuşlarına basın.")

        for process in processes:
            process.join()

    except KeyboardInterrupt:
        print("\n\n--- KAPATMA SİNYALİ ALINDI (CTRL+C) ---")
        print("Tüm düğümler sonlandırılıyor...")
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join(timeout=2)

        for process in processes:
            if process.is_alive():
                print(f"PID {process.pid} zorla kapatılıyor...")
                process.kill()
                process.join()

        print("🛑 Tüm işlemler güvenli bir şekilde sonlandırıldı.")

