const path = require('path');
// Proje kökündeki .env dosyasını yükle (backend/ → proje/ → proje(1)/ → SahiOtonom/)
require('dotenv').config({ path: path.join(__dirname, '..', '..', '..', '.env') });

const express = require('express');
const cors = require('cors');
const { spawn } = require('child_process');
const os = require('os');

const app = express();
app.use(cors());
app.use(express.json());

const PORT = parseInt(process.env.BACKEND_PORT || '3001', 10);

// Ağ IP adresini bul
function getLocalIp() {
  const interfaces = os.networkInterfaces();
  for (let iface of Object.values(interfaces)) {
    for (let alias of iface) {
      if (alias.family === 'IPv4' && !alias.internal) {
        return alias.address;
      }
    }
  }
  return 'localhost';
}

// Çalıştırılacak komut listesi
const NODE_LAUNCH_ORDER = [
  {
    name: "CAMERA (ZED)",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/Camera/zedi2connect_port.py"],
    delay_after: 5000
  },
  {
    name: "COMMUNICATION (UART)",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/Communication/uart_sender_node.py"],
    delay_after: 2000
  },
  {
    name: "LANE DETECTION",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/LaneDetection/lane_detection.py"],
    delay_after: 3000
  },
  {
    name: "SIGN DETECTION",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/SignDetection/run_tracker.py"],
    delay_after: 3000
  },
  {
    name: "LIDAR OBSTACLE DETECTION",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/ObstacleDetection/obstacle_detection.py"],
    delay_after: 2000
  },
  {
    name: "DECISION MAKING",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/DecisionMaking/basic-decision-making-node.py"],
    delay_after: 1000
  }
];

let isRunning = false;
let runningProcesses = [];

function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

app.get('/start', async (req, res) => {
  if (isRunning) {
    return res.status(400).json({ success: false, message: 'Zaten çalışıyor' });
  }

  isRunning = true;
  runningProcesses = [];

  try {
    for (const step of NODE_LAUNCH_ORDER) {
      console.log(`🚀 Başlatılıyor: ${step.cmd} ${step.args.join(' ')}`);

      const child = spawn(step.cmd, step.args, { stdio: 'inherit' });
      runningProcesses.push(child);

      console.log(`PID: ${child.pid}`);

      if (step.delay_after) {
        await delay(step.delay_after);
      }
    }

    res.json({ success: true, message: 'Tüm komutlar başlatıldı', pids: runningProcesses.map(p => p.pid) });
  } catch (err) {
    console.error('❌ Başlatma hatası:', err);
    res.status(500).json({ success: false, message: 'Başlatma sırasında hata oluştu' });
  }
});

app.get('/stop', (req, res) => {
  if (!isRunning) {
    return res.status(400).json({ success: false, message: 'Zaten çalışmıyor' });
  }

  runningProcesses.forEach(proc => {
    try {
      proc.kill();
      console.log(`🛑 PID ${proc.pid} sonlandırıldı`);
    } catch (err) {
      console.error(`PID ${proc.pid} kapatılamadı:`, err);
    }
  });

  runningProcesses = [];
  isRunning = false;

  res.json({ success: true, message: 'Tüm processler durduruldu' });
});

app.listen(PORT, '0.0.0.0', () => {
  console.log(`✅ Backend http://${getLocalIp()}:${PORT} üzerinden erişilebilir`);
});
