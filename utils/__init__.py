"""
utils paketi — paylaşılan yardımcı araçlar.

Modüller:
    ros_logger  → LOG_LEVEL yönetimi
    config      → Donanım, PID ve kamera yapılandırması
"""

import os
from pathlib import Path

_ENV_LOADED = False
_PROJECT_ROOT = Path(__file__).parent.parent


def load_env() -> None:
    """
    Proje kökündeki .env dosyasını os.environ'a yükler.
    python-dotenv kuruluysa onu kullanır, yoksa manuel parse eder.
    Tekrar çağrılsa da yalnızca bir kez yüklenir.
    """
    global _ENV_LOADED
    if _ENV_LOADED:
        return
    env_path = _PROJECT_ROOT / '.env'
    if env_path.exists():
        try:
            from dotenv import load_dotenv
            load_dotenv(env_path, override=False)
        except ImportError:
            with open(env_path) as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        key, _, val = line.partition('=')
                        os.environ.setdefault(key.strip(), val.strip())
    _ENV_LOADED = True
