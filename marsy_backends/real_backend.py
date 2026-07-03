import importlib
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REAL_PATH = PROJECT_ROOT / "4tronix" / "original"

if str(REAL_PATH) not in sys.path:
    sys.path.insert(0, str(REAL_PATH))

rover = importlib.import_module("rover")