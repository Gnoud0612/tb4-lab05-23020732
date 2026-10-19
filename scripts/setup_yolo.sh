#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
# Uses system ROS/cv_bridge; all ML packages live in this venv.
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install pip==24.3.1
.venv/bin/python -m pip install --index-url https://download.pytorch.org/whl/cpu \
  torch==2.5.1+cpu torchvision==0.20.1+cpu
.venv/bin/python -m pip install -r requirements-yolo.txt
mkdir -p weights
if [[ ! -s weights/yolov8n.pt ]]; then
  .venv/bin/python - <<'PY'
from urllib.request import urlopen
from pathlib import Path
url='https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt'
with urlopen(url,timeout=60) as response:
    Path('weights/yolov8n.pt').write_bytes(response.read())
PY
fi

.venv/bin/python - <<'PY'
from pathlib import Path
import hashlib,json
p=Path("weights/yolov8n.pt")
m=json.loads(Path("weights/manifest.json").read_text())
assert hashlib.sha256(p.read_bytes()).hexdigest()==m["sha256"], "YOLO weights checksum mismatch"
print("YOLOv8n weights checksum verified")
PY
