#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
exec ros2 launch tb4_yolo_nav perception.launch.py "$@"
