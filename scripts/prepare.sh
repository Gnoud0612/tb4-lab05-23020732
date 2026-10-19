#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
cd "${TB4_WORKSPACE}"
bash scripts/low_resource.sh
bash scripts/apply_sim_profile.sh --world maze
bash scripts/undock.sh --timeout 180
python3 scripts/spawn_objects.py
python3 scripts/initialize_pose.py
ros2 run tb4_lab check_topics --mapping --duration 15 --output evidence/topics.json
