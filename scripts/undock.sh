#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
export PYTHONPATH="${TB4_WORKSPACE}/src/tb4_lab:${PYTHONPATH:-}"
exec python3 -m tb4_lab.undock "$@"
