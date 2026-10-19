#!/usr/bin/env bash
# Usage: source scripts/env.bash
TB4_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export TB4_WORKSPACE
source "${TB4_WORKSPACE}/scripts/local_underlay.bash" || return 1
if [[ -f "${TB4_WORKSPACE}/install/local_setup.bash" ]]; then
    source "${TB4_WORKSPACE}/install/local_setup.bash"
fi
export ROS_DOMAIN_ID="${TB4_ROS_DOMAIN_ID:-55}"
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="tb4_lab_domain${ROS_DOMAIN_ID}"
export GZ_PARTITION="${IGN_PARTITION}"
export ROS_LOG_DIR="${TB4_WORKSPACE}/.runtime/ros_logs"
export ROS_HOME="${TB4_WORKSPACE}/.runtime/ros"
export IGN_LOG_PATH="${TB4_WORKSPACE}/.runtime/ignition_logs"
export IGN_FUEL_CACHE_PATH="${TB4_WORKSPACE}/.runtime/fuel"
export XDG_CACHE_HOME="${TB4_WORKSPACE}/.runtime/cache"
export XDG_CONFIG_HOME="${TB4_WORKSPACE}/.runtime/config"
export XDG_DATA_HOME="${TB4_WORKSPACE}/.runtime/data"
mkdir -p "${ROS_LOG_DIR}" "${ROS_HOME}" "${IGN_LOG_PATH}" "${IGN_FUEL_CACHE_PATH}" \
    "${XDG_CACHE_HOME}" "${XDG_CONFIG_HOME}" "${XDG_DATA_HOME}"

# YOLO dependencies are isolated; keep the system cv_bridge and ROS available.
if [[ -d "${TB4_WORKSPACE}/.venv/lib/python3.10/site-packages" ]]; then
  export PYTHONPATH="${TB4_WORKSPACE}/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi
export YOLO_CONFIG_DIR="${TB4_WORKSPACE}/.runtime/ultralytics"
export MPLCONFIGDIR="${TB4_WORKSPACE}/.runtime/matplotlib"
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=2
export LP_NUM_THREADS=2
export QT_X11_NO_MITSHM=1
