#!/usr/bin/env bash
# Source this file; all additional packages remain within this project.
# Debian packages contain a system setup script with a fixed /opt/ros prefix,
# so prepend our relocatable extracted prefix explicitly.
_tb4_local_project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_tb4_local_root="${_tb4_local_project}/.runtime/underlay"
_tb4_local_ros="${_tb4_local_root}/opt/ros/humble"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    printf '%s\n' 'ROS 2 Humble must first be installed at /opt/ros/humble.' >&2
    return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
if [[ -d "${_tb4_local_ros}" ]]; then
    export AMENT_PREFIX_PATH="${_tb4_local_ros}:${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="${_tb4_local_ros}:${CMAKE_PREFIX_PATH:-}"
    export COLCON_PREFIX_PATH="${_tb4_local_ros}:${COLCON_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${_tb4_local_ros}/lib:${_tb4_local_root}/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="${_tb4_local_ros}/local/lib/python3.10/dist-packages:${_tb4_local_ros}/lib/python3.10/site-packages:${PYTHONPATH:-}"
    export PATH="${_tb4_local_root}/usr/bin:${_tb4_local_ros}/bin:${PATH}"
    export QML2_IMPORT_PATH="${_tb4_local_root}/usr/lib/x86_64-linux-gnu/qt5/qml:${QML2_IMPORT_PATH:-}"
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${_tb4_local_ros}/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
    export IGN_GUI_PLUGIN_PATH="${_tb4_local_ros}/lib:${IGN_GUI_PLUGIN_PATH:-}"
    export IGN_GAZEBO_RESOURCE_PATH="${_tb4_local_ros}/share:${IGN_GAZEBO_RESOURCE_PATH:-}"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="${_tb4_local_ros}/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    export GZ_GUI_PLUGIN_PATH="${_tb4_local_ros}/lib:${GZ_GUI_PLUGIN_PATH:-}"
    export GZ_SIM_RESOURCE_PATH="${_tb4_local_ros}/share:${GZ_SIM_RESOURCE_PATH:-}"
fi
unset _tb4_local_project _tb4_local_root _tb4_local_ros
