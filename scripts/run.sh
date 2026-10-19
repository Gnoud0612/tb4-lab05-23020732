#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
mode="${1:-simulation}"
shift || true
printf '%s\n' "$$" > "${TB4_WORKSPACE}/.runtime/launch.pid"
case "${mode}" in
    simulation|mapping) exec ros2 launch tb4_lab "${mode}.launch.py" "$@" ;;
    localization|navigation)
        [[ -s "${TB4_WORKSPACE}/maps/map.yaml" ]] || {
            printf '%s\n' 'Chưa có maps/map.yaml. Hãy chạy mapping rồi scripts/save_map.sh.' >&2
            exit 1
        }
        exec ros2 launch tb4_lab "${mode}.launch.py" "map:=${TB4_WORKSPACE}/maps/map.yaml" "$@"
        ;;
    *) printf '%s\n' 'Usage: scripts/run.sh simulation|mapping|localization|navigation [launch arguments]' >&2; exit 2 ;;
esac
