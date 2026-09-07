#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
LOCAL_AGX_ARM_WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)/agx_arm_ws"

source "$ROS_SETUP"

if [[ -n "${AGX_ARM_WS:-}" ]]; then
    source "$AGX_ARM_WS/install/setup.bash"
elif ! ros2 pkg prefix agx_arm_ctrl >/dev/null 2>&1; then
    if [[ -f "$LOCAL_AGX_ARM_WS/install/setup.bash" ]]; then
        export AGX_ARM_WS="$LOCAL_AGX_ARM_WS"
        source "$AGX_ARM_WS/install/setup.bash"
    else
        echo "未找到 agx_arm_ctrl。请先 source 工作区，或设置 AGX_ARM_WS。" >&2
        exit 1
    fi
fi

exec python3 "$SCRIPT_DIR/piper_gui.py" "$@"
