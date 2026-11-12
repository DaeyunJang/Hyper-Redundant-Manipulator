#!/usr/bin/env bash
# AUTHOR: Daeyun - container-local runtime initialization; never operates motors.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/hrm_ws/.cuda_realsense/install/realsense-ros/local_setup.bash
source /opt/hrm_ws/install/setup.bash
export LD_LIBRARY_PATH="/opt/hrm_ws/.cuda_realsense/install/librealsense/lib:${LD_LIBRARY_PATH:-}"
export HRM_INFERENCE_PYTHON=/usr/bin/python3

export XDG_RUNTIME_DIR="/tmp/hrm-runtime-$(id -u)"
mkdir -p "$XDG_RUNTIME_DIR" /home/hrm/config
chmod 700 "$XDG_RUNTIME_DIR"
for name in config.json config_ROI_ref.json id_localization_config.json; do
    target="/home/hrm/config/$name"
    if [[ ! -e "$target" ]]; then
        cp -- "/opt/hrm_ws/docker/defaults/$name" "$target"
    fi
done

cd /opt/hrm_ws
if [[ $# -eq 0 ]]; then
    set -- ros2 launch launcher system.launch.py auto_start_components:=false
fi
exec "$@"

