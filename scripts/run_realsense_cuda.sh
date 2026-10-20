#!/usr/bin/env bash

set -e

source /opt/ros/humble/setup.bash

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/install/setup.bash"

exec ros2 launch launcher cuda_realsense.launch.py "$@"
