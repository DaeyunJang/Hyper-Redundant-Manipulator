#!/usr/bin/env bash
# Isolated experiment only: does not install or replace system ROS/CUDA packages.
# Usage: bash scripts/with_isaac_ros_compare.sh /usr/bin/python3 <comparison.py> ...
set -eo pipefail

compare_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
compare_root="${compare_repo_root}/.isaac_ros_compare"
compare_ros_prefix="${compare_root}/root/opt/ros/humble"
compare_negotiated_prefix="${compare_root}/negotiated_install"

if [[ $# -eq 0 ]]; then
    echo 'Usage: bash scripts/with_isaac_ros_compare.sh COMMAND [ARGUMENTS...]' >&2
    exit 2
fi
if [[ ! -f "${compare_ros_prefix}/lib/libapriltag_node.so" ||
      ! -f "${compare_negotiated_prefix}/local_setup.bash" ]]; then
    echo 'Isolated Isaac ROS comparison dependencies are missing; see docs/APRILTAG_TUNING.md.' >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source "${compare_negotiated_prefix}/local_setup.bash"
set -u

# Extracted Debian setup files refer to /opt/ros/humble; use the staged prefix
# explicitly so all package resources and message bindings stay local.
export AMENT_PREFIX_PATH="${compare_ros_prefix}:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="${compare_ros_prefix}/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${compare_ros_prefix}/lib:${compare_negotiated_prefix}/lib:/usr/local/cuda-12.4/lib64:${LD_LIBRARY_PATH:-}"
# GXF ships additional shared libraries in package resource directories.
while IFS= read -r compare_lib_directory; do
    export LD_LIBRARY_PATH="${compare_lib_directory}:${LD_LIBRARY_PATH}"
done < <(rg --files "${compare_ros_prefix}/share" | sed -n 's@/[^/]*\.so[^/]*$@@p' | sort -u)
if [[ -d "${compare_root}/root/opt/nvidia/vpi3/lib/x86_64-linux-gnu" ]]; then
    export LD_LIBRARY_PATH="${compare_root}/root/opt/nvidia/vpi3/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${compare_repo_root}/src/estimation_pkg/config/fastdds_images.xml}"

exec "$@"
