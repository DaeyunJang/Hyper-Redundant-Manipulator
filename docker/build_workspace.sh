#!/usr/bin/env bash
# AUTHOR: Daeyun
# Image-build helper only; never run this script against a host workspace.
set -eo pipefail

if [[ "${HRM_CONTAINER_BUILD:-}" != 1 || "$PWD" != /opt/hrm_ws ]]; then
    printf 'Run this helper only during the HRM image build in /opt/hrm_ws.\n' >&2
    exit 2
fi
source /opt/ros/humble/setup.bash
set -u

build_jobs="${BUILD_JOBS:-4}"
cuda_arch="${CUDA_ARCH:-86}"
rsusb_backend="${RSUSB_BACKEND:-OFF}"
[[ "$build_jobs" =~ ^[1-9][0-9]*$ ]] || { printf 'Invalid BUILD_JOBS\n' >&2; exit 2; }
[[ "$cuda_arch" =~ ^[0-9]{2,3}$ ]] || { printf 'Invalid CUDA_ARCH\n' >&2; exit 2; }
[[ "$rsusb_backend" == ON || "$rsusb_backend" == OFF ]] || {
    printf 'RSUSB_BACKEND must be ON or OFF\n' >&2; exit 2;
}
export CMAKE_BUILD_PARALLEL_LEVEL="$build_jobs"
cuda_root=/opt/hrm_ws/.cuda_realsense

case "${1:-}" in
vendor)
    mkdir -p "$cuda_root/src"
    git clone --depth 1 --branch v2.55.1 \
        https://github.com/realsenseai/librealsense.git "$cuda_root/src/librealsense"
    # v2.55.1 hard-codes older GPU targets in FindCUDA flags. Emit both native
    # code for the selected architecture and PTX for newer compatible GPUs.
    sed -i \
        "s|^set(CUDA_NVCC_FLAGS .*|set(CUDA_NVCC_FLAGS \"\${CUDA_NVCC_FLAGS}; -O3 -gencode arch=compute_${cuda_arch},code=sm_${cuda_arch} -gencode arch=compute_${cuda_arch},code=compute_${cuda_arch};\")|" \
        "$cuda_root/src/librealsense/CMake/cuda_config.cmake"
    cmake -S "$cuda_root/src/librealsense" -B "$cuda_root/build/librealsense" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$cuda_root/install/librealsense" \
        -DCMAKE_INSTALL_LIBDIR=lib \
        -DCMAKE_CUDA_ARCHITECTURES="$cuda_arch" \
        -DBUILD_WITH_CUDA=ON -DBUILD_SHARED_LIBS=ON \
        -DFORCE_RSUSB_BACKEND="$rsusb_backend" \
        -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_GLSL_EXTENSIONS=OFF -DBUILD_TOOLS=OFF \
        -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF \
        -DIMPORT_DEPTH_CAM_FW=OFF -DCHECK_FOR_UPDATES=OFF
    cmake --build "$cuda_root/build/librealsense" --parallel "$build_jobs"
    cmake --install "$cuda_root/build/librealsense"

    git clone --depth 1 --branch 4.55.1 \
        https://github.com/realsenseai/realsense-ros.git "$cuda_root/src/realsense-ros"
    colcon --log-base "$cuda_root/log/realsense-ros" build \
        --base-paths "$cuda_root/src/realsense-ros" \
        --packages-select realsense2_camera_msgs realsense2_camera \
        --build-base "$cuda_root/build/realsense-ros" \
        --install-base "$cuda_root/install/realsense-ros" \
        --merge-install --parallel-workers "$build_jobs" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -Drealsense2_DIR="$cuda_root/install/librealsense/lib/cmake/realsense2" \
        -DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF \
        -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
    {
        printf 'AUTHOR: Daeyun\nlibrealsense=2.55.1\nrealsense-ros=4.55.1\n'
        printf 'CUDA_ARCH=%s\nRSUSB_BACKEND=%s\n' "$cuda_arch" "$rsusb_backend"
        git -C "$cuda_root/src/librealsense" rev-parse HEAD
        git -C "$cuda_root/src/realsense-ros" rev-parse HEAD
    } > /opt/hrm_ws/docker/build-info.txt
    ;;
workspace)
    # Sourcing ROS/colcon setup files is incompatible with nounset.
    set +u
    source "$cuda_root/install/realsense-ros/local_setup.bash"
    set -u
    colcon --log-base /opt/hrm_ws/log build \
        --base-paths /opt/hrm_ws/src \
        --packages-select custom_interfaces image_rate_probe estimation_pkg \
            gui_py_pkg serial_pkg record_pkg robot_control_pkg launcher \
        --build-base /opt/hrm_ws/build --install-base /opt/hrm_ws/install \
        --merge-install --parallel-workers "$build_jobs" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
    mkdir -p /opt/hrm_ws/docker/defaults
    for filename in config.json config_ROI_ref.json id_localization_config.json; do
        cp "/opt/hrm_ws/src/estimation_pkg/estimation_pkg/$filename" \
            "/opt/hrm_ws/docker/defaults/$filename"
        ln -sfn "/home/hrm/config/$filename" "/opt/hrm_ws/install/share/estimation_pkg/$filename"
    done
    # Installed GUI model discovery uses the package share directory. The
    # actual trusted bundles remain a read-only external bind mount.
    ln -s /opt/hrm_ws/src/estimation_pkg/models /opt/hrm_ws/install/share/estimation_pkg/models
    ;;
*)
    printf 'Usage: %s {vendor|workspace}\n' "$0" >&2
    exit 2
    ;;
esac
