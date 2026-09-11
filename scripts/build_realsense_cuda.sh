#!/usr/bin/env bash

set -e

source /opt/ros/humble/setup.bash
set -u

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cuda_root="${repo_root}/.cuda_realsense"
librealsense_src="${cuda_root}/src/librealsense"
librealsense_build="${cuda_root}/build/librealsense"
librealsense_install="${cuda_root}/install/librealsense"
realsense_ros_src="${cuda_root}/src/realsense-ros"
realsense_ros_build="${cuda_root}/build/realsense-ros"
realsense_ros_install="${cuda_root}/install/realsense-ros"
realsense_ros_log="${cuda_root}/log/realsense-ros"
cuda_arch="${LRS_CUDA_ARCH:-86}"
parallel_jobs="${LRS_BUILD_JOBS:-4}"

mkdir -p "${cuda_root}/src" "${cuda_root}/build" "${cuda_root}/install"

if [[ ! -d "${librealsense_src}/.git" ]]; then
  git clone --depth 1 --branch v2.55.1 \
    https://github.com/realsenseai/librealsense.git "${librealsense_src}"
fi

cuda_config="${librealsense_src}/CMake/cuda_config.cmake"
if ! grep -q "arch=compute_${cuda_arch},code=sm_${cuda_arch}" "${cuda_config}"; then
  sed -i \
    "s|^set(CUDA_NVCC_FLAGS .*|set(CUDA_NVCC_FLAGS \"\${CUDA_NVCC_FLAGS}; -O3 -gencode arch=compute_${cuda_arch},code=sm_${cuda_arch} -gencode arch=compute_${cuda_arch},code=compute_${cuda_arch};\")|" \
    "${cuda_config}"
fi

cmake -S "${librealsense_src}" -B "${librealsense_build}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${librealsense_install}" \
  -DBUILD_WITH_CUDA=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DIMPORT_DEPTH_CAM_FW=OFF

cmake --build "${librealsense_build}" --parallel "${parallel_jobs}"
cmake --install "${librealsense_build}"

if [[ ! -d "${realsense_ros_src}/.git" ]]; then
  git clone --depth 1 --branch 4.55.1 \
    https://github.com/realsenseai/realsense-ros.git "${realsense_ros_src}"
fi

colcon --log-base "${realsense_ros_log}" build \
  --base-paths "${realsense_ros_src}" \
  --build-base "${realsense_ros_build}" \
  --install-base "${realsense_ros_install}" \
  --merge-install \
  --symlink-install \
  --parallel-workers "${parallel_jobs}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -Drealsense2_DIR="${librealsense_install}/lib/cmake/realsense2" \
    -DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF

echo "CUDA RealSense build completed."
echo "Run: ${repo_root}/scripts/run_realsense_cuda.sh"
