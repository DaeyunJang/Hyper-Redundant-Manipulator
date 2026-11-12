# syntax=docker/dockerfile:1
# AUTHOR: Daeyun
# Build from the repository root: docker build -f docker/Dockerfile .
# The CUDA compiler is used only in the builder; inference uses CPU PyTorch.
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04 AS cuda-toolchain

FROM nvidia/cuda:12.4.1-runtime-ubuntu22.04 AS runtime-base
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_APT_SOURCE_VERSION=1.3.0
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# ros2-apt-source supplies the maintained ROS signing key and Jammy apt source.
# All apt/pip installs in this recipe affect the image, never the Docker host.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl \
    && curl --fail --show-error --location --retry 3 \
       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb" \
       --output /tmp/ros2-apt-source.deb \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       python3 python3-pip python3-setuptools python3-yaml \
       python3-opencv python3-pyqt5 \
       libusb-1.0-0 libudev1 libssl3 libgomp1 libgl1 \
       libxkbcommon-x11-0 libxcb-xinerama0 fonts-dejavu-core \
       ros-humble-ros-base \
       ros-humble-apriltag-ros \
       ros-humble-cv-bridge \
       ros-humble-diagnostic-updater \
       ros-humble-image-proc \
       ros-humble-image-transport \
       ros-humble-rclcpp-components \
       ros-humble-rmw-fastrtps-cpp \
       ros-humble-robot-state-publisher \
       ros-humble-rosbag2-py \
       ros-humble-rosbag2-storage-default-plugins \
       ros-humble-sensor-msgs-py \
       ros-humble-tf2-geometry-msgs \
       ros-humble-urdf \
       ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY docker/requirements-runtime.txt /opt/hrm_ws/docker/requirements-runtime.txt
# Install the pinned stack into /usr/local without attempting to uninstall
# Ubuntu-owned NumPy/Matplotlib metadata under /usr/lib/python3/dist-packages.
RUN /usr/bin/python3 -m pip install --no-cache-dir --ignore-installed \
        -r /opt/hrm_ws/docker/requirements-runtime.txt \
    && /usr/bin/python3 -m pip install --no-cache-dir \
        --index-url https://download.pytorch.org/whl/cpu 'torch==2.7.1+cpu'

FROM runtime-base AS builder
ARG DEBIAN_FRONTEND=noninteractive
ARG CUDA_ARCH=86
ARG BUILD_JOBS=4
ARG RSUSB_BACKEND=OFF
# Copy the matching development toolkit without adding it to runtime layers.
COPY --from=cuda-toolchain /usr/local/cuda-12.4 /usr/local/cuda-12.4
ENV PATH=/usr/local/cuda-12.4/bin:${PATH} \
    HRM_CONTAINER_BUILD=1
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential cmake git pkg-config python3-dev \
       python3-colcon-common-extensions libeigen3-dev libssl-dev \
       libusb-1.0-0-dev libudev-dev \
       ros-humble-eigen3-cmake-module ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/hrm_ws
COPY docker/build_workspace.sh /opt/hrm_ws/docker/build_workspace.sh
# Build the expensive vendor dependencies before copying frequently edited code.
RUN CUDA_ARCH="${CUDA_ARCH}" BUILD_JOBS="${BUILD_JOBS}" \
    RSUSB_BACKEND="${RSUSB_BACKEND}" bash docker/build_workspace.sh vendor

# Explicit allowlist for the complete native GUI/system runtime, including TCP
# and control. The unused DAQ package, firmware, and training data stay outside.
COPY src/custom_interfaces/ src/custom_interfaces/
COPY src/image_rate_probe/ src/image_rate_probe/
COPY src/estimation_pkg/ src/estimation_pkg/
COPY src/gui_py_pkg/ src/gui_py_pkg/
COPY src/serial_pkg/ src/serial_pkg/
COPY src/record_pkg/ src/record_pkg/
COPY src/robot_control_pkg/ src/robot_control_pkg/
COPY src/launcher/ src/launcher/
COPY src/tcp_pkg/ src/tcp_pkg/
RUN BUILD_JOBS="${BUILD_JOBS}" bash docker/build_workspace.sh workspace

FROM runtime-base AS runtime
LABEL org.opencontainers.image.title="HRM GUI, sensor, inference, and control runtime" \
      org.opencontainers.image.authors="Daeyun"
ENV HRM_INFERENCE_PYTHON=/usr/bin/python3 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
    QT_X11_NO_MITSHM=1 \
    OPENBLAS_NUM_THREADS=1 \
    OMP_NUM_THREADS=1 \
    MKL_NUM_THREADS=1 \
    MPLCONFIGDIR=/tmp/hrm-matplotlib \
    NUMBA_CACHE_DIR=/tmp/hrm-numba
WORKDIR /opt/hrm_ws
COPY --from=builder /opt/hrm_ws/install/ /opt/hrm_ws/install/
COPY --from=builder /opt/hrm_ws/.cuda_realsense/install/ /opt/hrm_ws/.cuda_realsense/install/
COPY --from=builder /opt/hrm_ws/docker/defaults/ /opt/hrm_ws/docker/defaults/
COPY --from=builder /opt/hrm_ws/docker/build-info.txt /opt/hrm_ws/docker/build-info.txt
COPY docker/smoke_test.py /opt/hrm_ws/docker/smoke_test.py
COPY docker/entrypoint.sh /usr/local/bin/hrm-entrypoint
RUN groupadd --gid 1000 hrm \
    && useradd --uid 1000 --gid 1000 --create-home --shell /bin/bash hrm \
    && install -d -o hrm -g hrm /home/hrm/config /opt/hrm_ws/record \
       /opt/hrm_ws/src/estimation_pkg/models \
    && chmod 755 /usr/local/bin/hrm-entrypoint \
    && cp /opt/hrm_ws/docker/defaults/*.json /home/hrm/config/ \
    && chown hrm:hrm /home/hrm/config/*.json
USER hrm
# Import/inference/Qt checks run without devices, a display, ROS nodes, or TCP.
RUN source /opt/ros/humble/setup.bash \
    && source /opt/hrm_ws/.cuda_realsense/install/realsense-ros/local_setup.bash \
    && source /opt/hrm_ws/install/local_setup.bash \
    && export LD_LIBRARY_PATH="/opt/hrm_ws/.cuda_realsense/install/librealsense/lib:${LD_LIBRARY_PATH:-}" \
    && /usr/bin/python3 docker/smoke_test.py
ENTRYPOINT ["/usr/local/bin/hrm-entrypoint"]
CMD ["ros2", "launch", "launcher", "system.launch.py", "auto_start_components:=false"]
