# HRM Docker runtime: Ubuntu 22.04, ROS 2 Humble, NVIDIA GPU

This is an **additional sensor-only execution path**, not a replacement for the
existing native installation. It packages the operator GUI, current image/angle
pipeline, force/contact model runtime, recorder, AprilTag pipeline, and dry-run FK.
The Windows motor server is not containerized. Neither `tcp_pkg` nor the legacy
DAQ/MasterMACS/LSTM workspace is built into this image.

## Validation status (2026-11-11)

Docker support has been added, but **an actual image build and container hardware
test have not yet been completed on this PC**: Docker is absent, installing it
requires interactive administrator authentication, and the workspace filesystem
has approximately 12 GB free. No host packages, GPU drivers, camera processes, or
motors have been changed by this work. Static checks and local software tests are
not substitutes for building the image and testing the camera pipeline below.

Completed without hardware access:

- 95 GUI/process/port-selector regression tests and 10 Docker-helper tests passed.
- Compose base and generated device/group configurations passed the official
  standalone Compose validator; shell syntax and Python parsing passed.
- Python dependency resolution passed a dry run (no host installation).
- Synthetic CPU inference, offscreen Qt and cv_bridge checks passed in the existing
  host environments, not in the unbuilt image.
- The existing trusted `cable_loadcell_angle` Masked/KalmanNet/Transformer bundle
  loaded and inferred successfully on CPU with a synthetic 30-sample, 26-feature
  window. This is a loading check, not a live prediction-accuracy measurement.

Reserve **at least 25 GiB** in Docker's storage location for the initial image,
compiler stages and build cache. This is a conservative preparation estimate, not
a measured image size or upper bound. Do not delete recordings, `.git/objects`,
models or the working native installation to make room automatically.

## 1. Host prerequisites

Supported target: an Ubuntu 22.04 **amd64 Linux desktop**, an NVIDIA GPU with a
driver compatible with CUDA 12.4, Docker Engine, Docker Compose V2, and NVIDIA
Container Toolkit. Host ROS, a host Python venv and a host CUDA compiler are not
required by the container. Camera kernel/USB access and permissions are still
host responsibilities; Docker does not virtualize the camera or kernel driver.

Install Docker Engine and its Compose plugin using the official Ubuntu procedure:

- https://docs.docker.com/engine/install/ubuntu/
- https://docs.docker.com/engine/install/linux-postinstall/

Install/configure NVIDIA Container Toolkit using its official procedure:

- https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

After installing the toolkit, its documented Docker configuration commands are:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

These commands modify the host Docker configuration and restart its daemon. Run
them yourself when it is safe to interrupt any existing containers. Do not send
administrator passwords to the assistant. Docker group membership grants powerful
host access; follow your lab's policy. Run the HRM helper as your normal desktop
user after Docker access works, not with `sudo`, so recordings retain your UID.

Check the installation:

```bash
docker version
docker compose version
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.4.1-runtime-ubuntu22.04 nvidia-smi
```

Use a local Xorg session, or an XWayland session with working `DISPLAY` and
`XAUTHORITY`. Install `xauth` if missing. The helper copies only the active display's
cookie into a private, Git-ignored file; it does **not** run `xhost +` or expose the
Docker socket. A display cookie still authorizes access to your desktop: run only
trusted images. Pure headless/SSH-without-display operation is not this profile's
initial target.

For cameras, install the RealSense USB udev rules on the **host**, following the
librealsense installation instructions for the selected SDK. For ESP32, grant your
normal user access to the appropriate `dialout`/device group, then log in again.
Do not recursively chmod `/dev` or grant global writable permissions to all devices.

## 2. Clone, inspect and build

The container does not need the legacy LSTM submodule or `ros_controls.humble.repos`.
The latter is not imported: this runtime uses the repository's own controller, not
the unrelated ros2_control examples and simulators in that list.

```bash
git clone https://github.com/DaeyunJang/Hyper-Redundant-Manipulator.git
cd Hyper-Redundant-Manipulator
bash scripts/hrm_docker.sh doctor
bash scripts/hrm_docker.sh build
```

`doctor` is read-only. It checks executables, Docker accessibility, free space,
device names and model availability; it does not open devices or start ROS nodes.
The helper never installs packages, invokes sudo, cleans Docker caches, or deletes
your files. `build` downloads dependencies and builds software, not robot motion.

The image builds CUDA-enabled librealsense **v2.55.1** and realsense-ros **4.55.1**.
The default camera backend is native V4L2, matching the current native setup.
The default CUDA architecture is **86 (RTX 3060)**; set the architecture for another
GPU before building. This is not a universal prebuilt binary for every NVIDIA GPU.

```bash
HRM_CUDA_ARCH=86 HRM_BUILD_JOBS=2 bash scripts/hrm_docker.sh build
```

If your host kernel cannot support the native backend, an explicit alternative is:

```bash
HRM_RSUSB_BACKEND=ON bash scripts/hrm_docker.sh build
```

RSUSB changes the camera backend. It must be validated separately, particularly
with two cameras; do not assume identical bandwidth, alignment performance or 30 Hz.
No host kernel patch is installed by the image. Newer GPU architectures may also
require a newer CUDA toolchain, which is outside this initial pinned configuration.

Python packages are installed **inside the image**, using one system Python 3.10
for ROS and inference. The old broad root `requirements.txt` and host `HRM_env`
are not used. NumPy/SciPy versions in `requirements-runtime.txt` preserve compatibility
with the currently imported reconstruction APIs. Torch inference is CPU-only by
default, matching the GUI's current force-inference setting; CUDA accelerates
RealSense alignment. This does not include the retired TensorFlow estimator.

## 3. Models and persistent data

Put trusted `hrm_bundle.pt` files in `src/estimation_pkg/models`, preserving the
existing feature/model directory structure. They are not available in a fresh
clone and are never copied into the Docker build context. Do not load untrusted
PyTorch checkpoints.

| Host path (default) | Container path | Access |
|---|---|---|
| `src/estimation_pkg/models/` | `/opt/hrm_ws/src/estimation_pkg/models/` | Read-only |
| `record/` | `/opt/hrm_ws/record/` | Read/write |
| `.docker-local/home/` | `/home/hrm/` | Read/write |
| `.docker-local/home/config/` | Installed estimation JSONs resolve here | Read/write |

The first container start copies missing configuration defaults into the persisted
config directory. Existing values are never overwritten on restart. ROI changes
are saved atomically inside this directory; a single-file bind mount would not
support the GUI's rename-based save operation. Rebuilding code does not reset your
saved configuration. Compare it with the image defaults when deliberately updating
hardware/calibration parameters. Original native source configuration is untouched.

Use `HRM_MODELS_DIR` or `HRM_RECORD_DIR` to select other host directories:

```bash
HRM_MODELS_DIR=/path/to/models HRM_RECORD_DIR=/path/to/record \
  bash scripts/hrm_docker.sh gui
```

## 4. Software smoke check and GUI

First test imports, Qt offscreen rendering, messages and synthetic model inference
without opening cameras or starting ROS nodes:

```bash
bash scripts/hrm_docker.sh smoke
# Optional: load the actual trusted bundle; zero-valued input is only a software test.
bash scripts/hrm_docker.sh smoke --model \
  /opt/hrm_ws/src/estimation_pkg/models/cable_loadcell_angle/models/masked__force_kalmannet__location_transformer/hrm_bundle.pt
```

Close native camera/serial nodes and RealSense Viewer before container hardware
use. Do not run two owners of the same device. Connect the cameras and ESP32 before
starting the GUI:

```bash
bash scripts/hrm_docker.sh gui
```

The GUI starts with **all automatic component launching disabled**. Select the
actual camera model/serial and USB port, then start only the components you need:

1. HRM camera; inspect RGB and aligned depth.
2. Segment estimation after lighting/ROI/base placement are correct.
3. Serial sensors; inspect four load cells and F/T values.
4. Optional robot control for FK with physical output disabled.
5. Optional tag camera and AprilTag when a second camera is connected.
6. Optional force/contact and recorder, subject to the missing-input rules below.

`HRM_SENSOR_ONLY=1` is enforced by the entrypoint. TCP components are hidden/rejected,
Motor Enable is disabled, direct GUI motor publication is blocked, and GUI controller
launches force `motor_output_enabled:=false`. This is workflow protection, **not a
security boundary against arbitrary manual ROS commands**. Keep the physical motor
server disconnected for this validation. There is no supported motor-enabled
Docker profile in this change.

Without TCP, `/motor_state` normally does not exist. Models requiring motor/cable
inputs may correctly report invalid/not-ready rather than infer. Normal recording
also requires motor/cable inputs, so it may refuse to start or produce a complete
summary. No fake values, relaxed required-data checks, or automatic bag replay are
introduced. Use the smoke test to test model loading separately from live validity.

The helper enumerates current `/dev/video*`, `/dev/media*`, `/dev/ttyUSB*`, and
`/dev/ttyACM*` nodes and adds their numeric host groups. USB bus nodes are shared
for RealSense control. It does not expose the entire host `/dev` or use `privileged`.
After reconnecting video/serial hardware, stop and start the container to refresh
the mappings. Within the GUI select the mapped ttyUSB/ttyACM path (host `/dev/serial`
aliases are not separately mounted).

## 5. Diagnostics and shutdown

```bash
bash scripts/hrm_docker.sh config  # Show effective Compose/device mapping; no startup.
bash scripts/hrm_docker.sh logs
bash scripts/hrm_docker.sh stop
bash scripts/hrm_docker.sh shell
```

Use a second terminal for diagnostics inside the running container:

```bash
docker compose exec hrm /usr/local/bin/hrm-entrypoint bash
ros2 node list
ros2 run image_rate_probe pipeline_rate_probe
```

Networking and IPC use the host namespaces so Fast DDS shared-memory communication
is consistent. The default ROS domain is **87**, with `ROS_LOCALHOST_ONLY=1`, to
separate this validation from the normal robot graph. A ROS domain is not a security
boundary; do not reuse it for an active motor system. Host-to-container diagnostics
must use the same domain. All intra-container nodes share the existing image-sized
Fast DDS configuration. Do not forward host-only `PYTHONPATH`, ROS prefixes, or DDS
profile filenames into the container.

After hardware startup, measure RGB/aligned-depth rates, processed-angle rate,
source age, sensor rates and inference timing. Verify no `tcpclient` node and that
`ros2 param get /robot_control motor_output_enabled` reports false if FK is running.
Docker does not guarantee deterministic real-time scheduling or native performance.
Record actual measured values; do not reuse old native timing numbers as Docker results.

## Build scope and provenance

`.dockerignore` is an allowlist, not just a copy of `.gitignore`. Recordings, `.git`,
analysis, host environments, local vendor builds, and multi-GB model directories are
excluded before any context is sent to Docker. Runtime source is copied into the
image; editing host source requires a rebuild. Builds do not overwrite native
`build/`, `install/`, `.cuda_realsense/`, or source files.

Base image tags and key Python/SDK versions are specified, but apt dependencies are
resolved from the upstream repositories at build time. This is not a fully
bit-reproducible offline archive. Preserve the resulting image digest and the runtime
version report for paper reproducibility.

References:

- [Docker GPU reservations](https://docs.docker.com/compose/how-tos/gpu-support/)
- [Docker device access](https://docs.docker.com/reference/cli/docker/container/run/#using-dynamically-created-devices---device-cgroup-rule)
- [Fast DDS shared memory in Docker](https://fast-rtps.docs.eprosima.com/en/stable/docker/shm_docker.html)
- [NVIDIA driver capabilities](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html)
