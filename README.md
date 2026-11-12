# Hyper-Redundant-Manipulator
HRM(Hyper-Redundant-Manipulator) motion control on ROS system

For GUI/CUDA camera startup and checks of the 30 Hz estimation pipeline, see the
[real-time operation guide](docs/REALTIME_ESTIMATION.md).

## Docker on Ubuntu 22.04 (full system)

The [Docker guide](docker/README.md) provides an additional, isolated installation
path for the same GUI, CUDA RealSense cameras, ESP32 sensors, shape/force/contact
estimation, recording, TCP bridge and robot controller. The native installation
below is not replaced. It starts `ros2 launch launcher system.launch.py`, with
component autostart and physical motor output initially disabled, as in native
startup. Use the GUI's per-component Start/Stop controls to choose which devices
and processes to run, and Motor Enable to control physical motor output.

```bash
bash scripts/hrm_docker.sh doctor
# After installing host Docker/Compose and NVIDIA Container Toolkit:
bash scripts/hrm_docker.sh build
bash scripts/hrm_docker.sh smoke
bash scripts/hrm_docker.sh gui
```

The `smoke` command checks software without starting ROS nodes or accessing hardware.
Model bundles are supplied separately. Recorded data and editable settings are
stored in host directories, not baked into the image. Container build/hardware
validation status and remaining prerequisites are recorded in the Docker guide.

## Download the project
```
# clone repository with the submodule
git clone --recurse-submodules https://github.com/DaeyunJang/Hyper-Redundant-Manipulator.git
```

## python environments (virtual env with ROS2)
```
# pip module for create virtual environment
sudo apt install python3.xx-venv  # e.g. python3.10-venv (humble)

# make virtual env with --symlink
python3 -m venv {env_name} --system-site-packages --symlinks

# requirements
pip install -r requirements.txt
```

## Package ingoring
submodule (LSTM-force-estimation) has same package (custom_interfaces)  
Make COLCON_IGNORE in that package if COLCON_IGNORE file doesn't exist in directory './src/LSTM-force-estimation/src/custom_interfaces'
```
cd {PATH_TO_PROJECT_DIRECTORY}/src/LSTM-force-estimation/src/custom_interfaces

touch COLCON_IGNORE
```

## colcon build
```

deactivate

cd {PATH_TO_PROJECT_DIRECTORY}  # e.g. cd ~/Hyper-Redundant-Manipulator

~~colcon build --packages-select custom_interfaces~~

source {env_name}/bin/activate

colcon build --symlink-install
```
~~# please build 'custom_interfaces' first with deactivation of virtual env~~  
~~colcon build --packages-select custom_interfaces~~

> **Note** If you face the build error as  
AttributeError: 'NoneType' object has no attribute 'shutdown'  
&rightarrow **Check the version of module 'empy' is 3.3.4 on requirements.txt (not 'em')**



# Implementation
### prerequisition
```
. insatll/setup.bash
```
### Full system
```
ros2 launch launcher system.launch.py
```

### Sub system - without motor
```
ros2 launch launcher system_demo.launch.py
```
