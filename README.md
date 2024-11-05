# Hyper-Redundant-Manipulator
HRM(Hyper-Redundant-Manipulator) motion control on ROS system

## Download the project
```
# clone repository with the submodule
git clone --recurse-submodeuls https://github.com/DaeyunJang/Hyper-Redundant-Manipulator.git
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
submodule (LSTM-force-estimation) has same package (custom_interfaces), so make COLCON_IGNORE in that package
```
cd {PATH_TO_PROJECT_DIRECTORY}/src/LSTM-force-estimation/src/custom_interfaces

touch COLCON_IGNORE
```

## colcon build
```
# please build 'custom_interfaces' first with deactivation of virtual env

deactivate

cd {PATH_TO_PROJECT_DIRECTORY}  # e.g. cd ~/Hyper-Redundant-Manipulator

colcon build --packages-select custom_interfaces

source {env_name}/bin/activate

colcon build  (not use --symlink-install option, TBD)
```

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
