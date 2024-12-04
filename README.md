# Hyper-Redundant-Manipulator
HRM(Hyper-Redundant-Manipulator) motion control on ROS system

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
