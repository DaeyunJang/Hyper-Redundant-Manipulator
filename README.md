# Hyper-Redundant-Manipulator
HRM(Hyper-Redundant-Manipulator) motion control on ROS system

## python environments (virtual env with ROS2)
```
# pip module for create virtual environment
sudo apt install python3.xx-venv  # e.g. python3.10-venv (humble)

# make virtual env with --symlink
python3 -m venv {env_name} --system-site-packages --symlinks

source {env_name}/bin/activate

# requirements
pip install -r requirements.txt
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
