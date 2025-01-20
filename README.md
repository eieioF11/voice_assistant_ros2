# voice_assistant_ros2
ros2 Voice Assistant node pkgs

# Dependence
- [ros2_common_tools](https://github.com/eieioF11/ros2_common_tools)

## Installation

### Install Python pkgs
```bash
pip3 install -r requirements.txt
```

### Set OpenAI API
```bash
export OPENAI_API_KEY="your key"
```

### Installation with apt
Go to the following site to add a repository
※Installation by rosdep is also possible by adding a repository.

https://eieiof11.github.io/ppa/


```bash
sudo apt install ros-$ROS_DISTRO-data-logger ros-$ROS_DISTRO-extension-node ros-$ROS_DISTRO-common-utils

cd ros2_ws/src
cd ../
colcon build --symlink-install
```

### Build Manually
```bash
cd ros2_ws/src
git clone --recursive https://github.com/eieioF11/ros2_common_tools.git
cd ../
colcon build --symlink-install
```

## launch
```bash
ros2 launch voice_assistant voice_assistant.launch.py
```
