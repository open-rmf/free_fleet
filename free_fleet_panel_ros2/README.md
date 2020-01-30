# Free Fleet Panel - ROS 2

## Setup

Create a ROS 2 workspace and clone in the required packages,

```bash
mkdir -p ~/free_fleet_ws/src
cd ~/free_fleet_ws/src

git clone https://github.com/osrf/free_fleet.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
git clone https://github.com/osrf/rmf_core.git
git clone https://github.com/ros-planning/navigation2.git
git clone https://github.com/ros2/rcl_interfaces.git
git clone https://github.com/ros2/test_interface_files.git
```

Source ROS 2 `eloquent`, build `CycloneDDS` with a specific flag, before building the rest of the required packages,

```bash
source /opt/ros/eloquent/setup.bash
cd ~/free_fleet_ws

# build CycloneDDS first
colcon build --packages-select cyclonedds --cmake-args -DBUILD_IDLC=NO

# build the panel
colcon build --packages-up-to free_fleet_panel_ros2
```

For building the client or server, just follow the steps shown [here](./../docs/clients.md) or [here](./../docs/server.md).
