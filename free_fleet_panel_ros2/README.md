# Free Fleet Panel - ROS 2

## Setup

Create a ROS 2 workspace and clone in the required packages,

```bash
mkdir -p ~/free_fleet_ros2_ws/src
cd ~/free_fleet_ros2_ws/src

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
cd ~/free_fleet_ros2_ws

# build CycloneDDS first
colcon build --packages-select cyclonedds --cmake-args -DBUILD_IDLC=NO

# build the panel
colcon build --packages-up-to free_fleet_panel_ros2 free_fleet_server_ros2
```

### Build Client

For building the client, just follow the steps shown [here](./../docs/clients.md).

## Running the examples

### Server-side

Start the visualization and the server,

```bash
source ~/free_fleet_ros2_ws/install/setup.bash

ros2 launch free_fleet_panel_ros2 turtlebot3_world_viz.launch.py
```

In another terminal, start the life-cycle of the map server,

```bash
source ~/free_fleet_ros2_ws/install/setup.bash

ros2 run nav2_util lifecycle_bringup map_server
```

At this point the map should have appeared on `rviz2`.

### Client-side

Start the multi-turtlebot3 example from the ROS 1 workspace

```bash
source ~/free_fleet_ros1_ws/devel/setup.bash

roslaunch free_fleet_client_ros1 multi_turtlebot3_ff.launch
```

### Operation

Insert `turtlebot3` in the text input `Fleet: Name:` of the panel in `rviz2`, and click the `Refresh` button to start responding to the fleet with the corresponding name.

3 boxes representing the robots in the simulation should appear in `rviz2` then.
