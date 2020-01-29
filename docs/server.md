# Free Fleet Server notes


# ROS 2 building instructions

Start a new ROS 2 workspace, clone the `free_fleet`, `cyclonedds`, `rmf_core` repository,

```bash
mkdir -p ~/server_ws/src
cd ~/server_ws/src

git clone https://github.com/osrf/free_fleet.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
git clone https://github.com/osrf/rmf_core.git
```

Source ROS 2 and build with the cmake flag that minimizes the build of `CycloneDDS`

```bash
cd ~/server_ws/src

source /opt/ros/eloquent/setup.bash
colcon build --cmake-args -DBUILD_IDLC=NO --packages-up-to free_fleet_server_ros2
```

# Testing

Move onto testing the free fleet server [here](turtlebot3_sim_example.md)
