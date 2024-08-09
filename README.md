# Introduction

Supports and tested with ROS 2 Humble onwards.

# Setup

Easiest to set up with `zenoh-plugin-ros2dds` release `0.11.0` with the standalone binaries.

# Simulation

Launch simulation

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py
```

Start `zenoh-bridge-ros2dds`

```bash
./zenoh-bridge-ros2dds -c PATH_TO_WORKSPACE/src/free_fleet/free_fleet_examples/configs/example.json5
```
