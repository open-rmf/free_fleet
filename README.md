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

Listen to transforms,

```bash
source PATH_TO_WORKSPACE/install/setup.bash
ros2 run free_fleet_exmples test_tf.py \
    --namespace robot_ns
```

Start a `navigate_to_pose` action over `zenoh`,

```bash
source PATH_TO_WORKSPACE/install/setup.bash
ros2 run free_fleet_examples test_navigate_to_pose.py \
    --frame-id map \
    --namespace robot_ns \
    --x X_COORDINATES \
    --y Y_COORDDIATES

# Example values
# x: 1.808
# y: 0.503
```