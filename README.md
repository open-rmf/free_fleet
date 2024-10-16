# Free Fleet

- **[Introduction](#introduction)**
- **[Source build and setup](#source-build-and-setup)**
- **[Simulation examples](#simulation-examples)**
  - [Single turtlebot3 world](#single-turtlebot3-world)
  - [Multiple turtlebot3 world](#multiple-turtlebot3-world)
- **[Troubleshooting](#troubleshooting)**
- **[TODOs](#todos)**

## Introduction

Free fleet is a python implementation of the Open-RMF Fleet Adapter, based on the [`fleet_adapter_template`](https://github.com/open-rmf/fleet_adapter_template). It uses `zenoh` as a communication layer between each robot and the fleet adapter, allowing access and control over the navigation stacks of the robots.

Using `zenoh` bridges to pipe the necessary ROS 2 messages between each robot and the `free_fleet_adapter`, users have the flexibility to configure and customize their network setups accordingly following the [official guide](https://github.com/eclipse-zenoh/zenoh?tab=readme-ov-file#configuration-options). Examples provided in this repository are using [these configurations](./free_fleet_examples/zenoh_configs/), do take note of the selective topics that are required for the `free_fleet_adapter` to work. The `zenoh` configuration conveniently allows users to filter and limit the rate of messages based on topics as well, which will be helpful in deployments with limited network bandwidth.

![](../media/architecture.jpg)

Each robot's navigation stack is expected to be non-namespaced, while its `zenoh` bridge is expected to be set up with it's robot name as the namespace. This allows the `free_fleet_adapter` to integrate with each robot in the fleet individually using `zenoh` namespaces.

Supports
* [Ubuntu 24.04](https://ubuntu.com/blog/ubuntu-desktop-24-04-noble-numbat-deep-dive)
* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [rmw-cyclonedds-cpp](https://github.com/ros2/rmw_cyclonedds)
* [Open-RMF on main](https://github.com/open-rmf/rmf)
* [zenoh-bridge-ros2dds v1.0.0-beta.4](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/tag/1.0.0-beta.4)

We recommend setting up `zenoh-bridge-ros2dds` with the standalone binaries. After downloading the appropriate released version and platform, extract and use the standalone binaries as is. For source builds of `zenoh-bridge-ros2dds`, please follow the [official guides](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds).

Most of the tests have been performed using `rmw-cyclonedds-cpp`, while other RMW implementations have shown varying results. Support and testing with other RMW implementations will be set up down the road.

> [!WARNING]
> This has so far only been tested in simulation, and will undergo updates and changes as more testing is performed. Use at your own risk. For the legacy implementation, check out the [`legacy`](https://github.com/open-rmf/free_fleet/tree/legacy) branch.

## Source build and setup

Other system depenendencies,

```bash
sudo apt install python3-pip ros-jazzy-rmw-cyclonedds-cpp
```

The dependencies `eclipse-zenoh` and `pycdr2` are available through `pip`. Users can choose to set up a virtual environment, or `--break-system-packages` by performing the installation directly.

```bash
pip3 install pip install eclipse-zenoh==1.0.0b4 pycdr2 --break-system-packages
```

> [!NOTE]
> If an Open-RMF workspace has already been set up, users can choose to only set up an overlay workspace, which reduces build time. The following steps will assume a fresh new workspace is required.

Set up workspace, install dependencies and build,

```bash
mkdir -p ~/ff_ws/src
wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
vcs import ~/ff_ws/src < rmf.repos

cd ~/ff_ws/src
git clone https://github.com/open-rmf/free_fleet

# Install dependencies
cd ~/ff_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Simulation examples

Examples for running a single robot or multiple robots in simulation has been up in `free_fleet_examples`, along with example configuration files for `zenoh` as well as fleet configuration files for `free_fleet_adapter`.

For ROS 2, simulations will be launched using the `nav2_bringup` package. Since the `turtlebot3_gazebo` package is not being released past jazzy, users will need to clone the package to access the gazebo models,

```
sudo apt install ros-jazzy-nav2-bringup

git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations ~/turtlebot3_simulations
```

### Single turtlebot3 world

![](../media/ff_tb3_faster_smaller.gif)

This simulates running an isolated (by `ROS_DOMAIN_ID`) turtlebot3 with a ROS 2 navigation stack, and setting up RMF with `free_fleet_adapter` (on a different `ROS_DOMAIN_ID`), allowing the fleet adapter to command the robot via a configured `zenoh-bridge-ros2dds` with the namespace `turtlebot3_1`.

Launch simulation and set up the initial position of the robot (see gif),

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulations/turtlebot3_gazebo/models

# Launch the simulation
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=0

# Or launch headless
# ros2 launch nav2_bringup tb3_simulation_launch.py
```

Start `zenoh-bridge-ros2dds` with the appropriate zenoh configuration,

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd PATH_TO_EXTRACTED_ZENOH_BRIDGE
./zenoh-bridge-ros2dds -c ~/ff_ws/src/free_fleet/free_fleet_examples/zenoh_configs/turtlebot3_1_zenoh_config.json5
```

Listen to transforms over `zenoh`,

```bash
source ~/ff_ws/install/setup.bash
ros2 run free_fleet_examples test_tf.py \
    --namespace turtlebot3_1
```

Start a `navigate_to_pose` action over `zenoh`, using example values,

```bash
source ~/ff_ws/install/setup.bash
ros2 run free_fleet_examples test_navigate_to_pose.py \
    --frame-id map \
    --namespace turtlebot3_1 \
    -x 1.808 \
    -y 0.503
```

Start the RMF core packages on a different `ROS_DOMAIN_ID` to simulate running on a different machine,

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml
```

Launch the `free_fleet_adapter` with the current example's configurations, verify that `turtlebot3_1` has been added to fleet `turtletbot`.

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples tb3_simulation_fleet_adapter.launch.xml
```

Dispatch an example RMF patrol tasks using [`rmf-web`](https://github.com/open-rmf/rmf-web) on the same `ROS_DOMAIN_ID` as the RMF core packages, or use the `dispatch_patrol` script,

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 run rmf_demos_tasks dispatch_patrol \
  -p north_west north_east south_east south_west \
  -n 2 \
  -st 0
```

### Multiple turtlebot3 world

> [!NOTE]
> This multi-robot simulation example is only for testing purposes, as it is a different setup than `free_fleet` is intended to be used. The simulation spawns 2 already namespaced robots, while the `free_fleet` architecture expects individual non-namespaced robots to be partnered with a namespaced `zenoh-bridge-ros2dds`.

![](../media/multirobot_sim_architecture.jpg)

In this example, there will only be one non-namespaced zenoh bridge for both robots, which will produce the same zenoh message outputs as 2 individual namespaced zenoh bridge with non-namespaced robots. This allows the `free_fleet_adapter` to work with both robots on the same simulation.

![](../media/ff_unique_faster_smaller.gif)

Launch simulation, start the robots, and set up the initial positions (see gif),

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulations/turtlebot3_gazebo/models

ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py
```

Start `zenoh-bridge-ros2dds` with the appropriate zenoh configuration,

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd PATH_TO_EXTRACTED_ZENOH_BRIDGE
./zenoh-bridge-ros2dds -c ~/ff_ws/src/free_fleet/free_fleet_examples/zenoh_configs/unique_multi_tb3_zenoh_config.json5
```

Start the RMF core packages on a different `ROS_DOMAIN_ID` to simulate running on a different machine,

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml
```

Launch the `free_fleet_adapter` with the current example's configurations, verify that `turtlebot3_1` has been added to fleet `turtletbot`.

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples unique_multi_tb3_simulation_fleet_adapter.launch.xml

# Or launch with the rmf-web API server address
# ros2 launch free_fleet_examples unique_multi_tb3_simulation_fleet_adapter.launch.xml  server_uri:="ws://localhost:8000/_internal"
```

Dispatch example RMF patrol tasks using [`rmf-web`](https://github.com/open-rmf/rmf-web) on the same `ROS_DOMAIN_ID` as the RMF core packages, or use the `dispatch_patrol` scripts, which will cause the robot to negotiate as they perform their tasks.

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

# robot1 to run clockwise around the map
ros2 run rmf_demos_tasks dispatch_patrol \
  -p north_west north_east south_east south_west \
  -n 3 \
  -st 0 \
  -F turtlebot3 \
  -R robot1

# robot2 to run anti-clockwise around the map
ros2 run rmf_demos_tasks dispatch_patrol \
  -p south_west south_east north_east north_west \
  -n 3 \
  -st 0 \
  -F turtlebot3 \
  -R robot2
```

## Troubleshooting

* Looking for the legacy implementation of `free_fleet`? Check out the [`legacy`](https://github.com/open-rmf/free_fleet/tree/legacy) branch.

* `free_fleet_adapter` can't seem to control the robots? Check if the zenoh messages are going through using the testing scripts in `free_fleet_examples`. For ROS 2 navigation stacks, make sure that the `zenoh-bridge-ros2dds` is launched with the same `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` as the robot's navigation stack, otherwise no messages will be passed through the bridge.

* Failing to start `free_fleet_adapter` due to missing API in `rmf_fleet_adapter_python`? This may be due to using outdated `rmf_fleet_adapter_python` released binaries, either perform a `sudo apt update && sudo apt upgrade`, or build RMF from source following the [official guide](https://github.com/open-rmf/rmf).

* Simulations don't seem to work properly anymore? Try `ros2 deamon stop`, `ros2 daemon start`, or explicitly kill the `ros` and `gazebo` processes, or restart your machine. It's been noticed that if the ROS 2 or gazebo process are not terminated properly (happens rarely), the network traffic between the simulation robots and the fleet adapter get affected.

* Why is RMF not run with `use_sim_time:=true` in the examples? This is because it is on a different `ROS_DOMAIN_ID` than the simulation, therefore it will not have access to the simulation `clock` topic, the examples running RMF, `free_fleet_adapter` and the tasks will not be using sim time.

* For potential bandwidth issues, especially during multirobot sim example, spinning up a dedicated zenoh router and routing the `zenoh-bridge-ros2dds` manually to it, could help alleviate such issues.

## TODOs

* hardware testing
* attempt to optimize tf messages (not all are needed)
* ROS 1 nav support
* map switching support
* static testing
* end-to-end testing
* support for Rolling
* docker images
* releases
* testing and support for other RMW implementations
