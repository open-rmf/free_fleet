# Free Fleet

![Nightly](https://github.com/open-rmf/free_fleet/actions/workflows/nightly.yaml/badge.svg)![Unit tests](https://github.com/open-rmf/free_fleet/actions/workflows/unit-tests.yaml/badge.svg)![Nav2 Integration tests](https://github.com/open-rmf/free_fleet/actions/workflows/nav2-integration-tests.yaml/badge.svg)![Nav1 Integration tests](https://github.com/open-rmf/free_fleet/actions/workflows/nav1-integration-tests.yaml/badge.svg)[![codecov](https://codecov.io/github/open-rmf/free_fleet/graph/badge.svg?token=JCOB9g3YTn)](https://codecov.io/github/open-rmf/free_fleet)

- **[Introduction](#introduction)**
- **[Dependency installation, source build and setup](#dependency-installation-source-build-and-setup)**
- **[Simulation examples](#simulation-examples)**
  - [Nav2 Single turtlebot3 world](#nav2-single-turtlebot3-world)
  - [Nav2 Multiple turtlebot3 world](#nav2-multiple-turtlebot3-world)
  - [Nav1 Single turtlebot3 world](#nav1-single-turtlebot3-world)
- **[Troubleshooting](#troubleshooting)**
- **[TODOs](#todos)**

## Introduction

Free fleet is a python implementation of the Open-RMF Fleet Adapter, based on the [`fleet_adapter_template`](https://github.com/open-rmf/fleet_adapter_template). It uses `zenoh` as a communication layer between each robot and the fleet adapter, allowing access and control over the navigation stacks of the robots.

Using `zenoh` bridges to pipe the necessary ROS 2 / 1 messages between each robot and the `free_fleet_adapter`, users have the flexibility to configure and customize their network setups accordingly following the [official guide](https://github.com/eclipse-zenoh/zenoh?tab=readme-ov-file#configuration-options). Examples provided in this repository are using [these configurations](./free_fleet_examples/config/zenoh/), do take note of the selective topics that are required for the `free_fleet_adapter` to work. The `zenoh` configuration conveniently allows users to filter and limit the rate of messages based on topics as well, which will be helpful in deployments with limited network bandwidth.

![](../media/architecture.jpg)

Each robot's navigation stack is expected to be non-namespaced, while its `zenoh` bridge is expected to be set up with it's robot name as the namespace. This allows the `free_fleet_adapter` to integrate with each robot in the fleet individually using `zenoh` namespaces.

Supports
* [Ubuntu 24.04](https://ubuntu.com/blog/ubuntu-desktop-24-04-noble-numbat-deep-dive)
* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [rmw-cyclonedds-cpp](https://github.com/ros2/rmw_cyclonedds)
* [Open-RMF binaries on ROS 2 Jazzy](https://github.com/open-rmf/rmf)
* [zenoh-bridge-ros2dds v1.1.0](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/tag/1.1.0)
* [zenoh-bridge-ros1 main](https://github.com/eclipse-zenoh/zenoh-plugin-ros1)
* [zenoh router](https://zenoh.io/docs/getting-started/installation/#ubuntu-or-any-debian)

We recommend setting up `zenoh-bridge-ros2dds` with the released standalone binaries. After downloading the appropriate released version and platform, extract and use the standalone binaries as is. For source builds of `zenoh-bridge-ros2dds`, please follow the [official guides](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds).

As for `zenoh-bridge-ros1`, it requires the new [support for bridge namespaces](https://github.com/eclipse-zenoh/zenoh-plugin-ros1/pull/225), and therefore needs to be built from source. Once the feature has been released, this will be updated.

Most of the tests have been performed using `rmw-cyclonedds-cpp`, while other RMW implementations have shown varying results. Support and testing with other RMW implementations will be set up down the road.

## Dependency installation, source build and setup

System dependencies,

```bash
sudo apt update && sudo apt install python3-pip ros-jazzy-rmw-cyclonedds-cpp
```

The dependencies `eclipse-zenoh`, `pycdr2`, `rosbags` are available through `pip`. Users can choose to set up a virtual environment, or `--break-system-packages` by performing the installation directly.

```bash
pip3 install pip install eclipse-zenoh==1.1.0 pycdr2 rosbags --break-system-packages
```

Install `zenohd` from the [official guide](https://zenoh.io/docs/getting-started/installation/#ubuntu-or-any-debian).

Set up workspace, install dependencies and build,

```bash
mkdir -p ~/ff_ws/src
cd ~/ff_ws/src
git clone https://github.com/open-rmf/free_fleet

# Install dependencies
cd ~/ff_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Download and extract standalone binaries for `zenoh-bridge-ros2dds` (optionally `zenohd` if a non-latest version is desired) with the correct architecture, system setup and version. The following example instructions are for `x86_64-unknown-linux-gnu`,

```bash
# Change preferred zenoh version here
export ZENOH_VERSION=1.1.0

# Download and extract zenoh-bridge-ros2dds release
wget -O zenoh-plugin-ros2dds.zip https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/$ZENOH_VERSION/zenoh-plugin-ros2dds-$ZENOH_VERSION-x86_64-unknown-linux-gnu-standalone.zip
unzip zenoh-plugin-ros2dds.zip

# If using released standalone binaries of zenoh router, download and extract the release
# wget -O zenoh.zip https://github.com/eclipse-zenoh/zenoh/releases/download/$ZENOH_VERSION/zenoh-$ZENOH_VERSION-x86_64-unknown-linux-gnu-standalone.zip
# unzip zenoh.zip
```

## Simulation examples

Examples for running a single robot or multiple robots in simulation has been up in `free_fleet_examples`, along with example configuration files for `zenoh` as well as fleet configuration files for `free_fleet_adapter`.

For ROS 2, simulations will be launched using the `nav2_bringup` package. Since the `turtlebot3_gazebo` package is not being released past jazzy, users will need to clone the package to access the gazebo models,

```
sudo apt update && sudo apt install ros-jazzy-nav2-bringup

git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations ~/turtlebot3_simulations
```

### Nav2 Single turtlebot3 world

![](../media/ff_tb3_faster_smaller.gif)

This simulates running an isolated (by `ROS_DOMAIN_ID`) turtlebot3 with a ROS 2 navigation stack, and setting up RMF with `free_fleet_adapter` (on a different `ROS_DOMAIN_ID`), allowing the fleet adapter to command the robot via a configured `zenoh-bridge-ros2dds` with the namespace `nav2_tb3`.

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

Start `zenoh` router,

```bash
zenohd

# If using released standalaone binaries
# cd PATH_TO_EXTRACTED_ZENOH_ROUTER
# ./zenohd
```

Start `zenoh-bridge-ros2dds` with the appropriate zenoh client configuration,

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd PATH_TO_EXTRACTED_ZENOH_BRIDGE
./zenoh-bridge-ros2dds -c ~/ff_ws/src/free_fleet/free_fleet_examples/config/zenoh/nav2_tb3_zenoh_bridge_ros2dds_client_config.json5
```

Listen to transforms over `zenoh`,

```bash
source ~/ff_ws/install/setup.bash
ros2 run free_fleet_examples nav2_get_tf.py \
    --namespace nav2_tb3
```

Start a `navigate_to_pose` action over `zenoh`, using example values,

```bash
source ~/ff_ws/install/setup.bash
ros2 run free_fleet_examples nav2_send_navigate_to_pose.py \
    --frame-id map \
    --namespace nav2_tb3 \
    -x 1.808 \
    -y 0.503
```

Start the RMF core packages on a different `ROS_DOMAIN_ID` to simulate running on a different machine,

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml
```

Launch the `free_fleet_adapter` with the current example's configurations, verify that `nav2_tb3` has been added to fleet `turtletbot3`.

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples nav2_tb3_simulation_fleet_adapter.launch.xml

# Or launch with the rmf-web API server address
# ros2 launch free_fleet_examples nav2_tb3_simulation_fleet_adapter.launch.xml  server_uri:="ws://localhost:8000/_internal"
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

### Nav2 Multiple turtlebot3 world

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

Start `zenoh` router,

```bash
zenohd

# If using released standalaone binaries
# cd PATH_TO_EXTRACTED_ZENOH_ROUTER
# ./zenohd
```

Start `zenoh-bridge-ros2dds` with the appropriate zenoh client configuration,

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd PATH_TO_EXTRACTED_ZENOH_BRIDGE
./zenoh-bridge-ros2dds -c ~/ff_ws/src/free_fleet/free_fleet_examples/config/zenoh/nav2_unique_multi_tb3_zenoh_bridge_ros2dds_client_config.json5
```

Start the RMF core packages on a different `ROS_DOMAIN_ID` to simulate running on a different machine,

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml
```

Launch the `free_fleet_adapter` with the current example's configurations, verify that `nav2_tb3` has been added to fleet `turtlebot3`.

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples nav2_unique_multi_tb3_simulation_fleet_adapter.launch.xml

# Or launch with the rmf-web API server address
# ros2 launch free_fleet_examples nav2_unique_multi_tb3_simulation_fleet_adapter.launch.xml  server_uri:="ws://localhost:8000/_internal"
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

### Nav1 Single turtlebot3 world

![](../media/nav1_sim_architecture.jpg)

> [!WARNING]
> The Nav1 integration has only been tested in simulation and in ROS 1 Noetic, and currently requires a source build of [zenoh-plugin-ros1](https://github.com/eclipse-zenoh/zenoh-plugin-ros1), to [support bridge namespacing](https://github.com/eclipse-zenoh/zenoh-plugin-ros1/pull/225). This will be updated after the feature has been added to a release.

Check out the [docker compose integration tests](.github/docker/integration-tests/nav1-docker-compose.yaml) for an overview of how the integration can be set up.

On the machine where the free fleet adapter will run, start a `zenoh` router,

```bash
zenohd

# If using released standalaone binaries
# cd PATH_TO_EXTRACTED_ZENOH_ROUTER
# ./zenohd
```

In the ROS 1 Noetic environment, set up all the prerequisites as mentioned in the [official guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation), and start the turtlebot3 simulation. See the [relevant docker file](.github/docker/minimal-ros1-sim/Dockerfile) for reference.

```bash
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

In the ROS 1 Noetic environment, bringup the Nav1 stack with the prepared map in [navigation2](https://github.com/ros-navigation/navigation2/tree/main/nav2_bringup/maps). See the [relevant docker file](.github/docker/minimal-nav1-bringup/Dockerfile) for reference.

```bash
# prepare the map
git clone https://github.com/ros-navigation/navigation2

source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/PATH_TO_navigation2/nav2_bringup/maps/tb3_sandbox.yaml
```

In the ROS 1 Noetic environment, set up prerequisites of [zenoh-plugin-ros1](https://github.com/eclipse-zenoh/zenoh-plugin-ros1), build `zenoh-bridge-ros1` in release, and start it with the [provided config in examples](free_fleet_examples/config/zenoh/nav1_tb3_zenoh_bridge_ros1_client_config.json5). See the [relevant docker file](.github/docker/minimal-zenoh-bridge-ros1/Dockerfile) for reference.

```bash
# Get the config file
git clone https://github.com/open-rmf/free_fleet

# Build the bridge
git clone --recursive https://github.com/eclipse-zenoh/zenoh-plugin-ros1
cd zenoh-plugin-ros1
cargo build --package zenoh-bridge-ros1 --bin zenoh-bridge-ros1 --release

# Use cargo run, or just run the executable directly
source /opt/ros/noetic/setup.bash
./target/release/zenoh-bridge-ros1 -c PATH_TO_free_fleet/free_fleet_examples/config/zenoh/nav1_tb3_zenoh_bridge_ros1_client_config.json5
```

On the machine where the free fleet adapter will run, start the common launch files and the free fleet adapter,

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml
```

Launch the `free_fleet_adapter` with the current example's configurations, verify that `nav1_tb3` has been added to fleet `turtlebot3`.

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch free_fleet_examples nav1_tb3_simulation_fleet_adapter.launch.xml

# Or launch with the rmf-web API server address
# ros2 launch free_fleet_examples nav1_tb3_simulation_fleet_adapter.launch.xml  server_uri:="ws://localhost:8000/_internal"
```

Dispatch example RMF patrol tasks using [`rmf-web`](https://github.com/open-rmf/rmf-web) on the same `ROS_DOMAIN_ID` as the RMF core packages, or use the `dispatch_patrol` scripts, which will cause the robot to negotiate as they perform their tasks.

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

# nav1_tb3 to run clockwise around the map
ros2 run rmf_demos_tasks dispatch_patrol \
  -p north_west north_east south_east south_west \
  -n 3 \
  -st 0 \
  -F turtlebot3 \
  -R nav1_tb3
```

## Troubleshooting

* Looking for the legacy implementation of `free_fleet`? Check out the tag [1.3.0](https://github.com/open-rmf/free_fleet/releases/tag/1.3.0), or the [`legacy`](https://github.com/open-rmf/free_fleet/tree/legacy) branch.

* `free_fleet_adapter` can't seem to control the robots? Check if the zenoh messages are going through using the testing scripts in `free_fleet_examples`. For ROS 2 navigation stacks, make sure that the `zenoh-bridge-ros2dds` is launched with the same `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` as the robot's navigation stack, otherwise no messages will be passed through the bridge.

* Failing to start `free_fleet_adapter` due to missing API in `rmf_fleet_adapter_python`? This may be due to using outdated `rmf_fleet_adapter_python` released binaries, either perform a `sudo apt update && sudo apt upgrade`, or build RMF from source following the [official guide](https://github.com/open-rmf/rmf).

* Simulations don't seem to work properly anymore? Try `ros2 deamon stop`, `ros2 daemon start`, or explicitly kill the `ros` and `gazebo` processes, or restart your machine. It's been noticed that if the ROS 2 or gazebo process are not terminated properly (happens rarely), the network traffic between the simulation robots and the fleet adapter get affected.

* ROS 1 Navigation stack simulation does not seem to work as expected? Check out the [integration tests docker compose](.github/docker/integration-tests/nav1-docker-compose.yaml), as well as the [simulation](.github/docker/minimal-ros1-sim/Dockerfile) and [bringup](.github/docker/minimal-nav1-bringup/Dockerfile) docker files, for any missing dependencies.

* Why does RMF not run with `use_sim_time:=true` in the examples? This is because it is on a different `ROS_DOMAIN_ID` than the simulation, therefore it will not have access to the simulation `clock` topic, the examples running RMF, `free_fleet_adapter` and the tasks will not be using sim time.

* For potential bandwidth issues, especially during multirobot sim example, spinning up a dedicated zenoh router and routing the `zenoh-bridge-ros2dds` manually to it, could help alleviate such issues.

* If `zenoh` messages are not received, make sure the versions between the `eclipse-zenoh` in `pip`, `zenoh-bridge-ros2dds` and `zenohd` are all the same. If the debian binary releases of `zenohd` have breaking changes, and the repo has not yet migrate to the newer version, please open an issue ticket and we will look into migrating as soon as possible. In the meantime, using an older standalone release of `zenohd` would be a temporary workaround. Our integration tests will attempt to catch these breaking changes too.

* `zenohd` address already in use. This is most likely due to the `rest-http-port` which uses port 8000 by default, and might cause a conflict with other systems, for example `rmf-web`'s API server. Run `zenohd --rest-http-port 8001` to change it to 8001 or anything else.

* Please also check past or [existing issues on this repository](https://github.com/open-rmf/free_fleet/issues), or any discussions on the main [RMF discussion page](https://github.com/open-rmf/rmf/discussions).

## TODOs

* attempt to optimize tf messages (not all are needed)
* custom actions to be abstracted
* map switching support
* test replanning behavior
* support for Rolling
* docker images
* releases
* testing and support for other RMW implementations
