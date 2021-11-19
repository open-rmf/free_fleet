![](https://github.com/open-rmf/free_fleet/workflows/build/badge.svg)

# Free Fleet

## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Message Generation](#message-generation)
  - [Client in ROS 1](#client-in-ros-1)
  - [Client and Server in ROS 2](#client-and-server-in-ros-2)
- **[Examples](#examples)**
  - [Barebones Example](#barebones-example)
  - [Turtlebot3 Fleet Server](#turtlebot3-fleet-server)
  - [ROS 1 Turtlebot3 Simulation](#ros-1-turtlebot3-simulation)
  - [ROS 2 Turtlebot3 Simulation](#ros-2-turtlebot3-simulation)
  - [ROS 1 Multi Turtlebot3 Simulation](#ros-1-multi-turtlebot3-simulation)
  - [Commands and Requests](#commands-and-requests)
- **[Plans](#plans)**

</br>
</br>

## About

Welcome to `free_fleet`, an open-source robot fleet management system. 
Sometimes it is called the "Fun Free Fleet For Friends" (F5).

**Note**, this repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

</br>
</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
* [ROS1 - Noetic](https://wiki.ros.org/noetic)
* [ROS2 - Galactic](https://docs.ros.org/en/galactic/index.html)

Install all non-ROS prerequisite packages,

```bash
sudo apt update && sudo apt install \
  git wget qt5-default \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  # maven default-jdk   # Uncomment to install dependencies for message generation
```

</br>

### Message Generation

Message generation via `FleetMessages.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./free_fleet/src/messages/FleetMessages.idl).

```bash
./dds_idlc -allstructs FleetMessages.idl
```

</br>

### Client in ROS 1

Start a new ROS 1 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/ff_ros1_ws/src
cd ~/ff_ros1_ws/src
git clone https://github.com/open-rmf/free_fleet -b main
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.7.x
```

Install all the dependencies through `rosdep`,

```bash
cd ~/ff_ros1_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -yr
```

Source ROS 1 and build,

```bash
cd ~/ff_ros1_ws
source /opt/ros/noetic/setup.bash
colcon build
```

</br>

### Client and Server in ROS 2

Start a new ROS 2 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/ff_ros2_ws/src
cd ~/ff_ros2_ws/src
git clone https://github.com/open-rmf/free_fleet -b main
git clone https://github.com/open-rmf/rmf_internal_msgs -b main
```

Install all the dependencies through `rosdep`,

```bash
cd ~/ff_ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -yr
```

Source ROS 2 and build, 

```bash
cd ~/ff_ros2_ws
source /opt/ros/galactic/setup.bash
colcon build

# Optionally use the command below to only build the relevant packages,
# colcon build --packages-up-to \
#     free_fleet ff_examples_ros2 free_fleet_server_ros2 free_fleet_client_ros2
```

</br>
</br>

## Examples

### Barebones Example

This example emulates a running ROS 1 robot,

```bash
source ~/ff_ros1_ws/install/setup.bash
roslaunch ff_examples_ros1 fake_client.launch
```

This example emulates a running ROS 2 robot,

```bash
source ~/ff_ros2_ws/install/setup.bash
ros2 launch ff_examples_ros2 fake_client.launch.xml
```

The clients will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. Start the server using

```bash
source ~/ff_ros2_ws/install/setup.bash
ros2 launch ff_examples_ros2 fake_server.launch.xml

# Verify that the server registers the fake clients
# [INFO] [1636706176.184055177] [fake_server_node]: registered a new robot: [fake_ros1_robot]
# [INFO] [1636706176.184055177] [fake_server_node]: registered a new robot: [fake_ros2_robot]
```

ROS 2 messages over the `/fleet_states` topic can also be used to verify that the clients are registered,

```bash
source ~/ff_ros2_ws/install/setup.bash
ros2 topic echo /fleet_states
```

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Turtlebot3 Fleet Server

This launches a server for a fleet of simulated turtlebot3 robots.

```bash
source ~/ff_ros2_ws/install/setup.bash
ros2 launch ff_examples_ros2 turtlebot3_world_ff_server.launch.xml
```

At this point, the server should register any clients running, if any of the simulations below are running.

Users can issue requests and commands through the server to the clients. Check out the example scripts and their uses [here](#commands-and-requests).

### ROS 1 Turtlebot3 Simulation

Before starting these examples, remember to install all the prerequisites according to the [official tutorials](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) of using `Turtlebot3`, under `noetic`.

```bash
sudo apt install ros-noetic-dwa-local-planner
```

Launch the basic simulation of a single Turtlebot3, with a free fleet client attached to it, by sourcing the ROS 1 workspace and launching the provided example launch file,

```bash
source ~/ff_ros1_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger; roslaunch ff_examples_ros1 turtlebot3_world_ff.launch
```

This launch file starts the simulation in `gazebo`, visualization in `rviz`, as well as the simulated navigation stack of the single turtlebot3. Once the simulation and visualization show up, the robot can be commanded as per normal through `rviz` with `2D Nav Goal`.

If the server is already running, it should display that a new robot has been registered.

```bash
[INFO] [1636706001.275082185] [turtlebot3_fleet_server_node]: registered a new robot: [ros1_tb3_0]
```

Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

### ROS 2 Turtlebot3 Simulation

Before starting these examples, remember to install and build all the prerequisites,

```bash
sudo apt install ros-galactic-nav2-util ros-galactic-nav2-bringup ros-galactic-rviz2

cd ~/ff_ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3 -b galactic-devel

cd ~/ff_ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -yr

source /opt/ros/galactic/setup.bash
colcon build
```

Launch the basic simulation of a single Turtlebot3, with a free fleet client attached to it, by sourcing the ROS 1 workspace and launching the provided example launch file,

```bash
source ~/ff_ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger; ros2 launch ff_examples_ros2 turtlebot3_world_ff.launch.xml
```

This launch file starts the simulation in `gazebo`, visualization in `rviz2`, as well as the simulated navigation stack of the single turtlebot3. Once the simulation and visualization show up, the robot can be commanded as per normal through `rviz2` with `2D Nav Goal`.

If the server is already running, it should display that a new robot has been registered.

```bash
[INFO] [1636706001.275082185] [turtlebot3_fleet_server_node]: registered a new robot: [ros2_tb3_0]
```

Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### ROS 1 Multi Turtlebot3 Simulation

This example launches three Turtlebot3s in simulation, and registers each robot as a client within a fleet controlled by `free_fleet`. The setup of this simulation is the same as the [ROS 1 Turtlebot3 Simulation](#ros-1-turtlebot3-simulation).

Similarly to the example, launch the provided launch file, which will start the simulation, visualization, navigation stacks of 3 turtlebot3s, and free fleet clients for each of the robots,

```bash
source ~/ff_ros1_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger; roslaunch ff_examples_ros1 multi_turtlebot3_ff.launch
```

Once the simulation and visualization show up, the robots can then be commanded to navigate to different parts of the map by using the tool and panels in the visualization. Fill in the fleet name and robot name, select the navigation goal using `2D Nav Goal`, which will be reflected on the panel, and select `Send Nav Goal`.

![](media/multi_tb3.gif)

If the server is already running, it should display that a new robot has been registered.

```bash
[INFO] [1636706001.275082185] [turtlebot3_fleet_server_node]: registered a new robot: [tb3_0]
[INFO] [1636706001.275082185] [turtlebot3_fleet_server_node]: registered a new robot: [tb3_1]
[INFO] [1636706001.275082185] [turtlebot3_fleet_server_node]: registered a new robot: [tb3_2]
```

Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send more specific requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Commands and Requests

Now the fun begins! There are 3 types of commands/requests that can be sent to the simulated robots through `free_fleet`,

Destination requests, which allows single destination commands for the robots,

```bash
ros2 run ff_examples_ros2 send_destination_request.py -f FLEET_NAME -r ROBOT_NAME -x 1.725 -y -0.39 --yaw 0.0 -i UNIQUE_TASK_ID
```

Path requests, which requests that the robot perform a string of destination commands,

```bash
ros2 run ff_examples_ros2 send_path_request.py -f FLEET_NAME -r ROBOT_NAME -i UNIQUE_TASK_ID -p '[{"x": 1.725, "y": -0.39, "yaw": 0.0, "level_name": "B1"}, {"x": 1.737, "y": 0.951, "yaw": 1.57, "level_name": "B1"}, {"x": -0.616, "y": 1.852, "yaw": 3.14, "level_name": "B1"}, {"x": -0.626, "y": -1.972, "yaw": 4.71, "level_name": "B1"}]'
```

Mode requests which only supports `pause` and `resume` at the moment,

```bash
ros2 run ff_examples_ros2 send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m pause -i UNIQUE_TASK_ID
ros2 run ff_examples_ros2 send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m resume -i UNIQUE_TASK_ID
```

**Note** that the task IDs need to be unique, if a request is sent using a previously used task ID, the request will be ignored by the free fleet clients.

</br>
</br>

## Plans

* Significant changes incoming from the `develop` branch, however it is still a work in progress.
