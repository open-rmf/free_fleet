![](https://github.com/osrf/free_fleet/workflows/build/badge.svg)

# Free Fleet

## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Building from source](#building-from-source)
  - [Message Generation](#message-generation)
- **[Examples](#examples)**
  - [Client](#client)
  - [Server](#server)
  - [Turtlebot3 Simulation](#turtlebot3-simulation)
  - [Commands and Requests](#commands-and-requests)
- **[FAQ](#faq)**
- **[Plans](#plans)**

</br>

## About

Welcome to `free_fleet`, an open-source robot fleet management system. 
Sometimes it is called the "Fun Free Fleet For Friends" (F5).

**Note**, this repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04/)
* [ROS2 - Eloquent](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/)

Install all non-ROS prerequisite packages,

```bash
sudo apt update && sudo apt install \
  git wget \
  python-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions
```

</br>

### Building from source

Start a new ROS2 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/ff_ws/src
cd ~/ff_ws

# set up the ROS2 workspace for free_fleet
wget https://raw.githubusercontent.com/osrf/free_fleet/eloquent-devel/free_fleet.repos
vcs import src < free_fleet.repos
```

Install all the dependencies through `rosdep`,

```bash
cd ~/ff_ws
source /opt/ros/eloquent/setup.bash
rosdep install --from-paths src --ignore-src -y -r
```

Source ROS2 and build,

```bash
cd ~/ff_ws
source /opt/ros/eloquent/setup.bash
colcon build

# You can also skip other RMF packages by building the bare minimum
# colcon build --packages-up-to free_fleet_server free_fleet_examples
```

</br>

### Message Generation

Message generation via `FleetMessages.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./free_fleet/src/messages/FleetMessages.idl). In order to regenerate new messages, a source build of `CycloneDDS` is required.

```bash
./dds_idlc -allstructs FleetMessages.idl
```

</br>

## Examples

### Client

Implementation of a ROS 2 client will be available soon. If however users prefer to implement a custom free fleet client to work with a non-ROS API, they can refer to the free fleet client API in the `free_fleet` package.

</br>

### Server

This example launches a basic free fleet server,

```bash
source ~/ff_ws/install/setup.bash
ros2 launch free_fleet_examples fake_server.launch.xml
```

To verify that the fake client has been registered, there will be print-outs on the server terminal, otherwise, the ROS 2 messages over the `/fleet_states` topic can also be used to verify,

```bash
source ~/ff_ws/install/setup.bash
ros2 topic echo /fleet_states
```

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Turtlebot3 Simulation

This server launch will work hand-in-hand with a fleet of ROS 1 turtlebot3 robots in simulation, launched using `melodic-devel`.

Now, we start the free fleet server, with the provided launch file,

```bash
source ~/server_ws/install/setup.bash
ros2 launch free_fleet_examples turtlebot3_world_ff.launch.xml
```

At this point, the server should have registered the clients running on the simulated robots and print-outs should appear in the server terminal. Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Commands and Requests

Now the fun begins! There are 3 types of commands/requests that can be sent to the simulated robots through `free_fleet`,

Destination requests, which allows single destination commands for the robots,

```bash
ros2 run free_fleet_examples send_destination_request.py -f FLEET_NAME -r ROBOT_NAME -x 1.725 -y -0.39 --yaw 0.0 -i UNIQUE_TASK_ID
```

Path requests, which requests that the robot perform a string of destination commands,

```bash
ros2 run free_fleet_examples send_path_request.py -f FLEET_NAME -r ROBOT_NAME -i UNIQUE_TASK_ID -p '[{"x": 1.725, "y": -0.39, "yaw": 0.0, "level_name": "B1"}, {"x": 1.737, "y": 0.951, "yaw": 1.57, "level_name": "B1"}, {"x": -0.616, "y": 1.852, "yaw": 3.14, "level_name": "B1"}, {"x": -0.626, "y": -1.972, "yaw": 4.71, "level_name": "B1"}]'
```

Mode requests which only supports `pause` and `resume` at the moment,

```bash
ros2 run free_fleet_examples send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m pause -i UNIQUE_TASK_ID
ros2 run free_fleet_examples send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m resume -i UNIQUE_TASK_ID
```

**Note** that the task IDs need to be unique, if a request is sent using a previously used task ID, the request will be ignored by the free fleet clients.

</br>

## FAQ

Answers to frequently asked questions can be found [here](/docs/faq.md).

</br>
