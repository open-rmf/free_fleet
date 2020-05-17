![](https://github.com/osrf/free_fleet/workflows/build/badge.svg)

# Free Fleet

## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Message Generation](#message-generation)
  - [Client in ROS1](#client-in-ros1)
  - [Server in ROS2](#server-in-ros2)
- **[Examples](#examples)**
  - [Barebones Example](#barebones-example)
  - [Turtlebot3 Simulation](#turtlebot3-simulation)
  - [Multi Turtlebot3 Simulation](#multi-turtlebot3-simulation)
  - [Commands and Requests](#commands-and-requests)
- **[FAQ](#faq)**
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

* [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04/)
* [ROS1 - Melodic](https://wiki.ros.org/melodic)
* [ROS2 - Eloquent](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/)

Install all non-ROS prerequisite packages,

```bash
sudo apt update && sudo apt install \
  git wget \
  python-rosdep \
  python-catkin-tools \
  python3-vcstool \
  python3-colcon-common-extensions \
  maven default-jdk   # CycloneDDS dependencies
```

</br>

### Message Generation

Message generation via `FleetMessages.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./free_fleet/src/messages/FleetMessages.idl).

```bash
./dds_idlc -allstructs FleetMessages.idl
```

</br>

### Client in ROS1

Start a new ROS1 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/client_ws/src
cd ~/client_ws

# set up the ROS1 workspace free_fleet
wget https://raw.githubusercontent.com/osrf/free_fleet/master/ros1.repos
vcs import src < ros1.repos
```

Install all the dependencies through `rosdep`,

```bash
cd ~/client_ws
source /opt/ros/melodic/setup.bash
rosdep install --from-paths src --ignore-src -y -r \
  --skip-keys="rmf_fleet_msgs ament_lint_common rclpy rclcpp rosidl_default_generators ament_cmake builtin_interfaces"
```

Source ROS1 and build,

```bash
cd ~/client_ws
source /opt/ros/melodic/setup.bash
catkin build
```

</br>

### Server in ROS2

Start a new ROS2 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/server_ws/src
cd ~/server_ws

# set up the ROS2 workspace for free_fleet
wget https://raw.githubusercontent.com/osrf/free_fleet/master/ros2.repos
vcs import src < ros2.repos
```

Install all the dependencies through `rosdep`,

```bash
cd ~/server_ws
source /opt/ros/eloquent/setup.bash
rosdep install --from-paths src --ignore-src -y -r \
  --skip-keys="actionlib tf roscpp rviz catkin map_server turtlebot3_navigation turtlebot3_bringup turtlebot3_gazebo move_base amcl"
```

Source ROS2 and build with the provided mixin file, which skips the ROS1 packages,

```bash
cd ~/server_ws
source /opt/ros/eloquent/setup.bash
colcon build
```

</br>
</br>

## Examples

### Barebones Example

This example emulates a running robot and also a running free fleet server,

```bash
source ~/client_ws/install/setup.bash
roslaunch ff_examples_ros1 fake_client.launch
```

The client will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. Start the server using

```bash
source ~/server_ws/install/setup.bash
ros2 launch ff_examples_ros2 fake_server.launch.xml
```

To verify that the fake client has been registered, there will be print-outs on the server terminal, otherwise, the ROS2 messages over the `/fleet_states` topic can also be used to verify,

```bash
source ~/server_ws/install/setup.bash
ros2 topic echo /fleet_states
```

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Turtlebot3 Simulation

Before starting these examples, remember to install all the prerequisites according to the [official tutorials](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-1-packages) of using `Turtlebot3`.

Launch the basic simulation of a single Turtlebot3, with a free fleet client attached to it, by sourcing the client workspace and launching the provided example launch file,

```bash
source ~/client_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger; roslaunch ff_examples_ros1 turtlebot3_world_ff.launch
```

This launch file starts the simulation in `gazebo`, visualization in `rviz`, as well as the simulated navigation stack of the single turtlebot3. Once the simulation and visualization show up, the robot can be commanded as per normal through `rviz` with `2D Nav Goal`.

Now, we start the free fleet server, with the provided launch file,

```bash
source ~/server_ws/install/setup.bash
ros2 launch ff_examples_ros2 turtlebot3_world_ff.launch.xml
```

At this point, the server should have registered the client running on that single simulated robot and a print-out should appear in the server terminal. Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

</br>

### Multi Turtlebot3 Simulation

This example launches three Turtlebot3s in simulation, and registers each robot as a client within a fleet controlled by `free_fleet`. The setup of this simulation is the same as the [single turtlebot3 simulation example](#turtlebot3-simulation).

Similarly to the example above, launch the provided launch file, which will start the simulation, visualization, navigation stacks of 3 turtlebot3s, and free fleet clients for each of the robots,

```bash
source ~/client_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger; roslaunch ff_examples_ros1 multi_turtlebot3_ff.launch
```

Once the simulation and visualization show up, the robots can then be commanded to navigate to different parts of the map by using the tool and panels in the visualization. Fill in the fleet name and robot name, select the navigation goal using `2D Nav Goal`, which will be reflected on the panel, and select `Send Nav Goal`.

![](media/multi_tb3.gif)

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

## FAQ

Answers to frequently asked questions can be found [here](/docs/faq.md).

</br>
</br>

## Plans

* Basic Qt UI for sending the requests listed [here](#commands-and-requests)

* Server User Interface - in order to monitor and control the entire fleet of robots without running scripts all the time.

* Refactor more client and server functionalities into `free_fleet`, rather than having them in `free_fleet_client_ros1` and `free_fleet_server_ros2`

* Correctly identify and work with level names

* Documentation!

* Integration demonstrations with [`rmf_core`](https://github.com/osrf/rmf_core)
