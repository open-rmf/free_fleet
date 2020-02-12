![](https://github.com/osrf/free_fleet/workflows/build/badge.svg)

# Free Fleet

## Contents

**[About](#About)**
**[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Message Generation](#message-generation)
  - [Client in ROS 1](#client-in-ros-1)
  - [Server in ROS 2](#server-in-ros-2)
**[Examples](#examples)**
  - [Barebones Example](#barebones-example)
  - [Turtlebot3 Simulation](#turtlebot3-simulation)
  - [Multi Turtlebot3 Simulation](#multi-turtlebot3-simulation)
  - [Commands and Requests](#commands-and-requests)
**[Notes](#notes)**
  - [Client in ROS 1](#client-in-ros-1)
**[Plans](#plans)**

---

## About

Welcome to `free_fleet`, an open-source robot fleet management system. 
Sometimes it is called the "Fun Free Fleet For Friends" (F5).

**Note**, this repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

---

## Installation Instructions

### Prerequisites

ROS 1 Melodic and ROS 2 Eloquent need to be installed, since the ROS 1 client
uses DDS implementations packaged as part of the ROS 2 project.

The build scripts assume that ROS 2 Eloquent is installed to the canonical
location in `/opt/ros/eloquent`



### Message Generation

Message generation via `FreeFleet.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./free_fleet/src/messages/).

To recreate them, a full installtion of `CycloneDDS` will be needed, which will require additional prerequisites. The full instructions can be found on its [repository](https://github.com/eclipse-cyclonedds/cyclonedds).

```bash
./dds_idlc -allstructs FreeFleet.idl
```



### Client in ROS 1

Start a new ROS 1 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/client_ws/src
cd ~/client_ws

# set up the ROS 1 workspace free_fleet
wget https://raw.githubusercontent.com/osrf/free_fleet/master/free_fleet_ros1.repos
vcs import src < free_fleet_ros1.repos
```

Source ROS 1 and build with the cmake flag that minimizes the build of `CycloneDDS`,

```bash
cd ~/client_ws
source /opt/ros/melodic/setup.bash

# build cyclonedds first with the necessary cmake flags
catkin build cyclonedds --cmake-args -DBUILD_IDLC=NO

# build the rest of the packages
catkin build
```



### Server in ROS 2

Start a new ROS 2 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/server_ws/src
cd ~/server_ws

# set up the ROS 2 workspace for free_fleet
wget https://raw.githubusercontent.com/osrf/free_fleet/master/free_fleet_ros2.repos
vcs import src < free_fleet_ros2.repos
```

Source ROS 2 and build with the cmake flag that minimizes the build of `CycloneDDS`,

``` bash
cd ~/server_ws
source /opt/ros/eloquent/setup.bash

# build cyclonedds first with the necessary cmake flags
colcon build --packages-select cyclonedds --cmake-args -DBUILD_IDLC=NO

# build the rest of the packages
colcon build
```

---

## Examples

### Barebones Example

This example emulates a running robot and also a running free fleet server,

```bash
source ~/client_ws/devel/setup.bash
roslaunch free_fleet_client_ros1 fake_client.launch
```

The client will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. Start the server using

```bash
source ~/server_ws/install/setup.bash
ros2 launch free_fleet_server_ros2 fake_server.launch.xml
```

To verify that the fake client has been registered, there will be print-outs on the server terminal, otherwise, the ROS 2 messages over the `/fleet_states` topic can also be used to verify,

```bash
source ~/server_ws/install/setup.bash
ros2 topic echo /fleet_states
```

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).



### Turtlebot3 Simulation

This example launches a single Turtlebot3 in simulation, and registers it as a robot within a fleet controlled by `free_fleet`. To set up this simulation example, more ROS 1 packages will be needed, assuming we already have built the initial setup listed [here](#installation-instructions)

```bash
cd ~/client_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# source and build
cd ~/client_ws
source devel/setup.bash
catkin build
```

Launch the basic simulation of a single Turtlebot3, with a free fleet client attached to it, by sourcing the client workspace and launching the provided example launch file,

```bash
source ~/client_ws/devel/setup.bash
roslaunch free_fleet_client_ros1 turtlebot3_world_ff.launch
```

This launch file starts the simulation in `gazebo`, visualization in `rviz`, as well as the simulated navigation stack of the single turtlebot3. Once the simulation and visualization show up, the robot can be commanded as per normal through `rviz` with `2D Nav Goal`.

Now, we start the free fleet server, with the provided launch file,

```bash
source ~/server_ws/install/setup.bash
ros2 launch free_fleet_server_ros2 turtlebot3_world_ff.launch.xml
```

At this point, the server should have registered the client running on that single simulated robot and a print-out should appear in the server terminal. Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).



### Multi Turtlebot3 Simulation

This example launches three Turtlebot3s in simulation, and registers each robot as a client within a fleet controlled by `free_fleet`. The setup of this simulation is the same as the [single turtlebot3 simulation example](#turtlebot3-simulation).

Similarly to the example above, launch the provided launch file, which will start the simulation, visualization, navigation stacks of 3 turtlebot3s, and free fleet clients for each of the robots,

```bash
source ~/client_ws/devel/setup.bash
roslaunch free_fleet_client_ros1 multi_turtlebot3_ff.launch
```

Once the simulation shows up and the free fleet clients are alive, we launch the free fleet server just like the example above, 

```bash
source ~/server_ws/install/setup.bash
ros2 launch free_fleet_server_ros2 turtlebot3_world_ff.launch.xml
```

Note how we are using the exact same launch script as above, this is because all the turtlebot3s in the simulation are registered under the same fleet, identified by the `fleet_name`, which in this case is `turtlebot3`.

At this point, the server should have registered the client running on the 3 simulated robots and print-outs should appear in the server terminal. Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).



### Commands and Requests

Now the fun begins! There are 3 types of commands/requests that can be sent to the simulated robots through `free_fleet`,

Destination requests, which allows single destination commands for the robots,

```bash
ros2 run free_fleet_test_ros2 send_destination_request -f <fleet_name> -r <robot_name> -x 1.725 -y -0.39 --yaw 0.0 -l B1 -i <unique_task_id> -t destination_requests
```

Path requests, which requests that the robot perform a string of destination commands,

```bash
ros2 run free_fleet_test_ros2 send_path_request -f <fleet_name> -r <robot_name> -i <unique_task_id> -p '[{"x": 1.725, "y": -0.39, "yaw": 0.0, "level_name": "B1"}, {"x": 1.737, "y": 0.951, "yaw": 1.57, "level_name": "B1"}, {"x": -0.616, "y": 1.852, "yaw": 3.14, "level_name": "B1"}, {"x": -0.626, "y": -1.972, "yaw": 4.71, "level_name": "B1"}]'
```

Mode requests which only supports `pause` and `resume` at the moment,

```bash
ros2 run free_fleet_test_ros2 send_mode_request -f <fleet_name> -r <robot_name> -m pause -i <unique_task_id>
ros2 run free_fleet_test_ros2 send_mode_request -f <fleet_name> -r <robot_name> -m resume -i <unique_task_id>
```

**Note** that the task IDs need to be unique, if a request is sent using a previously used task ID, the request will be ignored by the free fleet clients.

---

## Notes

### Client in ROS 1

* the time of the state is tied to the transform, if no transform is found no state will be published over DDS

* battery percentages is derived from `sensor_msgs/BatteryState`

* robot mode is derived from a combination of battery states and robot motion

* level name is currently derived from a simple `std_msgs/String`, and at the moment is not used in any core components or decision making

---

## Plans

* Basic Qt UI for sending the requests listed [here](#commands-and-requests)

* Server User Interface - in order to monitor and control the entire fleet of robots without running scripts all the time.

* Refactor more client and server functionalities into `free_fleet`, rather than having them in `free_fleet_client_ros1` and `free_fleet_server_ros2`

* Correctly identify and work with level names

* Documentation!

* Integration demonstrations with `rmf_core`
