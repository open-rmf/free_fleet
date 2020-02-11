# Turtlebot3 Simulation Example

This example shows an example of running a fleet of Turtlebot3s as a free fleet.

## ROS 1 Setup

Start a new ROS 1 workspace, with all the necessary Turtlebot3 packages,

```bash
mkdir -p ~/tb3_client_ws/src
cd ~/tb3_client_ws/src

git clone https://github.com/osrf/free_fleet.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Source ROS 1 and build with the cmake flag that minimizes the build of `CycloneDDS`,

```bash
cd ~/tb3_client_ws

source /opt/ros/melodic/setup.bash
catkin build --cmake-args -DBUILD_IDLC=NO
```

## ROS 2 Setup

Follow the steps [here](server.md)

## Starting the simulation and clients

Launch the simulation with 3 turtlebot3s, and start their respective navigation stacks in different namespaces. At the same time, launch 3 clients directed at the different turtlebot3s.

```bash
source ~/tb3_client_ws/devel/setup.bash

roslaunch free_fleet_client_ros1 multi_turtlebot3_ff.launch
```

## Starting the server and monitoring the fleet

Launch the Free Fleet server with the parameters matching the turtlebot3 clients.

```bash
source ~/server_ws/install/setup.bash

ros2 launch free_fleet_server_ros2 turtlebot3_world_ff.launch.py
```

In a separate terminal list all the topics available over ROS 2, and listen in on the different robot states of the turtlebot3 fleet

```bash
source ~/server_ws/install/setup.bash

ros2 topic list

ros2 topic echo /fleet_state
```

## Mode Request

```bash
ros2 run free_fleet_test_ros2 send_mode_request -f turtlebot3 -r turtlebot3_1 -m pause -i gyifoiuds
ros2 run free_fleet_test_ros2 send_mode_request -f turtlebot3 -r turtlebot3_1 -m resume -i gyifoiudsudfd
```

## Path Reqeust

```bash
ros2 run free_fleet_test_ros2 send_path_request -f turtlebot3 -r turtlebot3_1 -i huiesio -p '[{"x": 1.725, "y": -0.39, "yaw": 0.0, "level_name": "B1"}, {"x": 1.737, "y": 0.951, "yaw": 1.57, "level_name": "B1"}, {"x": -0.616, "y": 1.852, "yaw": 3.14, "level_name": "B1"}, {"x": -0.626, "y": -1.972, "yaw": 4.71, "level_name": "B1"}]'
```

## Destination Request

```bash
ros2 run free_fleet_test_ros2 send_destination_request -f turtlebot3 -r turtlebot3_1 -x 1.725 -y -0.39 --yaw 0.0 -l B1 -i ygiuhjkdsfoijds -t destination_requests
```
