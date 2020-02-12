# Turtlebot3 Simulation Example

This example shows an example of running a fleet of Turtlebot3s as a free fleet.

---

## ROS 1 Setup

Start a new ROS 1 workspace, with all the necessary Turtlebot3 packages,

```bash
mkdir -p ~/client_ws/src
cd ~/client_ws

# set up the ROS 1 workspace free_fleet
wget https://raw.githubusercontent.com/osrf/free_fleet/master/free_fleet_ros1.repos
vcs import src < free_fleet_ros1.repos

# pull in the turtlebot3 packages required for the simulation
cd ~/client_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
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

---

## ROS 2 Setup

Start a new ROS 2 workspace, with all the necessary packages,

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

## Single Turtlebot3 world simulation

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

Now the fun begins! There are 3 types of commands/requests that can be sent to the simulated robot through `free_fleet`,

Destination requests, which allows single destination commands for the robots,

```bash
ros2 run free_fleet_test_ros2 send_destination_request -f turtlebot3 -r turtlebot3_1 -x 1.725 -y -0.39 --yaw 0.0 -l B1 -i <unique_task_id> -t destination_requests
```

Path requests, which requests that the robot perform a string of destination commands,

```bash
ros2 run free_fleet_test_ros2 send_path_request -f turtlebot3 -r turtlebot3_1 -i <unique_task_id> -p '[{"x": 1.725, "y": -0.39, "yaw": 0.0, "level_name": "B1"}, {"x": 1.737, "y": 0.951, "yaw": 1.57, "level_name": "B1"}, {"x": -0.616, "y": 1.852, "yaw": 3.14, "level_name": "B1"}, {"x": -0.626, "y": -1.972, "yaw": 4.71, "level_name": "B1"}]'
```

Mode requests which only supports `pause` and `resume` at the moment,

```bash
ros2 run free_fleet_test_ros2 send_mode_request -f turtlebot3 -r turtlebot3_1 -m pause -i <unique_task_id>
ros2 run free_fleet_test_ros2 send_mode_request -f turtlebot3 -r turtlebot3_1 -m resume -i <unique_task_id>
```

Note that the task IDs need to be unique, if a request is sent using a previously used task ID, the request will be ignored by the free fleet clients.

---

## Multiple Turtlebot3 world simulation
