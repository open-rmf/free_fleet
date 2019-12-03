# Free Fleet Client notes

* the time of the state will be tied to the transform, if no transform is found no state will be published over DDS

* robot location is derived from transforms

* battery percentages is derived from `sensor_msgs/BatteryState`

* robot mode is derived from a combination of battery states and robot motion

* level name is currently derived from a simple `std_msgs/String`

# ROS 1 building instructions

Clone the repository somewhere,

```
cd
git clone https://github.com/osrf/free_fleet
```

Start a new ROS 1 workspace while symbolically linking the client in,

```
mkdir -p ~/client_ws/src
cd ~/client_ws/src
ln -s ~/free_fleet/clients/ros1 free_fleet_ros1
```

Source ROS 1 and build!

```
cd ~/client_ws
source /opt/ros/melodic/setup.bash
catkin build
```

At this point the build will fail due to the project failing to find the necessary headers in the CycloneDDS `ExternalProject`, this can currently be solved by simply invoking build again.

```
catkin build
```

# Client tests

To emulate a running robot and also a running free fleet server,

```
# Terminal A
roscore

# Terminal B, at this point it will be asking for the move base action server, which will timeout after 10 seconds
rosrun free_fleet_client free_fleet_client

# Terminal C, the fake move base action server, the client will then be listening for transform frames
rosrun free_fleet_client test_action_server

# Terminal D, the fake static transform
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 base_footprint map 200
```

The client will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. To demonstrate this behaviour,

```
rosrun free_fleet_client test_dds_sub_state
```

The client will also be listening for commands over DDS, which will trigger action server calls for `MoveBase`. To demonstrate this behaviour

```
# sends out a single location command to the robot over DDS
rosrun free_fleet_client test_dds_pub_location_command

# sends out a path command, basically a sequence of location commands to the robot over DDS
rosrun free_fleet_client test_dds_pub_path_command

# sends out a path command with locations corresponding to real map locations
rosrun free_fleet_client test_dds_pub_sim_location_command

# sends out a mode command, to pause and resume what it is doing
rosrun free_fleet_client test_dds_pub_mode_command pause
rosrun free_fleet_client test_dds_pub_mode_command ressume
```

# Client simulation test

To simulate working with a live ROS 1 navigation stack, we will use the Turtlebot3 navigation examples given [here](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

Follow the examples and proceed to launch the simulation, mapping, saving the map, and launching the navigation stack with the saved map. For convenience we have also included sample maps, generated using `gmapping` just like the examples [here](../clients/ros1/free_fleet_client/test_maps).

In a separate terminal, source the workspace where `free_fleet_client` and `free_fleet_msgs` was built, and launch the client with fleet name, robot name and robot model as mandatory arguments,

```bash
source ~/client_ws/devel/setup.bash
rosrun free_fleet_client free_fleet_client -f FLEET_NAME -r ROBOT_NAME -m ROBOT_MODEL
```

Use the flag `-h` for more information about the default values of all optional arguments and how to adjust them to your needs.

Similar to the section above, the built test executables can now be run to control and monitor the robot. We recommend using the `map.yaml` map, which corresponds to `turtlebot3_world` in the ROBOTIS examples.

```
# location command
rosrun free_fleet_client test_dds_pub_location_command

# path command
rosrun free_fleet_client test_dds_sim_path_command
```

While the simulation is running, the robot's state can be observed by calling the DDS subscribing executable,

```
rosrun free_fleet_client test_dds_sub_state
```
