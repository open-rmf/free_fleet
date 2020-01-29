# Client tests

To emulate a running robot and also a running free fleet server,

```bash
source ~/client_ws/devel/setup.bash

roslaunch free_fleet_client_ros1 minimal_test.launch
```

The client will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. To demonstrate this behaviour,

```bash
cd ~/client_ws/devel/bin

./test_dds_sub_state
```

The client will also be listening for requests over DDS, which will trigger action server calls for `MoveBase`. To demonstrate this behaviour

```bash
cd ~/client_ws/devel/bin

# sends out a single destination request to the robot over DDS
./test_dds_pub_destination_request <fleet_name> <robot_name> <task_id> <x> <y> <yaw>

# sends out a path request, basically a sequence of destination requests to the robot over DDS
./test_dds_pub_path_request <fleet_name> <robot_name> <task_id>

# sends out a path request with locations corresponding to real map locations
./test_dds_pub_sim_path_request <fleet_name> <robot_name> <task_id>

# sends out a mode request, to pause and resume what it is doing
./test_dds_pub_mode_request <fleet_name> <robot_name> <task_id> pause
./test_dds_pub_mode_request <fleet_name> <robot_name> <task_id> resume
```
