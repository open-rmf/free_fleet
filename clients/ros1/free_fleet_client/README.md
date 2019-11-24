# generating the CycloneDDS message types
For convenience, the generation of FreeFleet.idl has been done offline. To
re-create it, use the `dds_idlc` generator that is built with CycloneDDS:
```
./dds_idlc -allstructs FreeFleet.idl
```

# Client notes

* the time of the state will be tied to the transform, if no transform is found no state will be published over DDS

* location comes from listening to transforms

* battery from listening to a std_msgs/Float32

* mode will be using free_fleet_msgs/RobotMode

* path will be using free_fleet_msgs/PathSequence

* level name will be a std_msgs/String

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
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 baseootprint map 200
```

The client will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. To demonstrate this behaviour,

```
rosrun free_fleet_client test_dds_sub_state
```

The client will also be listening for commands over DDS, which will trigger action server calls for `MoveBase`. To demonstrate this behaviour

```
rosrun free_fleet_client test_dds_pub_command
```
