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

* path ???, will be ignored for now

* level name will be a std_msgs/String

# Testing Client

To check that it will publish with transforms, start a static transform through command line before starting the node

```
# Terminal A
roscore

# Terminal B
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 base_footprint map 200

# Terminal C
rosrun free_fleet_client free_fleet_client
```

To check that the messages are really sent over DDS,

```
# Terminal D
rosrun free_fleet_client test_server
```
