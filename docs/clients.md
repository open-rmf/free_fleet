# Free Fleet Client notes

* the time of the state will be tied to the transform, if no transform is found no state will be published over DDS

* robot location is derived from transforms

* battery percentages is derived from `sensor_msgs/BatteryState`

* robot mode is derived from a combination of battery states and robot motion

* level name is currently derived from a simple `std_msgs/String`

# ROS 1 building instructions

Start a new ROS 1 workspace, clone the `free_fleet` and `cyclonedds` repository,

```bash
mkdir -p ~/client_ws/src
cd ~/client_ws/src

git clone https://github.com/osrf/free_fleet.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds
```

Source ROS 1 and build with the cmake flag that minimizes the build of `CycloneDDS`

```bash
cd ~/client_ws

source /opt/ros/melodic/setup.bash
catkin build --cmake-args -DBUILD_IDLC=NO
```

# Testing

Move onto testing the free fleet client [here](clients_test.md)
