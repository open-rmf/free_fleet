# Free Fleet Client notes

* the time of the state will be tied to the transform, if no transform is found no state will be published over DDS

* robot location is derived from transforms

* battery percentages is derived from `sensor_msgs/BatteryState`

* robot mode is derived from a combination of battery states and robot motion

* level name is currently derived from a simple `std_msgs/String`

# ROS 1 building instructions

Clone the repository somewhere,

```bash
cd
git clone https://github.com/osrf/free_fleet
```

Start a new ROS 1 workspace while symbolically linking the client in,

```bash
mkdir -p ~/client_ws/src
cd ~/client_ws/src
ln -s ~/free_fleet/clients/ros1 free_fleet_ros1
```

Source ROS 1 and build!

```bash
cd ~/client_ws
source /opt/ros/melodic/setup.bash
catkin build
```

At this point the build will fail due to the project failing to find the necessary headers in the CycloneDDS `ExternalProject`, this can currently be solved by simply invoking build again.

```bash
catkin build
```

# Testing

Move onto testing the free fleet client [here](clients_test.md)
