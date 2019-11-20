This repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

# Overview
Welcome to `free_fleet`, an open-source robot fleet management system.
Sometimes it is called the "Fun Free Fleet For Friends" (F5).

# Requirements

ROS 1 Melodic and ROS 2 Dashing need to be installed, since the ROS 1 client
uses DDS implementations packaged as part of the ROS 2 project.

The build scripts assume that ROS 2 Dashing is installed to the canonical
location in `/opt/ros/dashing`

# Building the ROS1 Client

We clone the repository,

```
cd
git clone https://github.com/osrf/free_fleet.git
```

Assuming we use a regular catkin workspace, we create symlinks to the various ROS 1 packages,

```
mkdir -p ~/client_ws/src
cd ~/client_ws/src
ln -s ~/free_fleet/clients/ros1/free_fleet_client free_fleet_client
```

We can then proceed to build the project, 

```
cd ~/client_ws
source /opt/ros/melodic/setup.bash
catkin build
```

To check that everything is good to go, we can run an example,

```
source devel/setup.bash
rosrun free_fleet_client test_client
```
