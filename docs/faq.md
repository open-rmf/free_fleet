# Frequently Asked Questions

Hello there!

This document will serve as intermediate documentation, to hopefully answer any questions regarding the ongoing development, as well as implementation choices that were made.

#### Build warnings for `cyclonedds`

When building the workspaces, most likely warnings related to `cyclonedds

```bash
Starting  >>> cyclonedds                                                                            
____________________________________________________________________________________________________
Warnings   << cyclonedds:cmake /home/aaron/workspaces/ffr1_ws/logs/cyclonedds/build.cmake.000.log   
You have called ADD_LIBRARY for library ddsc without any source files. This typically indicates a problem with your CMakeLists.txt file
cd /home/aaron/workspaces/ffr1_ws/build/cyclonedds; catkin build --get-env cyclonedds | catkin env -si  /usr/bin/cmake /home/aaron/workspaces/ffr1_ws/src/cyclonedds --no-warn-unused-cli -DCMAKE_INSTALL_PREFIX=/home/aaron/workspaces/ffr1_ws/devel; cd -
....................................................................................................
____________________________________________________________________________________________________
Warnings   << cyclonedds:make /home/aaron/workspaces/ffr1_ws/logs/cyclonedds/build.make.000.log     
WARNING: An illegal reflective access operation has occurred
WARNING: Illegal reflective access by com.google.inject.internal.cglib.core.$ReflectUtils$1 (file:/usr/share/maven/lib/guice.jar) to method java.lang.ClassLoader.defineClass(java.lang.String,byte[],int,int,java.security.ProtectionDomain)
WARNING: Please consider reporting this to the maintainers of com.google.inject.internal.cglib.core.$ReflectUtils$1
WARNING: Use --illegal-access=warn to enable warnings of further illegal reflective access operations
WARNING: All illegal access operations will be denied in a future release
cd /home/aaron/workspaces/ffr1_ws/build/cyclonedds; catkin build --get-env cyclonedds | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
....................................................................................................
Finished  <<< cyclonedds                              [ 16.8 seconds ]                            
```

```bash
--- stderr: cyclonedds                                                   
You have called ADD_LIBRARY for library ddsc without any source files. This typically indicates a problem with your CMakeLists.txt file
WARNING: An illegal reflective access operation has occurred
WARNING: Illegal reflective access by com.google.inject.internal.cglib.core.$ReflectUtils$1 (file:/usr/share/maven/lib/guice.jar) to method java.lang.ClassLoader.defineClass(java.lang.String,byte[],int,int,java.security.ProtectionDomain)
WARNING: Please consider reporting this to the maintainers of com.google.inject.internal.cglib.core.$ReflectUtils$1
WARNING: Use --illegal-access=warn to enable warnings of further illegal reflective access operations
WARNING: All illegal access operations will be denied in a future release
---
Finished <<< cyclonedds [47.2s]
```

These are related to the uses of build tools like `catkin` or `colcon`, which expects packages to be in a certain format, however they do not have any adverse effect, and can be ignored.

#### Build warnings for ROS1 packages when building in ROS2 workspace

This is due to the ROS1 packages in the same repository, and for the convenience of building, it is preferable to put the entire repository in the workspace instead of moving, copying or symlinking separate packages.

```bash
Starting >>> free_fleet_client_ros1
[58.269s] WARNING:colcon.colcon_cmake.task.cmake.build:Could not run installation step for package 'free_fleet_client_ros1' because it has no 'install' target
--- stderr: free_fleet_client_ros1                                          
catkin not found so skipping this ROS1 package
CMake Warning:
  Manually-specified variables were not used by the project:

    CATKIN_INSTALL_INTO_PREFIX_ROOT


---
Finished <<< free_fleet_client_ros1 [1.63s]
Starting >>> ff_examples_ros1
[59.959s] WARNING:colcon.colcon_cmake.task.cmake.build:Could not run installation step for package 'ff_examples_ros1' because it has no 'install' target
--- stderr: ff_examples_ros1                                        
catkin not found so skipping this ROS1 package
CMake Warning:
  Manually-specified variables were not used by the project:

    CATKIN_INSTALL_INTO_PREFIX_ROOT


---
Finished <<< ff_examples_ros1 [1.69s]
```

These do not have any adverse effect to the workspace, and can be ignored.

#### How is the `RobotState` derived?

The current example ROS1 client that uses the navigation stack, derives the time of the state from the transform of the robot, the battery percentages and robot modes are derived the battery states over `sensor_msgs/BatteryState` and robot motion.

Level names are a work in progress, and can currently be set with a simple `std_msgs/String`. At the moment, this field is not used by any core component or decision making yet.

#### How should the workspaces be built when they contain both ROS1 and ROS2 packages?

The repository can be placed in workspaces as per normal, while the build tool will choose to skip or build a package depending on the availability of `catkin` (ROS1) and `ament_cmake` (ROS2). Credits to [romainreignier](https://github.com/romainreignier/share_ros1_ros2_lib_demo) for the trick.

`catkin-tools` will skip packages with `COLCON_IGNORE`, while `colcon-mixin` will require specifying `mixin` files everytime a build is required. The current solution has thus far been the cleanest. In the near future, the repository will be split for ROS1 and ROS2 using branches.
