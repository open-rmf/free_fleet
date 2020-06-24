^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package free_fleet_client_ros1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-11)
------------------
* Fix/mode adapter error (`#41 <https://github.com/osrf/free_fleet/issues/41>`_)
  * Quick and dirty fix to handle request error, which results in MODE_ADAPTER_ERROR from RMF
  * Cleaned up sending robot states printing, added handling of MODE_REQUEST_ERROR
* Feature/multi tb example (`#40 <https://github.com/osrf/free_fleet/issues/40>`_)
  * new idl based off current messages
  * adding new rosdep installations and colcon mixin build instructions, git workflow updated
  * adding rosdep update to git workflow
  * removed traffic_editor from .repos
  * changed ros1 build to colcon, added mixin
  * in the middle of cleaning up readme
  * corrected build deps on colcon
  * changed readme to use colcon for ROS1, added FAQ
  * changed ros1 build back to catkin, added blacklist setup, and COLCON_IGNORE for ROS1 packages, updated build test
  * added missing catkin tools dep
  * added cyclonedds deps to apt installs
  * added catkin tools
  * installing rosdep for both environments
  * rosdep installation added to build test and instructions
  * packages search for ament_cmake now to avoid building ros2 packages
  * added complete deps for packages, used trick to handle both ros distros in single repository
  * changed repos names
  * corrected readme regarding topic name for single robot demo
  * updated README for instructions and added build warning faq
  * updated ModeParameter message and conversion
  * updated .idl conversion instructions
  * added build preference to faq
  * added cleaned up rviz config for multi turtlebot3 example
  * started ros1 rviz plugin implementation, got display context
  * nav goal updating internal pose state, WIP send through dds
  * panel running, panel server issue with reading states, WIP debugging multi navigation local planning
  * fixed model arg passing to single robot launching, made rviz config nicer
  * multi turtlebot3 demo fixed
  * added gif
  * Moved finding Qt5 to within the catkin_FOUND block
  * Cleaned up unused artifacts, name chages
  * Propagated name changes
* Feature/refactor (`#39 <https://github.com/osrf/free_fleet/issues/39>`_)
  * new idl based off current messages
  * adding new rosdep installations and colcon mixin build instructions, git workflow updated
  * adding rosdep update to git workflow
  * removed traffic_editor from .repos
  * changed ros1 build to colcon, added mixin
  * in the middle of cleaning up readme
  * corrected build deps on colcon
  * changed readme to use colcon for ROS1, added FAQ
  * changed ros1 build back to catkin, added blacklist setup, and COLCON_IGNORE for ROS1 packages, updated build test
  * added missing catkin tools dep
  * added cyclonedds deps to apt installs
  * added catkin tools
  * installing rosdep for both environments
  * rosdep installation added to build test and instructions
  * packages search for ament_cmake now to avoid building ros2 packages
  * added complete deps for packages, used trick to handle both ros distros in single repository
  * changed repos names
  * corrected readme regarding topic name for single robot demo
  * updated README for instructions and added build warning faq
  * updated ModeParameter message and conversion
  * updated .idl conversion instructions
  * added build preference to faq
* Merge pull request `#33 <https://github.com/osrf/free_fleet/issues/33>`_ from osrf/feature/request-scripts
  Feature/request scripts
* refactored examples and tests into their own packages
* cleaned up readme
* mono readme structure, with content links, renamed launch files
* multi turtlebot free fleet sim works, time to refactor into its own test package
* Merge branch 'master' into feature/request-scripts
* pause not responding, probably not merged in yet
* Merge pull request `#32 <https://github.com/osrf/free_fleet/issues/32>`_ from osrf/bug/missing-stop-go-handling
  added missing implementation of pausing robot
* added missing implementation of pausing robot
* Merge pull request `#28 <https://github.com/osrf/free_fleet/issues/28>`_ from osrf/refactor
  Refactor
* finished multi turtlebot3 world test
* having trouble matching the map frames to individual robots
* removed printout buffering, added turtlebot3 testing
* tb3 world testing nav launch works
* adding test maps and launch file for testing in turtle simulation
* cleaned up constructors
* added small launch script for ros2 testing, server node not intiailizing parameters
* basic interfaces and structure of free_fleet_server_ros2 done
* addition of minimal dds tests in free fleet core package
* client node should be fully refactored
* build and basic publish test done
* continue refactor
* all because of linker language missed out of a C and CXX
* undefined references for some of the objects, not sure why yet
* still trying to figure out why the link is needed
* looks like impl ptrs are needed
* ros1 client package somehow also needs to link with cyclonedds?
* testing vendor build
* trying to link ddsc
* Contributors: Aaron, Aaron Chong
