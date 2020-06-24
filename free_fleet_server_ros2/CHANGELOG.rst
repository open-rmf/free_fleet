^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package free_fleet_server_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-11)
------------------
* Fix/mode adapter error (`#41 <https://github.com/osrf/free_fleet/issues/41>`_)
  * Quick and dirty fix to handle request error, which results in MODE_ADAPTER_ERROR from RMF
  * Cleaned up sending robot states printing, added handling of MODE_REQUEST_ERROR
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
* removed testing script, will require a new launch file on dp2 side
* basic cleanup
* mono readme structure, with content links, renamed launch files
* multi turtlebot free fleet sim works, time to refactor into its own test package
* pause not responding, probably not merged in yet
* Merge pull request `#30 <https://github.com/osrf/free_fleet/issues/30>`_ from osrf/bug/missing-rmf-to-fleet-transform
  added missing transformation to fleet frame
* added missing transformation to fleet frame
* Merge pull request `#29 <https://github.com/osrf/free_fleet/issues/29>`_ from osrf/bug/launch-node-name
  Bug/launch node name
* basic xml launch for test_server
* change build test branch name reference, changed launch param name
* added suffix for node name
* Merge pull request `#28 <https://github.com/osrf/free_fleet/issues/28>`_ from osrf/refactor
  Refactor
* removed printout buffering, added turtlebot3 testing
* changed the parameter file node name
* added small launch script for ros2 testing, server node not intiailizing parameters
* seems to be fully refactored, not tested yet
* basic interfaces and structure of free_fleet_server_ros2 done
* Contributors: Aaron, Aaron Chong
