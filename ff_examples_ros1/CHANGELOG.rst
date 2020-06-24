^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ff_examples_ros1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-11)
------------------
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
* Contributors: Aaron Chong
