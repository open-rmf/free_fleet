^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package free_fleet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#28 <https://github.com/osrf/free_fleet/issues/28>`_ from osrf/refactor
  Refactor
* seems to be fully refactored, not tested yet
* basic interfaces and structure of free_fleet_server_ros2 done
* server impl done but not tested
* addition of minimal dds tests in free fleet core package
* build and basic publish test done
* all because of linker language missed out of a C and CXX
* undefined references for some of the objects, not sure why yet
* Perhaps a find_package() call is missing for an IMPORTED target, or an ALIAS target is missing?
* still trying to figure out why the link is needed
* starting internal impl
* looks like impl ptrs are needed
* ros1 client package somehow also needs to link with cyclonedds?
* testing vendor build
* trying to link ddsc
* finished some basic documentation
* forward declaration of dds message types, leave timing and publishing state to the client
* message utils should be done
* helper message structs
* before refactoring everything into rosless projects
* Contributors: Aaron, Aaron Chong
