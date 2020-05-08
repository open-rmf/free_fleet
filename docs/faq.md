# Frequently Asked Questions

Hello there!

This document will serve as intermediate documentation, to hopefully answer any questions regarding the ongoing development, as well as implementation choices that were made.

#### How is the `RobotState` derived?

The current example ROS1 client that uses the navigation stack, derives the time of the state from the transform of the robot, the battery percentages and robot modes are derived the battery states over `sensor_msgs/BatteryState` and robot motion.

Level names are a work in progress, and can currently be set with a simple `std_msgs/String`. At the moment, this field is not used by any core component or decision making yet.
