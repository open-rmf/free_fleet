ARG ROS_DISTRO=noetic
FROM docker.io/ros:$ROS_DISTRO-ros-base

RUN apt update && apt install -y ros-$ROS_DISTRO-turtlebot3-gazebo ros-$ROS_DISTRO-gazebo-ros-pkgs

ENV TURTLEBOT3_MODEL=burger

RUN rm -rf \
  /var/lib/apt/lists \
  /dist

ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && roslaunch turtlebot3_gazebo turtlebot3_world.launch gui:=false"]
