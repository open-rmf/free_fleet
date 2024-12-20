ARG ROS_DISTRO=noetic
FROM docker.io/ros:$ROS_DISTRO-ros-base

RUN apt update && apt install -y curl ros-$ROS_DISTRO-turtlebot3-navigation ros-$ROS_DISTRO-dwa-local-planner

RUN mkdir -p /tb3 && cd /tb3 \
  && curl -sL https://github.com/ros-navigation/navigation2/archive/refs/heads/master.tar.gz -o navigation2.tar.gz \
  && mkdir -p /tb3/navigation2 && tar zxf navigation2.tar.gz -C /tb3/navigation2 --strip-components=1 && rm navigation2.tar.gz

ENV TURTLEBOT3_MODEL burger

RUN rm -rf \
  /var/lib/apt/lists \
  /dist

ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/tb3/navigation2/nav2_bringup/maps/tb3_sandbox.yaml open_rviz:=false initial_pose_x:=-2.0 initial_pose_y:=-0.5 initial_pose_a:=0.0"]
