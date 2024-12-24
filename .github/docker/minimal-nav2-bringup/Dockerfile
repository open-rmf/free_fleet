ARG ROS_DISTRO=jazzy
FROM docker.io/ros:$ROS_DISTRO-ros-base

RUN apt update && apt install -y curl ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-rmw-cyclonedds-cpp

RUN mkdir -p /tb3 && cd /tb3 \
  && curl -sL https://github.com/ROBOTIS-GIT/turtlebot3_simulations/archive/refs/heads/master.tar.gz -o turtlebot3_simulations.tar.gz \
  && mkdir -p /tb3/turtlebot3_simulations && tar zxf turtlebot3_simulations.tar.gz -C /tb3/turtlebot3_simulations --strip-components=1 && rm turtlebot3_simulations.tar.gz

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/tb3/turtlebot3_simulations/turtlebot3_gazebo/models"

RUN rm -rf \
  /var/lib/apt/lists \
  /dist

# Modify existing params file to add initial pose
RUN mkdir -p /params && cd /params \
  && cp /opt/ros/$ROS_DISTRO/share/nav2_bringup/params/nav2_params.yaml . \
  && sed -z 's|amcl:\n  ros__parameters:\n|amcl:\n  ros__parameters:\n    set_initial_pose: true\n    initial_pose: {x: -2.0, y: -0.5, z: 0.0, yaw: 0.0}\n|' nav2_params.yaml > nav2_params_edited.yaml

ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/params/nav2_params_edited.yaml use_rviz:=False"]
