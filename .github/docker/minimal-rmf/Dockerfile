ARG ROS_DISTRO=jazzy
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG RMF_BRANCH=main
ARG FREE_FLEET_BRANCH=main

RUN apt update && apt install -y curl ros-$ROS_DISTRO-rmw-cyclonedds-cpp

RUN mkdir -p /rmf && cd /rmf \
    && curl -sL https://github.com/open-rmf/free_fleet/archive/refs/heads/$FREE_FLEET_BRANCH.tar.gz -o free_fleet.tar.gz \
    && mkdir -p /rmf/src/free_fleet && tar zxf free_fleet.tar.gz -C /rmf/src/free_fleet --strip-components=1 && rm free_fleet.tar.gz \
    && curl -sL https://github.com/open-rmf/rmf_demos/archive/refs/heads/$RMF_BRANCH.tar.gz -o rmf_demos.tar.gz \
    && mkdir -p /rmf/src/rmf_demos && tar zxf rmf_demos.tar.gz -C /rmf/src/rmf_demos --strip-components=1 && rm rmf_demos.tar.gz

RUN rosdep update && rosdep install --from-paths /rmf/src -yi

RUN cd /rmf \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --merge-install --install-base /opt/rmf --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to free_fleet_examples \
  && rm -rf /rmf

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN rm -rf \
  /var/lib/apt/lists \
  /dist

ENTRYPOINT ["bash", "-c", ". /opt/rmf/setup.bash && ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml"]
