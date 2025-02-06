ARG ROS_DISTRO=jazzy
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG FREE_FLEET_BRANCH=main

RUN apt update && apt install -y curl ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# fetch sources
RUN mkdir -p /ff && cd /ff \
  && curl -sL https://github.com/open-rmf/free_fleet/archive/refs/heads/$FREE_FLEET_BRANCH.tar.gz -o free_fleet.tar.gz \
  && mkdir -p /ff/src/free_fleet && tar zxf free_fleet.tar.gz -C /ff/src/free_fleet --strip-components=1 && rm free_fleet.tar.gz

RUN rosdep update && rosdep install --from-paths /ff/src -yi

RUN cd /ff \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --merge-install --install-base /opt/ff --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf /ff

# cleanup
RUN rm -rf \
  /var/lib/apt/lists \
  /dist

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["bash", "-c", ". /opt/ff/setup.bash && bash"]
