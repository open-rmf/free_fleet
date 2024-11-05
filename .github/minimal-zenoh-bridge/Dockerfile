ARG ROS_DISTRO=jazzy
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG ZENOH_VERSION=1.0.0
ARG BRANCH=efc/integration-testing

RUN apt update && apt install -y curl wget unzip ros-jazzy-rmw-cyclonedds-cpp

RUN mkdir -p /zenoh-bridge && cd /zenoh-bridge \
  && wget https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/$ZENOH_VERSION/zenoh-plugin-ros2dds-$ZENOH_VERSION-x86_64-unknown-linux-gnu-standalone.zip \
  && unzip zenoh-plugin-ros2dds-$ZENOH_VERSION-x86_64-unknown-linux-gnu-standalone.zip \
  && rm zenoh-plugin-ros2dds-$ZENOH_VERSION-x86_64-unknown-linux-gnu-standalone.zip 

RUN cd /zenoh-bridge && mkdir -p /zenoh-bridge/free_fleet \
  && curl -L https://github.com/open-rmf/free_fleet/archive/$BRANCH.tar.gz -o free_fleet.tar.gz \
  && tar zxf free_fleet.tar.gz -C /zenoh-bridge/free_fleet --strip-components=1 \
  && rm free_fleet.tar.gz

ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

RUN rm -rf \
  /var/lib/apt/lists \
  /dist

ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && . /zenoh-bridge/zenoh-bridge-ros2dds -c /zenoh-brdige/free_fleet/free_fleet_examples/zenoh_configs/turtlebot3_1_zenoh_config.json5"]
