ARG ROS_DISTRO=noetic
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG ZENOH_VERSION=1.0.0-beta.2
ARG FREE_FLEET_BRANCH=main

RUN apt update && apt install -y wget unzip ros-jazzy-rmw-cyclonedds-cpp

RUN mkdir -p /zenoh-bridge && cd /zenoh-bridge \
  && wget -O zenoh-plugin-ros1.zip https://github.com/eclipse-zenoh/zenoh-plugin-ros1/releases/download/$ZENOH_VERSION/zenoh-plugin-ros1-$ZENOH_VERSION-x86_64-unknown-linux-gnu-standalone.zip \
  && unzip zenoh-plugin-ros1.zip \
  && rm zenoh-plugin-ros1.zip

RUN cd /zenoh-bridge \
  && wget -O turtlebot3_1_client_zenoh_config.json5 https://raw.githubusercontent.com/open-rmf/free_fleet/refs/heads/$FREE_FLEET_BRANCH/free_fleet_examples/config/zenoh/turtlebot3_1_client_zenoh_config.json5

ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

RUN rm -rf \
  /var/lib/apt/lists \
  /dist

ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && /zenoh-bridge/zenoh-bridge-ros2dds -c /zenoh-bridge/turtlebot3_1_client_zenoh_config.json5"]
