ARG ROS_DISTRO=noetic
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG ZENOH_BRIDGE_REPO=eclipse-zenoh/zenoh-plugin-ros1
ARG ZENOH_BRIDGE_TAG=main
ARG FREE_FLEET_BRANCH=main

RUN apt update && apt install -y wget unzip build-essential curl

# Get Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

ENV PATH="/root/.cargo/bin:${PATH}"

RUN mkdir -p /zenoh-bridge && cd /zenoh-bridge \
  && curl -sL https://github.com/$ZENOH_BRIDGE_REPO/archive/refs/heads/$ZENOH_BRIDGE_TAG.zip -o zenoh-plugin-ros1.tar.gz \
  && mkdir -p /zenoh-bridge/zenoh-plugin-ros1 \
  && tar zxf zenoh-plugin-ros1.tar.gz -C /zenoh-bridge/zenoh-plugin-ros1 --strip-components=1 \
  && rm zenoh-plugin-ros1.tar.gz

RUN cd /zenoh-bridge/zenoh-plugin-ros1 \
  && cargo build --package zenoh-bridge-ros1 --bin zenoh-bridge-ros1 --release

RUN cd /zenoh-bridge \
  && wget -O turtlebot3_zenoh_bridge_ros1_client_config.json5 https://raw.githubusercontent.com/open-rmf/free_fleet/refs/heads/$FREE_FLEET_BRANCH/free_fleet_examples/config/zenoh/turtlebot3_zenoh_bridge_ros1_client_config.json5

RUN rm -rf \
  /var/lib/apt/lists \
  /dist

ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && /zenoh-bridge/zenoh-bridge-ros1/target/release/zenoh-bridge-ros1 -c /zenoh-bridge/turtlebot3_zenoh_bridge_ros1_client_config.json5"]
