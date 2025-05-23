### Build bridge

ARG ROS_DISTRO=noetic
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG ZENOH_BRIDGE_REPO=eclipse-zenoh/zenoh-plugin-ros1
ARG ZENOH_BRIDGE_TAG=main
ARG FREE_FLEET_BRANCH=main

RUN apt update && apt install -y build-essential curl git unzip wget

# Get Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

ENV PATH="/root/.cargo/bin:${PATH}"

RUN mkdir -p /zenoh-bridge && cd /zenoh-bridge \
  && git clone --recursive https://github.com/$ZENOH_BRIDGE_REPO -b $ZENOH_BRIDGE_TAG zenoh-plugin-ros1 \
  && cd /zenoh-bridge/zenoh-plugin-ros1 \
  && cargo build --package zenoh-bridge-ros1 --bin zenoh-bridge-ros1 --release

RUN cd /zenoh-bridge \
  && wget -O nav1_tb3_zenoh_bridge_ros1_client_config.json5 https://raw.githubusercontent.com/open-rmf/free_fleet/refs/heads/$FREE_FLEET_BRANCH/free_fleet_examples/config/zenoh/nav1_tb3_zenoh_bridge_ros1_client_config.json5

RUN rm -rf \
  /var/lib/apt/lists \
  /dist

### Set up bare minimum zenoh-bridge-ros1 image

FROM docker.io/ros:$ROS_DISTRO-ros-base

COPY --from=0 /zenoh-bridge/nav1_tb3_zenoh_bridge_ros1_client_config.json5 /zenoh-bridge/nav1_tb3_zenoh_bridge_ros1_client_config.json5

COPY --from=0 /zenoh-bridge/zenoh-plugin-ros1/target/release/zenoh-bridge-ros1 /zenoh-bridge/zenoh-bridge-ros1

ENTRYPOINT ["bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && /zenoh-bridge/zenoh-bridge-ros1 -c /zenoh-bridge/nav1_tb3_zenoh_bridge_ros1_client_config.json5"]
