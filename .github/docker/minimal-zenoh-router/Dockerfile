ARG ROS_DISTRO=jazzy
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG ZENOH_VERSION=1.0.1

RUN apt update && apt install -y wget unzip

RUN mkdir -p /zenoh && cd /zenoh \
  && wget -O zenoh.zip https://github.com/eclipse-zenoh/zenoh/releases/download/1.0.1/zenoh-1.0.1-x86_64-unknown-linux-gnu-standalone.zip \
  && unzip zenoh.zip \
  && rm zenoh.zip

ENTRYPOINT ["bash", "-c", "/zenoh/zenohd"]
