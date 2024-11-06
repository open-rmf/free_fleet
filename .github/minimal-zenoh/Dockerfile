ARG ROS_DISTRO=jazzy
FROM docker.io/ros:$ROS_DISTRO-ros-base

RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null

RUN apt update && apt install -y zenoh

ENTRYPOINT ["bash", "-c", "zenohd"]
