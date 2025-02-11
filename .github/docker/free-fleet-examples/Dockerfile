ARG ROS_DISTRO=jazzy
FROM docker.io/ros:$ROS_DISTRO-ros-base
ARG FREE_FLEET_BRANCH=main
ARG ZENOH_VERSION=1.1.0

RUN apt update && apt install -y curl python3-pip

RUN pip3 install eclipse-zenoh==$ZENOH_VERSION pycdr2 rosbags nudged --break-system-packages

# fetch sources
RUN mkdir -p /ff && cd /ff \
  && curl -sL https://github.com/open-rmf/free_fleet/archive/refs/heads/$FREE_FLEET_BRANCH.tar.gz -o free_fleet.tar.gz \
  && mkdir -p /ff/src/free_fleet && tar zxf free_fleet.tar.gz -C /ff/src/free_fleet --strip-components=1 && rm free_fleet.tar.gz

RUN rosdep update && rosdep install --from-paths /ff/src -yi

RUN cd /ff \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --merge-install --install-base /opt/ff --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf /ff

RUN apt update && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# cleanup
RUN rm -rf \
  /var/lib/apt/lists \
  /dist

# Environment setup
RUN echo '#!/usr/bin/env bash' > /ff_entrypoint.sh
RUN echo 'source /opt/ff/setup.bash' >> /ff_entrypoint.sh
RUN echo 'exec "$@"' >> /ff_entrypoint.sh
RUN chmod +x /ff_entrypoint.sh

ENTRYPOINT ["/ff_entrypoint.sh"]
