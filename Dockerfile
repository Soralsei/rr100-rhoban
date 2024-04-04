ARG FROM_IMAGE=ros:noetic
ARG WORKSPACE=/opt/ros/rr100_ws

FROM $FROM_IMAGE AS cacher
ARG WORKSPACE

WORKDIR $WORKSPACE/src
COPY rr100_ws/src .

WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find . -name "package.xml" | \
        xargs cp --parents -t /tmp/opt && \
    find . -name "CATKIN_IGNORE" | \
        xargs cp --parents -t /tmp/opt || true

FROM $FROM_IMAGE as builder
ARG WORKSPACE
WORKDIR $WORKSPACE
COPY --from=cacher /tmp/$WORKSPACE/src ./src
RUN --mount=type=cache,target=/var/cache/apt \
    DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-universal-robots \
    ros-${ROS_DISTRO}-ur-robot-driver \
    ros-${ROS_DISTRO}-librealsense2 \
    libpcl-dev libmodbus5 libpcap0.8 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-robot-plugins

COPY debfiles/* ./debfiles/
RUN python3 debfiles/deploy_debians_noetic.py debfiles && apt -f install

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update --include-eol-distros \
    && rosdep install -r -y --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO}

COPY --from=cacher $WORKSPACE/src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make

ENV ROS_MASTER_URI=http://rr-100-07:11311