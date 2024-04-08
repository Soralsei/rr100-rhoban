ARG FROM_IMAGE=ros:noetic
ARG WORKSPACE=/opt/ros/rr100_ws

FROM $FROM_IMAGE as apt-depends
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

# Caching stage
FROM $FROM_IMAGE AS cacher
ARG WORKSPACE

WORKDIR $WORKSPACE/src
COPY src .

# Separate package.xml files in /tmp directory
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find . -name "package.xml" | \
        xargs cp --parents -t /tmp/opt && \
    find . -name "CATKIN_IGNORE" | \
        xargs cp --parents -t /tmp/opt || true

# Hack to copy dependencies in a single COPY command later
WORKDIR $WORKSPACE/src
RUN mkdir ../dependencies && ls | grep -v 'rho' | xargs mv -t ../dependencies && mv ../dependencies . && ls -la

#Building stage
FROM apt-depends as builder
ARG WORKSPACE
WORKDIR $WORKSPACE

# Install RR100 ros package dependencies
COPY debfiles/* ./debfiles/
RUN python3 debfiles/deploy_debians_noetic.py debfiles && apt -f install

# Install package dependencies declared in package.xml files
COPY --from=cacher /tmp/$WORKSPACE/src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update --include-eol-distros \
    && rosdep install -r -y --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO}

# Compile dependencies as separate layer for better cache
COPY --from=cacher $WORKSPACE/src/dependencies ./src/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && catkin_make --only-pkg-with-deps fast_gicp hdl_global_localization \
        hdl_graph_slam hdl_localization ndt_omp \
        realsense-ros robosense_simulator

# Compile actual project packages
COPY --from=cacher $WORKSPACE/src/ ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rm -rf src/dependencies \
    && catkin_make --only-pkg-with-deps rho_rr100_gazebo rhoban_package

FROM builder as simulation
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
    vim gdb valgrind
#     ros-${ROS_DISTRO}-interactive-marker-twist-server \
#     ros-${ROS_DISTRO}-twist-mux
    # ros-${ROS_DISTRO}-four-wheel-steering-controller

FROM builder as real
ENV ROS_MASTER_URI=http://rr-100-07:11311
ENV ROS_IP=192.168.1.241

