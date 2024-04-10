ARG WORKSPACE=/opt/ros/rr100_ws
ARG CARTOGRAPHER_WS=/opt/ros/cartographer_ws
ARG ROS_DISTRO=noetic
ARG FROM_IMAGE=ros:${ROS_DISTRO}

# Cache apt dependencies
FROM $FROM_IMAGE as apt-depends
RUN --mount=type=cache,target=/var/cache/apt \
    DEBIAN_FRONTEND=noninteractive apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    wget libpcl-dev libmodbus5 libpcap0.8 \
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

# Hack to compile everything separately in later stage
## Should find better way to partition everything
WORKDIR $WORKSPACE/src
RUN mkdir ../dependencies \
    && ls | grep -v 'rho' | grep -v 'gazebo\|simulator\|CMake' | xargs mv -t ../dependencies \
    && mv ../dependencies .
RUN mkdir ../packages \
    && ls | grep -v 'dependencies\|gazebo\|simulator\|CMake' | xargs mv -t ../packages \
    && mv ../packages .
RUN mkdir ../simulation \
    && ls | grep -v 'dependencies\|packages\|CMake' | xargs mv -t ../simulation \
    && mv ../simulation .


#Building stage
FROM apt-depends as builder
ARG WORKSPACE
ARG CARTOGRAPHER_WS

RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-wstool python3-rosdep ninja-build stow

WORKDIR $CARTOGRAPHER_WS
RUN mkdir -p src \
    && wstool init src \
    && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
    && wstool update -t src

# Install package dependencies declared in package.xml files
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && rosdep install -r -y --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO} \
    && src/cartographer/scripts/install_abseil.sh \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    # Unecesssary ? && apt-get remove ros-${ROS_DISTRO}-abseil-cpp \
    && catkin_make_isolated --install --use-ninja

WORKDIR ${WORKSPACE}
COPY --from=cacher /tmp/$WORKSPACE/src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && rosdep install -r -y --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO}
# Install RR100 ros package dependencies
COPY debfiles/* ./debfiles/
RUN python3 debfiles/deploy_debians_noetic.py debfiles && apt -f install

# Copy files from cacher stage
COPY --from=cacher $WORKSPACE/src/ .
RUN mv CMakeLists.txt src \
    && cp -r dependencies/* packages/* simulation/* src

# Compile dependencies as separate layer for better cache
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && ls dependencies | xargs -n 1 basename | xargs catkin_make --use-ninja --only-pkg-with-deps \
    && rm -rf dependencies

# COPY src/packages src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && ls packages | xargs -n 1 basename | xargs catkin_make --use-ninja --only-pkg-with-deps \
    && rm -rf packages


FROM builder as simulation
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
    ignition-citadel vim gdb

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && ls simulation | xargs -n 1 basename | xargs catkin_make --use-ninja --only-pkg-with-deps \
    && rm -rf simulation

FROM builder as real
ENV ROS_MASTER_URI=http://rr-100-07:11311
ENV ROS_IP=192.168.1.241

