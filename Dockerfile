ARG WORKSPACE=/opt/ros/rr100_ws
ARG FROM_IMAGE=ros:noetic
ARG IP=192.168.0.64
ARG BUILD_SHA

# Cache apt dependencies
FROM $FROM_IMAGE as apt-depends
LABEL "rhoban.project.name"="rhoban-rr100" \
"author"="Kohio Deflesselle"

RUN --mount=type=cache,target=/var/cache/apt \
DEBIAN_FRONTEND=noninteractive apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
wget libpcl-dev libmodbus5 libpcap0.8 \
ros-${ROS_DISTRO}-tf2-tools \
ros-${ROS_DISTRO}-rqt \
ros-${ROS_DISTRO}-rqt-common-plugins \
ros-${ROS_DISTRO}-rqt-robot-plugins \
ros-${ROS_DISTRO}-image-transport-plugins \
&& rm -rf /var/lib/apt/lists/*

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

# Hacky way to compile everything separately in later stage
## Should find better way to partition everything
## Here, any package in src/ that doesn't have 'rr100' 
## and 'simulator' in its name is treated as a dependency
WORKDIR $WORKSPACE/src
RUN mkdir ../dependencies \
&& ls | grep -v 'rhoban\|rr100\|simulator\|CMake' | xargs mv -t ../dependencies \
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

WORKDIR ${WORKSPACE}

# Install RR100 ros packages
COPY debfiles/* ./debfiles/
RUN python3 debfiles/deploy_debians_noetic.py debfiles

# Install all workspace packages dependencies
COPY --from=cacher /tmp/$WORKSPACE/src ./src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
&& apt-get update \
&& rosdep update \
&& rosdep install -r -y --from-paths ./src --ignore-src --rosdistro ${ROS_DISTRO} \
&& apt-get upgrade -y \
&& rm -rf /var/lib/apt/lists/*

# Copy files from cacher stage
COPY --from=cacher $WORKSPACE/src/ .

# Compile dependencies as separate layer for better cache
RUN mv CMakeLists.txt src \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cp -r dependencies/* src \
    && ls dependencies | xargs -n 1 basename | xargs catkin_make --only-pkg-with-deps \
    && rm -rf dependencies

# Compile actual RR100 project packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cp -r packages/* src \
    && ls packages | xargs -n 1 basename | xargs catkin_make --only-pkg-with-deps \
    && rm -rf packages

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cp -r simulation/* src \
    && ls -lar simulation/ \
    && ls -lar src/rr100_description \
    && ls simulation | xargs -n 1 basename | xargs catkin_make --only-pkg-with-deps \
    && rm -rf simulation

ENV WORKSPACE=$WORKSPACE
RUN sed --in-place --expression \
    '$isource "$WORKSPACE/devel/setup.bash"' \
    /ros_entrypoint.sh \
    && echo "source ${WORKSPACE}/devel/setup.bash" >> ~/.bashrc

COPY Dockerfile .

ARG BUILD_SHA
LABEL "rhoban.rr100.build.sha"=${BUILD_SHA}

FROM builder as simulation
ENV ROS_MASTER_URI=http://localhost:11311

# Do not remove, used to identify build target
LABEL target="simulation"

FROM builder as real
ARG IP
ENV ROS_MASTER_URI=http://rr-100-07:11311
ENV ROS_IP=${IP}

# Do not remove, used to identify build target
LABEL target="real"
