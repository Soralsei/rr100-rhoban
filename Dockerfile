FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "--login", "-c"] 

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN DEBIAN_FRONTEND=noninteractive sudo apt update \
&& sudo apt upgrade -y \
&& sudo apt install -y --no-install-recommends -y \
wget libpcl-dev libmodbus5 libpcap0.8 git ros-${ROS_DISTRO}-tf2-tools \
python-is-python3 \
ros-${ROS_DISTRO}-rqt \
ros-${ROS_DISTRO}-rqt-common-plugins \
ros-${ROS_DISTRO}-rqt-robot-plugins \
ros-${ROS_DISTRO}-image-transport-plugins

# Rosdep update
RUN rosdep update --rosdistro ${ROS_DISTRO}

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

WORKDIR /home/ubuntu

# Create conda env for zmq rl_controller
RUN  echo $(pwd) && arch=$(uname -m) && \
if [ "$arch" = "x86_64" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh"; \
elif [ "$arch" = "aarch64" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh"; \
else \
    echo "Unsupported architecture: $arch"; \
    exit 1; \
fi && \
wget $MINICONDA_URL -O miniconda.sh && \
mkdir -p ${HOME}/.conda && \
bash miniconda.sh -b -p ${HOME}/miniconda3 && \
rm -f miniconda.sh

COPY requirements_rl.txt .

RUN $HOME/miniconda3/bin/conda init \
&& $HOME/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main \ 
&& $HOME/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
RUN $HOME/miniconda3/bin/conda create -n rl python=3.10 -y \
&& $HOME/miniconda3/bin/conda run -n rl pip install -r requirements_rl.txt \
&& $HOME/miniconda3/bin/conda run -n rl pip install zmq \
&& $HOME/miniconda3/bin/conda config --set auto_activate_base false

RUN echo "source $HOME/rr100_ws/devel/setup.bash" >> $HOME/.bashrc
