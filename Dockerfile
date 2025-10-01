ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ENV ROS_DISTRO=${ROS_DISTRO}
ARG MUJOCO_VERSION=3.2.6

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Delete existing user if it exists
RUN if getent passwd ${USER_UID}; then \
    userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi

# Delete existing group if it exists
RUN if getent group ${USER_GID}; then \
    groupdel $(getent group ${USER_GID} | cut -d: -f1); \
    fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    vim \
    nano \
    xterm \
    build-essential \
    cmake \
    wget \
    git \
    unzip \
    pip \
    python3-venv \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-colcon-common-extensions \
    cmake \
    libpoco-dev \
    libeigen3-dev \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rmw-zenoh-cpp \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-rqt-joint-trajectory-controller \
    dpkg \
    freeglut3-dev \
    liblapacke-dev \
    libopenblas-dev \
    libpcap-dev \
    libusb-1.0-0-dev \
    libx11-dev \
    zlib1g-dev

# Symlink python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python

USER $USERNAME

RUN mkdir -p /home/ros/ros2_ws

WORKDIR /home/ros/ros2_ws

RUN git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git src/cartesian_controllers 
RUN git clone https://github.com/asymingt/libsurvive_ros2.git src/libsurvive_ros2 
RUN git clone https://github.com/JafarAbdi/feetech_ros2_driver.git src/feetech_ros2_driver 
    
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep update \
    && rosdep install --ignore-src --from-paths . -y -r \
    && colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install 