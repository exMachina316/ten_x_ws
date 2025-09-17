ARG ROS_DISTRO=jazzy

# Use base image, https://hub.docker.com/_/ros/
FROM ros:$ROS_DISTRO-ros-base
# Prevent console from interacting with the user
# Read more here - https://bobcares.com/blog/debian_frontendnoninteractive-docker
ARG DEBIAN_FRONTEND=noninteractive

# Prevent hash mismatch error for apt-get update, qqq makes the terminal quiet while downloading pkgs
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Set folder for RUNTIME_DIR. Only to prevent warnings when running RViz2 and Gz
RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# https://stackoverflow.com/questions/51023312/docker-having-issues-installing-apt-utils

# Non Python/ROS Dependencies
# apt-utils: https://stackoverflow.com/questions/51023312/docker-having-issues-installing-apt-utils
RUN apt-get install --no-install-recommends -y \
    apt-utils \
    vim \
    wget \
    libglfw3 libglfw3-dev

# Python Dependencies
RUN apt-get install --no-install-recommends -y \
    python3-pip

# Install ros2 control
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rqt-controller-manager

# Install Gazebo
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-gz-ros2-control

# Install turtlebot3 packages
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-turtlebot3-simulations \
    ros-$ROS_DISTRO-turtlebot3

# Install util packages
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-teleop-twist-keyboard

# Using shell to use bash commands like 'source'
SHELL ["/bin/bash", "-c"]

# Target workspace for ROS2 packages
ARG WORKSPACE=/root/ten_x_ws

# Add target workspace in environment
ENV WORKSPACE=$WORKSPACE

# Create folders
RUN mkdir -p $WORKSPACE/src && \
    mkdir -p /scripts

COPY scripts/utils $WORKSPACE/scripts/

# For .bashrc
ENV ROS_DISTRO=$ROS_DISTRO
WORKDIR $WORKSPACE

COPY workspace $WORKSPACE/src/

# Build workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd $WORKSPACE && \
    apt-get update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    colcon build --symlink-install

# Update .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source $WORKSPACE/install/setup.bash" >> /root/.bashrc
