# Use official ROS 2 Humble desktop image as base
FROM osrf/ros:humble-desktop AS base

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update, upgrade, and install essential tools including vulnerability checker
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        debsums \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        git \
        curl \
        gnupg2 \
        lsb-release && \
    debsums -c && \
    rm -rf /var/lib/apt/lists/*

# Add ROS GZ (Ignition) repository key and source

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://packages.osrfoundation.org/gazebo.key | apt-key add - && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-latest.list'
# Initialize rosdep and update database
RUN rosdep update

# Set workspace directory and copy your source code
WORKDIR /ros2_ws
COPY . /ros2_ws

# Install all ROS dependencies defined in your source packages
RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/humble/setup.sh
RUN colcon build 

# Source ROS 2 and workspace setup scripts in all bash shells
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# Default to bash shell
CMD ["bash"]
