FROM ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Workspace
WORKDIR /morphobot_ws

# Use bash
SHELL ["/bin/bash", "-c"]

# Source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# rosdep (safe)
RUN rosdep init || echo "rosdep already initialized"

ENTRYPOINT ["/bin/bash"]
