FROM osrf/ros:humble-desktop

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

# Qt + X11 runtime dependencies (required for turtlesim, rviz, rqt)
RUN apt-get update && apt-get install -y \
    libx11-xcb1 \
    libxcb1 \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

