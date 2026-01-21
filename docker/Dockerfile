# ROS distribution to use
ARG ROS_DISTRO=jazzy

##############
# Base Image #
##############
FROM ros:${ROS_DISTRO} AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
# Install apt packages
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      curl \
      libi2c-dev \
      python3-pip \
      libserial-dev \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-teleop-twist-keyboard  \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-ros2controlcli \
      ros-${ROS_DISTRO}-hardware-interface \
      ros-${ROS_DISTRO}-diff-drive-controller \
      ros-${ROS_DISTRO}-joint-state-broadcaster \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-example-interfaces \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# For IMU node
RUN python3 -m pip install smbus pyserial transforms3d --break-system-packages

# Use Cyclone DDS as middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create Colcon workspace with external dependencies
RUN mkdir -p /base_ws/src
WORKDIR /base_ws/src

COPY dependencies .

#COPY dependencies.repos .
#RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /base_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y \
 && colcon build --symlink-install

# Set up the entrypoint
WORKDIR /base_ws
COPY ./docker/entrypoint.sh /
ENV DEBIAN_FRONTEND=dialog

ENTRYPOINT [ "/entrypoint.sh" ]

#################
# Overlay Image #
#################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./my_diffbot_bringup/ ./src/my_diffbot_bringup/
COPY ./my_diffbot_description/ ./src/my_diffbot_description/
COPY ./my_diffbot_hardware_interface/ ./src/my_diffbot_hardware_interface/
COPY ./my_diffbot_localization/ ./src/my_diffbot_localization/
COPY ./my_diffbot_navigation/ ./src/my_diffbot_navigation/
RUN source /base_ws/install/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y \
 && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay AS dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
 gdb gdbserver nano neovim tmux git

# In Ubuntu 24.04, there is already a user named "ubuntu" with UID 1000.
# Delete this in the (common) event that the user on the host also has this UID.
RUN touch /var/mail/ubuntu \
    && chown ubuntu /var/mail/ubuntu \
    && userdel -r ubuntu

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /overlay_ws/

# Add user to dialout and i2c groups for device access
RUN usermod -aG dialout ${USERNAME}
#RUN groupadd -r i2c 
RUN usermod -aG i2c ${USERNAME}

# Set up the entrypoint, including it in the .bashrc for interactive shells
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
