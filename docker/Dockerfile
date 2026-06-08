FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    curl \
    git \
    less \
    ripgrep \
    vim \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-urdf-launch \
    ros-${ROS_DISTRO}-xacro \
    ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces/rmgo_ws

COPY docker/ros_env.sh /etc/profile.d/ros_env.sh
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh /etc/profile.d/ros_env.sh \
 && printf '\n[ -f /etc/profile.d/ros_env.sh ] && source /etc/profile.d/ros_env.sh\n' >> /etc/bash.bashrc \
 && printf '\n[ -f /etc/profile.d/ros_env.sh ] && source /etc/profile.d/ros_env.sh\n' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
