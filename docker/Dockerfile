FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    clang \
    clang-format \
    clang-tidy \
    clangd \
    cmake-format \
    gcc-14 \
    g++-14 \
    gdb \
    curl \
    git \
    less \
    python3-yaml \
    ripgrep \
    vim \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-urdf-launch \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-dev-tools \
 && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 \
 && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces/rmgo_ws

COPY docker/ros_env.sh /etc/profile.d/ros_env.sh
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh /etc/profile.d/ros_env.sh \
 && printf '\n[ -f /etc/profile.d/ros_env.sh ] && source /etc/profile.d/ros_env.sh\n' >> /etc/bash.bashrc \
 && printf '\n[ -f /etc/profile.d/ros_env.sh ] && source /etc/profile.d/ros_env.sh\n' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
