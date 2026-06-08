#!/bin/bash

if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

for workspace_setup in \
    "/workspaces/rmgo/rmgo_ws/install/setup.bash" \
    "/workspaces/rmgo_ws/install/setup.bash"
do
    if [ -f "$workspace_setup" ]; then
        source "$workspace_setup"
        break
    fi
done
