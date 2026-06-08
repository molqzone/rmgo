#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "/workspaces/rmgo_ws/install/setup.bash" ]; then
    source "/workspaces/rmgo_ws/install/setup.bash"
fi

exec "$@"
