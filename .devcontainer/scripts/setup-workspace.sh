#!/usr/bin/env bash

set -eu

workspace_root="/workspaces/rmgo/rmgo_ws"
src_path="$workspace_root/src"

if [ ! -d "$src_path" ]; then
    printf 'Workspace source path not found: %s\n' "$src_path" >&2
    exit 1
fi

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi

rosdep update
rosdep install --from-paths "$src_path" --ignore-src -r -y

cd "$workspace_root"
colcon build

printf 'Workspace setup complete: %s\n' "$workspace_root"
