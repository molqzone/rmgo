#!/usr/bin/env bash

set -eu

repo_root="/workspaces/rmgo"
workspace_root="$repo_root/rmgo_ws"
src_path="$workspace_root/src"

if [ ! -d "$src_path" ]; then
    printf 'Workspace source path not found: %s\n' "$src_path" >&2
    exit 1
fi

restore_nounset=0
if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
fi

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ "$restore_nounset" -eq 1 ]; then
    set -u
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init || printf 'Warning: rosdep init failed, continuing with existing rosdep state.\n' >&2
fi

if ! rosdep update --rosdistro "${ROS_DISTRO:-jazzy}"; then
    printf 'Warning: rosdep update failed, likely due to network timeout. Continuing with cached rosdep data.\n' >&2
fi

if ! rosdep install --from-paths "$src_path" --ignore-src -r -y --rosdistro "${ROS_DISTRO:-jazzy}"; then
    printf 'Warning: rosdep install did not resolve every dependency. Continuing because core dependencies are baked into the image.\n' >&2
fi

cd "$workspace_root"
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

python3 "$repo_root/.devcontainer/scripts/aggregate_compile_commands.py" "$workspace_root/build"

printf 'Workspace setup complete: %s\n' "$workspace_root"
