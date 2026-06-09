# rmgo ROS 2 Jazzy Docker Workspace

This repository uses Docker to provide a ROS 2 Jazzy development environment.
The ROS workspace root is `rmgo_ws`, and ROS packages should go in `rmgo_ws/src`.

Current workspace packages:

- `rmgo_bringup`
- `rmgo_core`
- `rmgo_utility`
- `rmgo_description`

## Prerequisites

- Docker Desktop installed and running
- Docker Compose V2 (`docker compose`)

## Build the image

```powershell
docker compose build
```

## Open a shell in the ROS container

```powershell
docker compose run --rm ros2
```

The container starts with ROS 2 Jazzy sourced automatically. After the workspace
has been built once, `rmgo_ws/install/setup.bash` is sourced automatically too.
The image also installs the standard ROS development tools used by `rosdep`,
`colcon`, and package templates.

## Build the workspace

From inside the container:

```bash
cd /workspaces/rmgo_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

Because `rmgo_ws/.colcon/defaults.yaml` enables `symlink-install`, you do not
need to pass `--symlink-install` each time.

## Add a package

From inside the container:

```bash
cd /workspaces/rmgo_ws/src
ros2 pkg create --build-type ament_cmake my_package
```

Then rebuild:

```bash
cd /workspaces/rmgo_ws
colcon build
```

## Preview the RMCS-derived omni infantry model

From inside the container:

```bash
ros2 launch rmgo_description display.launch.py
```

## Devcontainer

This repo also includes an RMCS-style devcontainer under `.devcontainer/`.
Open the repository root in VS Code and choose `Reopen in Container`.

The devcontainer uses the repository root as the workspace folder and keeps the
existing ROS workspace at `rmgo_ws/` inside it.
The initialization step uses `bash` on the host, so on Windows you should have
Git Bash or WSL available.

TODO: Delete all trace in the future when stable.

