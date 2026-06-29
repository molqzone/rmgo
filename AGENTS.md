# AGENTS.md

Instructions for AI coding agents working on the RoboMaster Go (rmgo) robot
control system.

## Environment

Everything runs inside the Docker/ROS 2 Jazzy container. The root
`docker-compose.yml` mounts `./rmgo_ws` as the ROS workspace and starts in it.
Before running Docker/build commands, check the host OS first.

```bash
# Host → container (native Linux runs `docker compose` directly;
# on Windows, wrap with `wsl.exe -- bash -lc "cd <wsl-path> && docker compose ..."`)
docker compose build
docker compose run --rm ros2

# Inside container
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon build --symlink-install --packages-up-to rmgo_core   # rebuild one package + deps
colcon test --packages-select rmgo_core && colcon test-result --verbose
source install/setup.bash
```

On Windows, use PowerShell 7 (`pwsh`) as the host shell before entering WSL.
If Docker CLI exists but the daemon is down, try to start it once: use the normal
Linux service command for Docker Engine, or start Docker Desktop with WSL
integration on Windows. If Docker is still unavailable, stop and report an
environment issue. Don't attempt native Windows builds; Linux is the only
supported build/runtime target.

## Project Layout

```text
rmgo_ws/src/rmgo_core/          ros2_control controllers + Gazebo hardware interface
rmgo_ws/src/rmgo_referee/        Lifecycle referee serial node, frame translation, UI
rmgo_ws/src/rmgo_utility/        Header-only shared helpers (NodeMixin, watchdog, CRC, serial)
rmgo_ws/src/rmgo_msg/            ROS message definitions (public package contracts)
rmgo_ws/src/rmgo_description/    Xacro/URDF robot descriptions, meshes, RViz config
rmgo_ws/src/rmgo_bringup/        YAML configs and launch orchestration
rmgo_ws/src/fast_tf/             Git submodule — treat as external unless asked to edit
docker/                          Container image, entrypoint, shell hooks
```

Omni-infantry plugins are grouped under the robot-specific
`rmgo_omni_infantry_plugins` library. Use robot/profile names for plugin groups,
not vague subsystem names. When adding/renaming/removing a controller, keep its
CMake target/source list, generated parameter library entry, controller plugin
XML, install rules, and bringup YAML in sync. Only touch hardware plugin XML for
hardware interfaces.

## Standards

- **C++23**, **GCC 14**, **Linux only**. ROS 2 Jazzy compatibility required.
- **`-Wall -Wextra -Wpedantic`** — treat warnings as defects.
- **clang-format**: LLVM base, 4-space indent, 100-col limit, left pointers.
  Run `tools/format_cpp_files.sh` before committing; CI rejects unformatted code.
- **cmake-format**: lower-case commands, upper-case keywords, 90-col, 2-space indent.
- **Conventional Commits** for commit messages.

## Code Style (principles, not just rules)

**Naming.** Keep namespaces explicit and package-local. Don't repeat the
namespace in names. `rmgo_referee::Node`, not `rmgo_referee::RefereeNode`.

**Logging.** All node-like classes inherit `rmgo_utility::NodeMixin` and use
`logging::info("... {}", value)`. Prefer `std::format`-style; never use
stringstream or `+` concatenation at formatting boundaries.

**Comments.** Explain robot/control assumptions and non-obvious safety behavior.
Skip line-by-line mechanics.

**Parameters.** Controller parameters use `generate_parameter_library` from YAML
under `config/controller/`. Extend those schemas; avoid ad hoc runtime lookups.

## Three Architectural Rules

These drive every design decision in this codebase.

### 1. Fail Fast

Validate everything checkable during construction/configuration. After that
succeeds, the object assumes validity — don't re-validate invariants.

- No `default` values in parameter YAML for correctness-critical fields.
- Validate cross-parameter constraints in configure/activate when
  `generate_parameter_library` cannot express them.
- Missing required interfaces → activation error. Failed writes → update error.
- Function signatures are contracts. If the type says a value is present, don't
  add another fallback layer. Only check real runtime hazards (out-of-range,
  missing index, I/O failure).
- The exception: safety timeouts (stale game/robot/power/heat data) may degrade
  to safe values — surface these through diagnostics.

### 2. No Heap in Hot Paths

Hot paths: controller `update`, `HardwareInterface::read/write`, realtime
publishers, high-frequency timers, serial frame processing.

- Allocate during init/configure/activate. Use fixed storage in hot paths:
  `std::array`, `std::span`, reserved vectors, prebuilt interface indexes.
- Use the provided `period` for controller timing. Do not hide control-loop
  assumptions behind hardcoded update rates.
- Forbidden on hot paths: `new`, `make_unique`/`make_shared`, vector growth,
  string construction, `std::format`, parameter lookup, ROS entity creation,
  blocking I/O.
- Error paths may allocate for diagnostics after leaving the hot path.

### 3. `std::expected` for Errors, Not Exceptions

- Use `std::expected<T, E>` with structured error codes for recoverable errors
  inside workspace code. Keep error flow explicit in return types.
- Let ROS 2 framework exceptions propagate to the executor and crash — don't
  wrap them.
- Convert structured errors to human-readable strings and log them at cold
  framework boundaries (lifecycle callbacks, configure/activate/deactivate).
- Hot-path entry points (`read`/`write`/`update`): return failure, update
  preallocated state, or surface via a non-realtime diagnostic path. Don't
  format or log there.

## Quick Checks Before Committing

1. `tools/format_cpp_files.sh` — format tracked C++ sources (`git add -N` new
   files first)
2. `colcon build --symlink-install` — must build clean
3. `colcon test --packages-select <changed-package>` — tests pass
4. Did you add a controller? Update plugin XMLs, install rules, bringup YAML.
5. Did you change a `.msg` file? That's a public contract — check downstream
   consumers.
