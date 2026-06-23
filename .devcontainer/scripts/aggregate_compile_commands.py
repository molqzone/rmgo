#!/usr/bin/env python3

import json
import os
import sys
from pathlib import Path


def devcontainer_workspace_root(build_dir: Path) -> str:
    workspace_root = os.environ.get("RMGO_CLANGD_WORKSPACE_ROOT")
    if workspace_root:
        return workspace_root.rstrip("/")

    workspace_root = build_dir.parent.as_posix().rstrip("/")
    if workspace_root.startswith("/workspaces/"):
        return workspace_root

    return "/workspaces/rmgo/rmgo_ws"


def legacy_workspace_roots(
    build_dir: Path,
    target_workspace_root: str,
) -> tuple[str, ...]:
    roots = {
        build_dir.parent.as_posix().rstrip("/"),
        "/workspaces/rmgo_ws",
        "/workspaces/rmgo/rmgo_ws",
        target_workspace_root,
    }
    return tuple(sorted(roots - {target_workspace_root}, key=len, reverse=True))


def normalize_path_roots(
    value: str,
    build_dir: Path,
    target_workspace_root: str,
) -> str:
    normalized = value.replace("\\", "/")
    for legacy_root in legacy_workspace_roots(build_dir, target_workspace_root):
        normalized = normalized.replace(legacy_root, target_workspace_root)
    return normalized


def normalize_command(
    command: dict,
    build_dir: Path,
    target_workspace_root: str,
) -> dict:
    normalized = dict(command)
    for key in ("directory", "file", "output", "command", "arguments"):
        value = normalized.get(key)
        if isinstance(value, str):
            normalized[key] = normalize_path_roots(value, build_dir, target_workspace_root)
        elif isinstance(value, list):
            normalized[key] = [
                normalize_path_roots(item, build_dir, target_workspace_root)
                if isinstance(item, str)
                else item
                for item in value
            ]
    return normalized


def write_database(path: Path, entries: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(entries, indent=2) + "\n")


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: aggregate_compile_commands.py <build-dir>", file=sys.stderr)
        return 2

    build_dir = Path(sys.argv[1]).resolve()
    target_workspace_root = devcontainer_workspace_root(build_dir)
    legacy_output = build_dir / "rmgo_core" / "compile_commands.json"
    databases = sorted(
        path
        for path in build_dir.glob("*/compile_commands.json")
        if path.parent != build_dir and path != legacy_output
    )

    entries = []
    seen = set()
    for database in databases:
        try:
            commands = json.loads(database.read_text())
        except (OSError, json.JSONDecodeError) as error:
            print(f"warning: failed to read {database}: {error}", file=sys.stderr)
            continue

        if not isinstance(commands, list):
            print(f"warning: skipped {database}: root entry is not a list", file=sys.stderr)
            continue

        for index, command in enumerate(commands):
            if not isinstance(command, dict):
                print(
                    f"warning: skipped {database} entry {index}: expected object",
                    file=sys.stderr,
                )
                continue
            command = normalize_command(command, build_dir, target_workspace_root)
            key = (command.get("file"), command.get("directory"), command.get("command"))
            if key in seen:
                continue
            seen.add(key)
            entries.append(command)

    output = build_dir / "compile_commands.json"
    write_database(output, entries)

    # VS Code Remote may keep stale clangd arguments from an existing
    # devcontainer. Keep that old path useful until the window is rebuilt.
    write_database(legacy_output, entries)

    print(f"Wrote {len(entries)} compile command(s) to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
