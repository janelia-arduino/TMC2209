#!/usr/bin/env python3
"""
PlatformIO task wrapper used by pixi tasks.

Why this exists:
- Selecting which example to build in this repo normally requires editing
  `platformio.ini` (`src_dir = examples/...`).
- PlatformIO supports overriding `src_dir` and `build_dir` via environment
  variables:
    - PLATFORMIO_SRC_DIR
    - PLATFORMIO_BUILD_DIR

This script sets those variables in a cross-platform way (works on Windows,
macOS, Linux) and then invokes the `pio` CLI.

Typical usage (via pixi tasks):
  python tools/pio_task.py build  --example examples/... --env teensy40
  python tools/pio_task.py clean  --example examples/... --env teensy40
  python tools/pio_task.py upload --example examples/... --env teensy40 --port /dev/ttyACM0
  python tools/pio_task.py test   --env native
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


def _project_root() -> Path:
    # Pixi sets PIXI_PROJECT_ROOT when running tasks.
    env_root = os.environ.get("PIXI_PROJECT_ROOT")
    if env_root:
        return Path(env_root).resolve()

    # Fallback: tools/pio_task.py -> repo root is parent of tools/
    return Path(__file__).resolve().parents[1]


def _normalize_rel_path(p: str) -> Path:
    # Treat user input as a repo-relative path.
    # Allow forward slashes on Windows.
    return Path(p.replace("\\", "/"))


def _example_src_dir(root: Path, example: str) -> Path:
    rel = _normalize_rel_path(example)
    src_dir = (root / rel).resolve()
    if not src_dir.exists():
        raise FileNotFoundError(f"Example path does not exist: {rel}")
    return src_dir


def _example_build_dir(root: Path, example: str) -> Path:
    # Keep builds isolated per example to avoid cache collisions when switching.
    rel = _normalize_rel_path(example)
    return (root / ".pio" / "build" / rel).resolve()


def _run(cmd: list[str], *, cwd: Path, env: dict[str, str]) -> int:
    # Print a shell-ish line for transparency.
    printable = " ".join(cmd)
    print(f"+ {printable}")
    try:
        completed = subprocess.run(cmd, cwd=str(cwd), env=env, check=False)
        return int(completed.returncode)
    except FileNotFoundError:
        print("ERROR: 'pio' command not found. Did you run `pixi install`?", file=sys.stderr)
        return 127


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="pio_task.py")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_common_example_env(sp: argparse.ArgumentParser) -> None:
        sp.add_argument(
            "--example",
            default="examples/UnidirectionalCommunication/MoveAtVelocity",
            help="Repo-relative example directory to build (contains a .ino).",
        )
        sp.add_argument(
            "--env",
            default="teensy40",
            help="PlatformIO environment name (e.g. teensy40, uno, mega, pico).",
        )

    # build
    sp_build = sub.add_parser("build", help="Build an example (compile only).")
    add_common_example_env(sp_build)

    # clean
    sp_clean = sub.add_parser("clean", help="Clean build artifacts for an example/env.")
    add_common_example_env(sp_clean)

    # upload
    sp_upload = sub.add_parser("upload", help="Build + upload an example.")
    add_common_example_env(sp_upload)
    sp_upload.add_argument(
        "--port",
        default=None,
        help="Upload port (e.g. /dev/ttyACM0, COM3). If omitted, PlatformIO auto-detects.",
    )

    # flash (clean + upload)
    sp_flash = sub.add_parser("flash", help="Clean + upload an example.")
    add_common_example_env(sp_flash)
    sp_flash.add_argument("--port", default=None, help="Upload port (optional).")

    # rebuild (clean + build)
    sp_rebuild = sub.add_parser("rebuild", help="Clean + build an example (compile only).")
    add_common_example_env(sp_rebuild)

    # deepclean
    sp_deep = sub.add_parser("deepclean", help="Delete the isolated build dir for an example.")
    sp_deep.add_argument(
        "--example",
        default="examples/UnidirectionalCommunication/MoveAtVelocity",
        help="Repo-relative example directory.",
    )

    # test
    sp_test = sub.add_parser("test", help="Run PlatformIO unit tests.")
    sp_test.add_argument(
        "--env",
        default="native",
        help="PlatformIO test environment (default: native).",
    )
    sp_test.add_argument(
        "--verbose",
        action="store_true",
        help="Pass -vvv to PlatformIO.",
    )

    args, unknown = parser.parse_known_args(argv)

    root = _project_root()
    env = os.environ.copy()

    if args.command in ("build", "clean", "upload", "flash", "rebuild"):
        src_dir = _example_src_dir(root, args.example)
        build_dir = _example_build_dir(root, args.example)
        env["PLATFORMIO_SRC_DIR"] = str(src_dir)
        env["PLATFORMIO_BUILD_DIR"] = str(build_dir)

    if args.command == "build":
        return _run(["pio", "run", "-e", args.env, *unknown], cwd=root, env=env)

    if args.command == "clean":
        return _run(["pio", "run", "-e", args.env, "-t", "clean", *unknown], cwd=root, env=env)

    if args.command == "upload":
        cmd = ["pio", "run", "-e", args.env, "-t", "upload"]
        if args.port:
            cmd += ["--upload-port", args.port]
        cmd += unknown
        return _run(cmd, cwd=root, env=env)

    if args.command == "flash":
        # Clean then upload.
        rc = _run(["pio", "run", "-e", args.env, "-t", "clean", *unknown], cwd=root, env=env)
        if rc != 0:
            return rc
        cmd = ["pio", "run", "-e", args.env, "-t", "upload"]
        if args.port:
            cmd += ["--upload-port", args.port]
        cmd += unknown
        return _run(cmd, cwd=root, env=env)

    if args.command == "rebuild":
        rc = _run(["pio", "run", "-e", args.env, "-t", "clean", *unknown], cwd=root, env=env)
        if rc != 0:
            return rc
        return _run(["pio", "run", "-e", args.env, *unknown], cwd=root, env=env)

    if args.command == "deepclean":
        build_dir = _example_build_dir(root, args.example)
        print(f"+ rm -rf {build_dir}")
        shutil.rmtree(build_dir, ignore_errors=True)
        return 0

    if args.command == "test":
        cmd = ["pio", "test", "-e", args.env]
        if args.verbose:
            cmd.append("-vvv")
        cmd += unknown
        return _run(cmd, cwd=root, env=env)

    parser.error("Unknown command")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
