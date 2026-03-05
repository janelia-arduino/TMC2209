#!/usr/bin/env python3
"""Format (or check) all tracked C/C++/Arduino files with clang-format.

Why this exists:
- `git clang-format` is great for formatting staged or WIP changes, but sometimes
  you want a one-shot "format everything" command.
- PlatformIO / Arduino projects often contain `.ino` sources, which we also
  include.

Usage:
  python tools/clang_format_all.py          # formats files in-place
  python tools/clang_format_all.py --check  # fails if any file would change

This script:
- finds files via `git ls-files` (so it only touches tracked files)
- formats with `clang-format --style=file`
- avoids relying on `clang-format --dry-run` (not available in older versions)

It is intended to be run via Pixi:
  pixi run format-all
  pixi run format-check
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


_EXTS = {
    ".c",
    ".cc",
    ".cpp",
    ".cxx",
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
    ".ino",
}


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _git_ls_files(root: Path) -> list[Path]:
    proc = subprocess.run(
        ["git", "ls-files"],
        cwd=str(root),
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if proc.returncode != 0:
        print(proc.stderr, file=sys.stderr)
        raise RuntimeError("Failed to run 'git ls-files'. Are you in a git repo?")

    paths: list[Path] = []
    for line in proc.stdout.splitlines():
        if not line:
            continue
        p = (root / line).resolve()
        if p.suffix.lower() in _EXTS and p.exists():
            paths.append(p)
    return paths


def _clang_format_bytes(path: Path) -> bytes:
    proc = subprocess.run(
        ["clang-format", "--style=file", str(path)],
        cwd=str(path.parent),
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if proc.returncode != 0:
        sys.stderr.buffer.write(proc.stderr)
        raise RuntimeError(f"clang-format failed on {path}")
    return proc.stdout


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="clang_format_all.py")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Do not write files; exit non-zero if formatting changes are needed.",
    )
    args = parser.parse_args(argv)

    root = _repo_root()
    files = _git_ls_files(root)

    if not files:
        print("No tracked source files found.")
        return 0

    needs_format: list[Path] = []

    for path in files:
        original = path.read_bytes()
        formatted = _clang_format_bytes(path)

        if original != formatted:
            if args.check:
                needs_format.append(path)
            else:
                path.write_bytes(formatted)

    if args.check:
        if needs_format:
            print("Files need formatting:")
            for p in needs_format:
                # Print repo-relative paths.
                try:
                    print(f"  {p.relative_to(root)}")
                except Exception:
                    print(f"  {p}")
            return 1
        print("Formatting OK.")
        return 0

    print(f"Formatted {len(files)} files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
