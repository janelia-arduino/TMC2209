#!/usr/bin/env python3
"""Synchronize version numbers across common project metadata files.

This repo keeps the version number in multiple places:
- library.properties          (Arduino Library Manager metadata)
- .metadata/README.org        (source for generated README.md)
- pixi.toml                   (Pixi workspace version)

The goal is to keep them consistent, especially when doing a release.

Usage:
  python tools/version_sync.py check
  python tools/version_sync.py set 11.0.1

Intended to be run via Pixi tasks:
  pixi run check-version
  pixi run set-version -- 11.0.1
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Versions:
    library_properties: str
    readme_org: str
    pixi_toml: str


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _read_library_properties_version(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    for line in text.splitlines():
        if line.startswith("version="):
            return line.split("=", 1)[1].strip()
    raise RuntimeError(f"No 'version=' line found in {path}")


def _write_library_properties_version(path: Path, version: str) -> None:
    lines = path.read_text(encoding="utf-8").splitlines(True)
    out: list[str] = []
    changed = False
    for line in lines:
        if line.startswith("version="):
            out.append(f"version={version}\n")
            changed = True
        else:
            out.append(line)
    if not changed:
        raise RuntimeError(f"No 'version=' line found in {path}")
    path.write_text("".join(out), encoding="utf-8")


def _read_readme_org_version(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    # Example line: "- Version :: 11.0.0"
    m = re.search(r"^\-\s+Version\s+::\s+([^\s]+)\s*$", text, flags=re.MULTILINE)
    if not m:
        raise RuntimeError(f"No '- Version ::' line found in {path}")
    return m.group(1).strip()


def _write_readme_org_version(path: Path, version: str) -> None:
    lines = path.read_text(encoding="utf-8").splitlines(True)
    out: list[str] = []
    changed = False
    for line in lines:
        if re.match(r"^\-\s+Version\s+::\s+", line):
            out.append(f"- Version :: {version}\n")
            changed = True
        else:
            out.append(line)
    if not changed:
        raise RuntimeError(f"No '- Version ::' line found in {path}")
    path.write_text("".join(out), encoding="utf-8")


def _read_pixi_toml_version(path: Path) -> str:
    # We intentionally do a lightweight parse (no external deps).
    lines = path.read_text(encoding="utf-8").splitlines()
    in_workspace = False
    for line in lines:
        stripped = line.strip()
        if stripped == "[workspace]":
            in_workspace = True
            continue
        if in_workspace and stripped.startswith("[") and stripped != "[workspace]":
            # Next section.
            break
        if in_workspace:
            m = re.match(r"^version\s*=\s*\"([^\"]+)\"\s*$", stripped)
            if m:
                return m.group(1)
    raise RuntimeError(f"No [workspace].version found in {path}")


def _write_pixi_toml_version(path: Path, version: str) -> None:
    lines = path.read_text(encoding="utf-8").splitlines(True)
    out: list[str] = []
    in_workspace = False
    changed = False

    for line in lines:
        stripped = line.strip()
        if stripped == "[workspace]":
            in_workspace = True
            out.append(line)
            continue
        if in_workspace and stripped.startswith("[") and stripped != "[workspace]":
            in_workspace = False

        if in_workspace:
            m = re.match(r"^version\s*=\s*\"([^\"]+)\"\s*$", stripped)
            if m:
                # Preserve indentation (if any).
                indent = line[: len(line) - len(line.lstrip(" \t"))]
                out.append(f"{indent}version = \"{version}\"\n")
                changed = True
                continue

        out.append(line)

    if not changed:
        raise RuntimeError(f"No [workspace].version found in {path}")

    path.write_text("".join(out), encoding="utf-8")


def _read_versions(root: Path) -> Versions:
    return Versions(
        library_properties=_read_library_properties_version(root / "library.properties"),
        readme_org=_read_readme_org_version(root / ".metadata" / "README.org"),
        pixi_toml=_read_pixi_toml_version(root / "pixi.toml"),
    )


def _validate_version(version: str) -> None:
    # Basic semver-ish validation: 1.2.3, optionally with -suffix
    if not re.match(r"^\d+\.\d+\.\d+([\-\+].+)?$", version):
        raise ValueError(f"Invalid version: {version!r} (expected e.g. 11.0.0)")


def cmd_check() -> int:
    root = _repo_root()
    v = _read_versions(root)

    all_versions = {v.library_properties, v.readme_org, v.pixi_toml}
    if len(all_versions) == 1:
        print(f"OK: version = {v.library_properties}")
        return 0

    print("ERROR: version mismatch")
    print(f"  library.properties: {v.library_properties}")
    print(f"  .metadata/README.org: {v.readme_org}")
    print(f"  pixi.toml: {v.pixi_toml}")
    return 1


def cmd_set(version: str) -> int:
    _validate_version(version)

    root = _repo_root()

    lib_path = root / "library.properties"
    readme_path = root / ".metadata" / "README.org"
    pixi_path = root / "pixi.toml"

    _write_library_properties_version(lib_path, version)
    _write_readme_org_version(readme_path, version)
    _write_pixi_toml_version(pixi_path, version)

    print(f"Set version to {version}")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="version_sync.py")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="Verify all version numbers match.")

    sp_set = sub.add_parser("set", help="Set the version in all tracked metadata files.")
    sp_set.add_argument("version", help="Version to set (e.g. 11.0.1)")

    args = parser.parse_args(argv)

    if args.cmd == "check":
        return cmd_check()
    if args.cmd == "set":
        return cmd_set(args.version)

    parser.error("Unknown command")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
