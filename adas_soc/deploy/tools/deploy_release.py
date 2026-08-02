#!/usr/bin/env python3
"""Activate, inspect, and roll back ADAS release symlinks."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import release_integrity


DEFAULT_PROTOCOL_VERSION = 3


def current_target(path: Path) -> Path | None:
    if not path.exists() and not path.is_symlink():
        return None
    return path.resolve()


def replace_symlink(link: Path, target: Path) -> None:
    tmp = link.with_name(f".{link.name}.tmp.{os.getpid()}")
    if tmp.exists() or tmp.is_symlink():
        tmp.unlink()
    os.symlink(target, tmp, target_is_directory=True)
    os.replace(tmp, link)


def verify_release(release: Path, protocol_version: int) -> None:
    manifest = release / "release_manifest.json"
    errors = release_integrity.verify_manifest(release, manifest, protocol_version)
    if errors:
        joined = "\n".join(f"  - {error}" for error in errors)
        raise RuntimeError(f"release verification failed:\n{joined}")


def maybe_start_service(service: str, enabled: bool) -> None:
    if not enabled:
        return
    subprocess.run(["systemctl", "restart", service], check=True)


def activate(args: argparse.Namespace) -> int:
    root = args.root.resolve()
    release = (root / "releases" / args.version).resolve()
    if not release.is_dir():
        raise RuntimeError(f"release does not exist: {release}")
    verify_release(release, args.protocol_version)

    current = root / "current"
    previous = root / "previous"
    old_current = current_target(current)
    if old_current is not None and old_current != release:
        replace_symlink(previous, old_current)
    replace_symlink(current, release)
    print(f"activated {release} -> {current}")

    try:
        maybe_start_service(args.service, args.start_service)
    except subprocess.CalledProcessError as exc:
        if old_current is not None:
            replace_symlink(current, old_current)
            print(f"service start failed; rolled back current to {old_current}", file=sys.stderr)
        raise RuntimeError(f"systemctl restart {args.service} failed") from exc
    if not args.start_service:
        print("service not started; operator must start HIL (session auto-arms once CAN is fresh)")
    return 0


def rollback(args: argparse.Namespace) -> int:
    root = args.root.resolve()
    current = root / "current"
    previous = root / "previous"
    target = current_target(previous)
    if target is None:
        raise RuntimeError("no previous release symlink found")
    verify_release(target, args.protocol_version)
    old_current = current_target(current)
    replace_symlink(current, target)
    if old_current is not None:
        replace_symlink(previous, old_current)
    print(f"rolled back current to {target}")
    maybe_start_service(args.service, args.start_service)
    return 0


def status(args: argparse.Namespace) -> int:
    root = args.root.resolve()
    for name in ("current", "previous"):
        target = current_target(root / name)
        print(f"{name}: {target if target else '(none)'}")
        if not target:
            continue
        manifest_path = target / "release_manifest.json"
        if manifest_path.is_file():
            with manifest_path.open("r", encoding="utf-8") as handle:
                manifest = json.load(handle)
            print(f"  protocol_version: {manifest.get('protocol_version')}")
            print(f"  git_commit: {manifest.get('git_commit')}")
            print(f"  created_at_utc: {manifest.get('created_at_utc')}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("/opt/adas"))
    sub = parser.add_subparsers(dest="command", required=True)

    activate_parser = sub.add_parser("activate")
    activate_parser.add_argument("version")
    activate_parser.add_argument("--protocol-version", type=int, default=DEFAULT_PROTOCOL_VERSION)
    activate_parser.add_argument("--start-service", action="store_true")
    activate_parser.add_argument("--service", default="adas-hil.service")
    activate_parser.set_defaults(func=activate)

    rollback_parser = sub.add_parser("rollback")
    rollback_parser.add_argument("--protocol-version", type=int, default=DEFAULT_PROTOCOL_VERSION)
    rollback_parser.add_argument("--start-service", action="store_true")
    rollback_parser.add_argument("--service", default="adas-hil.service")
    rollback_parser.set_defaults(func=rollback)

    status_parser = sub.add_parser("status")
    status_parser.set_defaults(func=status)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except Exception as exc:  # noqa: BLE001 - CLI should print concise failures.
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
