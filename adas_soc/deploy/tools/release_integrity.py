#!/usr/bin/env python3
"""Create and verify SHA-256 manifests for ADAS release directories."""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import os
import platform
import sys
from pathlib import Path
from typing import Iterable


SCHEMA = "adas-release-manifest-v1"
DEFAULT_PROTOCOL_VERSION = 3


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def iter_release_files(root: Path, manifest_path: Path) -> Iterable[Path]:
    manifest_resolved = manifest_path.resolve()
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if path.resolve() == manifest_resolved:
            continue
        if "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        yield path


def create_manifest(
    root: Path,
    manifest_path: Path,
    protocol_version: int,
    git_commit: str,
) -> dict:
    root = root.resolve()
    manifest_path = manifest_path.resolve()
    files = []
    for path in iter_release_files(root, manifest_path):
        rel = path.relative_to(root).as_posix()
        stat = path.stat()
        files.append(
            {
                "path": rel,
                "size": stat.st_size,
                "sha256": sha256_file(path),
            }
        )
    manifest = {
        "schema": SCHEMA,
        "created_at_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(),
        "protocol_version": protocol_version,
        "git_commit": git_commit,
        "host": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
        "files": files,
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with manifest_path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return manifest


def load_manifest(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def verify_manifest(
    root: Path,
    manifest_path: Path,
    protocol_version: int | None,
) -> list[str]:
    root = root.resolve()
    manifest = load_manifest(manifest_path)
    errors: list[str] = []
    if manifest.get("schema") != SCHEMA:
        errors.append(f"schema mismatch: {manifest.get('schema')!r}")
    if protocol_version is not None and manifest.get("protocol_version") != protocol_version:
        errors.append(
            "protocol version mismatch: "
            f"manifest={manifest.get('protocol_version')!r} expected={protocol_version}"
        )
    seen: set[str] = set()
    for entry in manifest.get("files", []):
        rel = entry.get("path")
        seen.add(rel)
        path = root / rel
        if not path.is_file():
            errors.append(f"missing file: {rel}")
            continue
        size = path.stat().st_size
        if size != entry.get("size"):
            errors.append(f"size mismatch: {rel} manifest={entry.get('size')} actual={size}")
        digest = sha256_file(path)
        if digest != entry.get("sha256"):
            errors.append(f"sha256 mismatch: {rel}")
    if not seen:
        errors.append("manifest contains no files")
    return errors


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    create = sub.add_parser("create", help="write a manifest for a release root")
    create.add_argument("root", type=Path)
    create.add_argument("manifest", type=Path)
    create.add_argument("--protocol-version", type=int, default=DEFAULT_PROTOCOL_VERSION)
    create.add_argument("--git-commit", default="unknown")

    verify = sub.add_parser("verify", help="verify a release root against a manifest")
    verify.add_argument("root", type=Path)
    verify.add_argument("manifest", type=Path)
    verify.add_argument("--protocol-version", type=int)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "create":
        manifest = create_manifest(
            args.root,
            args.manifest,
            args.protocol_version,
            args.git_commit,
        )
        print(
            f"wrote {args.manifest} "
            f"({len(manifest['files'])} files, protocol v{manifest['protocol_version']})"
        )
        return 0
    if args.command == "verify":
        errors = verify_manifest(args.root, args.manifest, args.protocol_version)
        if errors:
            for error in errors:
                print(f"ERROR: {error}", file=sys.stderr)
            return 1
        print(f"verified {args.manifest}")
        return 0
    raise AssertionError(f"unknown command {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
