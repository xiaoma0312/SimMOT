#!/usr/bin/env python3
"""Generate a deterministic SHA-256 manifest for a SimMOT release tree."""

from __future__ import annotations

import argparse
import csv
import hashlib
from pathlib import Path


MANIFEST_NAME = "SHA256SUMS.csv"
CHUNK_SIZE = 8 * 1024 * 1024


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(CHUNK_SIZE):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--release-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="SimMOT_v1.0 directory (defaults to the parent of tools/).",
    )
    args = parser.parse_args()
    release = args.release_root.resolve()
    manifest = release / MANIFEST_NAME
    files = sorted(
        path
        for path in release.rglob("*")
        if path.is_file() and path != manifest
    )
    with manifest.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["sha256", "size_bytes", "relative_path"])
        for index, path in enumerate(files, start=1):
            writer.writerow([sha256(path), path.stat().st_size, path.relative_to(release).as_posix()])
            if index % 1000 == 0:
                print(f"hashed {index}/{len(files)} files", flush=True)
    print(f"wrote {manifest} with {len(files)} entries")


if __name__ == "__main__":
    main()
