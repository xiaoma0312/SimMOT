"""
Public SimMOT perturbation utility.

This cleaned release script preserves the perturbation definition used in the
audited SimMOT experiments while removing machine-specific paths.

Input:
    <data_root>/<sequence>/det/det.txt
    <data_root>/<sequence>/seqinfo.ini   (when image size is required)

Output:
    <output_root>/<sequence>/det/<variant>.txt

The clean det.txt is never modified.
"""

from __future__ import annotations

import argparse
import configparser
import hashlib
from pathlib import Path
import numpy as np

DEFAULT_SIGMAS = [1.0, 3.0, 5.0, 8.0]


def discover_sequences(data_root: Path) -> list[str]:
    seqs = []
    for p in sorted(data_root.iterdir()):
        if p.is_dir() and (p / "det" / "det.txt").is_file() and (p / "seqinfo.ini").is_file():
            seqs.append(p.name)
    return seqs


def stable_seed(base_seed: int, seq_name: str, tag: str) -> int:
    # Historical SimMOT seed-stability rule.
    # This reproduces the released seed21 / seed42 perturbation inputs.
    key = f"{base_seed}_{seq_name}_jitter_{tag}_simmot_det_seed_folder"
    digest = hashlib.md5(key.encode("utf-8")).hexdigest()
    return int(digest[:8], 16)


def read_image_size(seq_dir: Path) -> tuple[int, int]:
    cfg = configparser.ConfigParser()
    cfg.read(seq_dir / "seqinfo.ini")
    if "Sequence" not in cfg:
        raise ValueError(f"Missing [Sequence] section: {seq_dir / 'seqinfo.ini'}")
    return cfg.getint("Sequence", "imWidth"), cfg.getint("Sequence", "imHeight")


def load_det(path: Path) -> np.ndarray:
    data = np.loadtxt(path, delimiter=",")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 10:
        raise ValueError(f"{path} must contain at least 10 MOT-style columns.")
    return data


def save_det(path: Path, data: np.ndarray, overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"{path} already exists. Use --overwrite to replace it.")
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(
        path,
        data,
        delimiter=",",
        fmt=["%d", "%d", "%.3f", "%.3f", "%.3f", "%.3f",
             "%.6f", "%d", "%d", "%d"],
    )


def level_tag(sigma: float) -> str:
    if float(sigma).is_integer():
        return f"{int(sigma):02d}"
    return str(sigma).replace(".", "p")


def make_one(
    data: np.ndarray,
    seq_name: str,
    sigma_px: float,
    width: int,
    height: int,
    base_seed: int,
) -> tuple[np.ndarray, int]:
    tag = level_tag(sigma_px)
    seed = stable_seed(base_seed, seq_name, tag)
    rng = np.random.default_rng(seed)

    out = data.copy()
    x = out[:, 2].astype(float)
    y = out[:, 3].astype(float)
    w = out[:, 4].astype(float)
    h = out[:, 5].astype(float)

    cx = x + w / 2.0
    cy = y + h / 2.0

    # Paper perturbation definition:
    # center-only Gaussian jitter; width and height remain unchanged.
    cx_j = cx + rng.normal(0.0, sigma_px, size=len(out))
    cy_j = cy + rng.normal(0.0, sigma_px, size=len(out))

    x_j = cx_j - w / 2.0
    y_j = cy_j - h / 2.0

    # Keep the complete AABB inside the image while preserving w/h.
    x_j = np.clip(x_j, 0.0, np.maximum(0.0, width - w))
    y_j = np.clip(y_j, 0.0, np.maximum(0.0, height - h))

    out[:, 2] = x_j
    out[:, 3] = y_j
    out[:, 4] = w
    out[:, 5] = h
    out[:, 0] = out[:, 0].astype(int)
    out[:, 1] = out[:, 1].astype(int)
    out[:, 7:10] = -1
    return out, seed


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate SimMOT center-only Gaussian bbox jitter variants."
    )
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--sequences", nargs="*", default=None)
    parser.add_argument("--sigmas", nargs="+", type=float, default=DEFAULT_SIGMAS)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    seqs = args.sequences or discover_sequences(args.data_root)
    if not seqs:
        raise RuntimeError("No valid sequences found.")

    for sigma in args.sigmas:
        if sigma < 0:
            raise ValueError(f"Invalid jitter sigma: {sigma}")

    for seq_name in seqs:
        seq_dir = args.data_root / seq_name
        src = seq_dir / "det" / "det.txt"
        data = load_det(src)
        width, height = read_image_size(seq_dir)

        for sigma in args.sigmas:
            tag = level_tag(sigma)
            out, seed = make_one(data, seq_name, sigma, width, height, args.seed)
            dst = args.output_root / seq_name / "det" / f"det_jitter_{tag}px.txt"
            save_det(dst, out, args.overwrite)
            print(f"[OK] {seq_name} jitter_{tag}px: sigma={sigma}px, seed={seed}, rows={len(out)}")


if __name__ == "__main__":
    main()
