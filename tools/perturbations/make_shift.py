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
from pathlib import Path
import numpy as np

DEFAULT_LEVELS = {
    "x05": (5.0, 0.0),
    "y05": (0.0, 5.0),
    "xy05": (5.0, 5.0),
    "xy10": (10.0, 10.0),
}


def discover_sequences(data_root: Path) -> list[str]:
    seqs = []
    for p in sorted(data_root.iterdir()):
        if p.is_dir() and (p / "det" / "det.txt").is_file() and (p / "seqinfo.ini").is_file():
            seqs.append(p.name)
    return seqs


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


def make_one(data: np.ndarray, dx: float, dy: float, width: int, height: int) -> np.ndarray:
    out = data.copy()
    x = out[:, 2].astype(float)
    y = out[:, 3].astype(float)
    w = out[:, 4].astype(float)
    h = out[:, 5].astype(float)

    x_new = x + dx
    y_new = y + dy

    # Apply the requested deterministic offset, then constrain the full box
    # to the image. Near an image boundary, the realized offset may therefore
    # be smaller than (dx, dy).
    x_new = np.clip(x_new, 0.0, np.maximum(0.0, width - w))
    y_new = np.clip(y_new, 0.0, np.maximum(0.0, height - h))

    out[:, 2] = x_new
    out[:, 3] = y_new
    out[:, 4] = w
    out[:, 5] = h
    out[:, 0] = out[:, 0].astype(int)
    out[:, 1] = out[:, 1].astype(int)
    out[:, 7:10] = -1
    return out


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate SimMOT deterministic fixed-position shift variants."
    )
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--sequences", nargs="*", default=None)
    parser.add_argument(
        "--levels",
        nargs="*",
        choices=sorted(DEFAULT_LEVELS),
        default=list(DEFAULT_LEVELS),
        help="Subset of standard shift levels."
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    seqs = args.sequences or discover_sequences(args.data_root)
    if not seqs:
        raise RuntimeError("No valid sequences found.")

    for seq_name in seqs:
        seq_dir = args.data_root / seq_name
        src = seq_dir / "det" / "det.txt"
        data = load_det(src)
        width, height = read_image_size(seq_dir)

        for tag in args.levels:
            dx, dy = DEFAULT_LEVELS[tag]
            out = make_one(data, dx, dy, width, height)
            dst = args.output_root / seq_name / "det" / f"det_shift_{tag}.txt"
            save_det(dst, out, args.overwrite)
            print(f"[OK] {seq_name} shift_{tag}: dx={dx}px, dy={dy}px, rows={len(out)}")


if __name__ == "__main__":
    main()
