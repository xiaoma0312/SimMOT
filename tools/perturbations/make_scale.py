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

DEFAULT_AMPLITUDES = [0.05, 0.10, 0.20]


def discover_sequences(data_root: Path) -> list[str]:
    seqs = []
    for p in sorted(data_root.iterdir()):
        if p.is_dir() and (p / "det" / "det.txt").is_file() and (p / "seqinfo.ini").is_file():
            seqs.append(p.name)
    return seqs


def stable_seed(base_seed: int, seq_name: str, tag: str) -> int:
    # Historical SimMOT seed-stability rule.
    # This reproduces the released seed21 / seed42 perturbation inputs.
    key = f"{base_seed}_{seq_name}_scale_{tag}_simmot_det_seed_folder"
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


def level_tag(amplitude: float) -> str:
    return f"{int(round(amplitude * 100)):02d}"


def make_one(
    data: np.ndarray,
    seq_name: str,
    amplitude: float,
    width: int,
    height: int,
    base_seed: int,
) -> tuple[np.ndarray, int]:
    tag = level_tag(amplitude)
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
    # s ~ Uniform(1-a, 1+a), and the same s scales both w and h.
    scale = rng.uniform(1.0 - amplitude, 1.0 + amplitude, size=len(out))
    new_w = w * scale
    new_h = h * scale

    x1 = cx - new_w / 2.0
    y1 = cy - new_h / 2.0
    x2 = cx + new_w / 2.0
    y2 = cy + new_h / 2.0

    # Intended center is preserved before clipping. Boxes that cross the image
    # boundary are clipped, so their realized center/scale may change slightly.
    x1 = np.clip(x1, 0.0, width - 1.0)
    y1 = np.clip(y1, 0.0, height - 1.0)
    x2 = np.clip(x2, 1.0, width)
    y2 = np.clip(y2, 1.0, height)

    new_w = np.maximum(1.0, x2 - x1)
    new_h = np.maximum(1.0, y2 - y1)

    out[:, 2] = x1
    out[:, 3] = y1
    out[:, 4] = new_w
    out[:, 5] = new_h
    out[:, 0] = out[:, 0].astype(int)
    out[:, 1] = out[:, 1].astype(int)
    out[:, 7:10] = -1
    return out, seed


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate SimMOT synchronized random bbox scale variants."
    )
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--sequences", nargs="*", default=None)
    parser.add_argument("--amplitudes", nargs="+", type=float, default=DEFAULT_AMPLITUDES)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    seqs = args.sequences or discover_sequences(args.data_root)
    if not seqs:
        raise RuntimeError("No valid sequences found.")

    for amp in args.amplitudes:
        if not 0.0 <= amp < 1.0:
            raise ValueError(f"Invalid scale amplitude: {amp}")

    for seq_name in seqs:
        seq_dir = args.data_root / seq_name
        src = seq_dir / "det" / "det.txt"
        data = load_det(src)
        width, height = read_image_size(seq_dir)

        for amp in args.amplitudes:
            tag = level_tag(amp)
            out, seed = make_one(data, seq_name, amp, width, height, args.seed)
            dst = args.output_root / seq_name / "det" / f"det_scale_{tag}.txt"
            save_det(dst, out, args.overwrite)
            print(
                f"[OK] {seq_name} scale_{tag}: "
                f"range=[{1-amp:.2f},{1+amp:.2f}], seed={seed}, rows={len(out)}"
            )


if __name__ == "__main__":
    main()
