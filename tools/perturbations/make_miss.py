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
import hashlib
from pathlib import Path
import numpy as np

DEFAULT_RATES = [0.05, 0.10, 0.20, 0.30]


def discover_sequences(data_root: Path) -> list[str]:
    seqs = []
    for p in sorted(data_root.iterdir()):
        if p.is_dir() and (p / "det" / "det.txt").is_file():
            seqs.append(p.name)
    return seqs


def stable_seed(base_seed: int, seq_name: str, tag: str) -> int:
    # Historical SimMOT seed-stability rule.
    # This reproduces the released seed21 / seed42 perturbation inputs.
    key = f"{base_seed}_{seq_name}_miss_{tag}_simmot_det_seed_folder"
    digest = hashlib.md5(key.encode("utf-8")).hexdigest()
    return int(digest[:8], 16)


def load_det(path: Path) -> np.ndarray:
    data = np.loadtxt(path, delimiter=",")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 10:
        raise ValueError(f"{path} must contain at least 10 MOT-style columns.")
    if not np.isfinite(data[:, :7]).all():
        raise ValueError(f"{path} contains non-finite values.")
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


def level_tag(rate: float) -> str:
    return f"{int(round(rate * 100)):02d}"


def make_one(data: np.ndarray, seq_name: str, rate: float, base_seed: int) -> tuple[np.ndarray, int]:
    tag = level_tag(rate)
    seed = stable_seed(base_seed, seq_name, tag)
    rng = np.random.default_rng(seed)

    keep_mask = rng.random(len(data)) >= rate
    out = data[keep_mask].copy()

    out[:, 0] = out[:, 0].astype(int)
    out[:, 1] = out[:, 1].astype(int)
    out[:, 7:10] = -1
    return out, seed


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate SimMOT missed-detection variants by independent row deletion."
    )
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--sequences", nargs="*", default=None,
                        help="Sequence names. Omit to auto-discover all sequences.")
    parser.add_argument("--rates", nargs="+", type=float, default=DEFAULT_RATES)
    parser.add_argument("--seed", type=int, default=2026,
                        help="Base seed for deterministic public generation.")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    seqs = args.sequences or discover_sequences(args.data_root)
    if not seqs:
        raise RuntimeError("No valid sequences found.")

    for rate in args.rates:
        if not 0.0 <= rate < 1.0:
            raise ValueError(f"Invalid miss rate: {rate}")

    for seq_name in seqs:
        src = args.data_root / seq_name / "det" / "det.txt"
        if not src.is_file():
            raise FileNotFoundError(src)

        data = load_det(src)

        for rate in args.rates:
            tag = level_tag(rate)
            out, seed = make_one(data, seq_name, rate, args.seed)
            dst = args.output_root / seq_name / "det" / f"det_miss_{tag}.txt"
            save_det(dst, out, args.overwrite)
            actual = 1.0 - len(out) / len(data)
            print(
                f"[OK] {seq_name} miss_{tag}: "
                f"requested={rate:.4f}, actual={actual:.4f}, "
                f"seed={seed}, rows={len(out)}"
            )


if __name__ == "__main__":
    main()
