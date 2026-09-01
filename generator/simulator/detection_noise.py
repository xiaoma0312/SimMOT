# -*- coding: utf-8 -*-
from __future__ import annotations

"""Detection-noise injection for MOT experiments.

This module perturbs ideal labels into detector-like inputs. It is intentionally
independent from vehicle dynamics: the simulator first generates high-confidence
labels, then this module creates controlled detection errors such as missed
boxes, bbox jitter, center shift, scale noise and false positives.
"""

from dataclasses import dataclass
import math
import random
from typing import Iterable, Optional, Tuple


def _clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class DetectionNoiseConfig:
    name: str = "noise_none"
    enabled: bool = False
    miss_rate: float = 0.0
    bbox_jitter_std_px: float = 0.0
    center_shift_px: Tuple[float, float] = (0.0, 0.0)
    scale_noise_std: float = 0.0
    false_positive_rate_per_frame: float = 0.0
    false_positive_size_px: Tuple[float, float] = (25.0, 80.0)
    confidence_mean: float = 1.0
    confidence_std: float = 0.0
    min_box_size_px: float = 2.0

    @classmethod
    def from_dict(cls, data: Optional[dict]) -> "DetectionNoiseConfig":
        if not data:
            return cls()
        center_shift = data.get("center_shift_px", (0.0, 0.0))
        fp_size = data.get("false_positive_size_px", (25.0, 80.0))
        return cls(
            name=str(data.get("name", "noise_custom")),
            enabled=bool(data.get("enabled", False)),
            miss_rate=float(data.get("miss_rate", 0.0)),
            bbox_jitter_std_px=float(data.get("bbox_jitter_std_px", 0.0)),
            center_shift_px=(float(center_shift[0]), float(center_shift[1])),
            scale_noise_std=float(data.get("scale_noise_std", 0.0)),
            false_positive_rate_per_frame=float(data.get("false_positive_rate_per_frame", 0.0)),
            false_positive_size_px=(float(fp_size[0]), float(fp_size[1])),
            confidence_mean=float(data.get("confidence_mean", 1.0)),
            confidence_std=float(data.get("confidence_std", 0.0)),
            min_box_size_px=float(data.get("min_box_size_px", 2.0)),
        )


def should_write_noisy_det(noise: DetectionNoiseConfig) -> bool:
    """Always write det_noisy.txt for reproducibility, even for noise_none."""
    return True


def perturb_bbox(
    bbox: Tuple[float, float, float, float],
    image_width: int,
    image_height: int,
    rng: random.Random,
    noise: DetectionNoiseConfig,
) -> Optional[Tuple[float, float, float, float, float]]:
    """Return perturbed bbox as (x, y, w, h, conf), or None for missed detection."""
    x, y, w, h = bbox

    if noise.enabled and rng.random() < _clip(noise.miss_rate, 0.0, 1.0):
        return None

    conf = noise.confidence_mean
    if noise.enabled and noise.confidence_std > 0:
        conf += rng.gauss(0.0, noise.confidence_std)
    conf = _clip(conf, 0.01, 1.0)

    if not noise.enabled:
        return x, y, w, h, conf

    cx = x + 0.5 * w
    cy = y + 0.5 * h

    if noise.bbox_jitter_std_px > 0:
        cx += rng.gauss(0.0, noise.bbox_jitter_std_px)
        cy += rng.gauss(0.0, noise.bbox_jitter_std_px)
        w += rng.gauss(0.0, noise.bbox_jitter_std_px)
        h += rng.gauss(0.0, noise.bbox_jitter_std_px)

    cx += noise.center_shift_px[0]
    cy += noise.center_shift_px[1]

    if noise.scale_noise_std > 0:
        # Log-normal keeps scale positive and makes perturbation symmetric in multiplicative space.
        scale = math.exp(rng.gauss(0.0, noise.scale_noise_std))
        w *= scale
        h *= scale

    w = max(noise.min_box_size_px, w)
    h = max(noise.min_box_size_px, h)
    x = cx - 0.5 * w
    y = cy - 0.5 * h

    # Keep boxes within image bounds to avoid invalid MOT inputs.
    x = _clip(x, 0.0, max(0.0, image_width - noise.min_box_size_px))
    y = _clip(y, 0.0, max(0.0, image_height - noise.min_box_size_px))
    w = _clip(w, noise.min_box_size_px, max(noise.min_box_size_px, image_width - x))
    h = _clip(h, noise.min_box_size_px, max(noise.min_box_size_px, image_height - y))
    return x, y, w, h, conf


def sample_false_positives(
    frame_idx: int,
    image_width: int,
    image_height: int,
    rng: random.Random,
    noise: DetectionNoiseConfig,
) -> Iterable[Tuple[int, float, float, float, float, float]]:
    """Yield MOT-format false-positive boxes for one frame.

    Returns tuples: (frame_idx, x, y, w, h, conf). The count is drawn from a
    Bernoulli/Poisson-like process; for small rates it is usually 0 or 1.
    """
    if not noise.enabled or noise.false_positive_rate_per_frame <= 0:
        return []

    rate = noise.false_positive_rate_per_frame
    count = int(rate)
    if rng.random() < (rate - count):
        count += 1

    out = []
    min_size, max_size = noise.false_positive_size_px
    for _ in range(count):
        w = rng.uniform(min_size, max_size)
        h = rng.uniform(min_size, max_size)
        x = rng.uniform(0.0, max(0.0, image_width - w))
        y = rng.uniform(0.0, max(0.0, image_height - h))
        conf = _clip(rng.gauss(noise.confidence_mean, max(0.01, noise.confidence_std)), 0.01, 1.0)
        out.append((frame_idx, x, y, w, h, conf))
    return out
