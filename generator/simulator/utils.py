# -*- coding: utf-8 -*-
from __future__ import annotations

"""通用工具函数：路径、单位转换、随机选择、数值裁剪。"""

import os
import random
from typing import Iterable, List, Union

from .config import VEHICLE_TYPE_WEIGHTS


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def safe_path(base_dir: str, name: str) -> str:
    return name if os.path.isabs(name) else os.path.join(base_dir, name)


def kmh_to_mps(v_kmh: float) -> float:
    return float(v_kmh) / 3.6


def mps_to_kmh(v_mps: float) -> float:
    return float(v_mps) * 3.6


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def parse_norm_list(s: Union[str, Iterable[float]]) -> List[float]:
    """Parse lane-position lists from either JSON arrays or comma-separated CLI strings."""
    if isinstance(s, str):
        return [float(v.strip()) for v in s.split(",") if v.strip()]
    return [float(v) for v in s]


def choose_weighted_vehicle_type(rng: random.Random) -> str:
    r = rng.random()
    acc = 0.0
    for name, w in VEHICLE_TYPE_WEIGHTS:
        acc += w
        if r <= acc:
            return name
    return VEHICLE_TYPE_WEIGHTS[-1][0]
