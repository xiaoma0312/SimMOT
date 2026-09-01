# -*- coding: utf-8 -*-
from __future__ import annotations

"""背景图和车辆贴图读取、裁剪与兜底生成。"""

import os
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .config import COLORS_BGR, SPRITE_FILE_CANDIDATES, VEHICLE_TYPES
from .utils import safe_path


def make_background_if_missing(width: int = 1280, height: int = 720) -> np.ndarray:
    bg = np.full((height, width, 3), 42, dtype=np.uint8)
    cv2.rectangle(bg, (int(width * 0.25), 0), (int(width * 0.58), height), (58, 58, 58), -1)
    cv2.rectangle(bg, (int(width * 0.63), 0), (int(width * 0.92), height), (58, 58, 58), -1)
    cv2.rectangle(bg, (int(width * 0.58), 0), (int(width * 0.63), height), (40, 65, 40), -1)
    return bg


def trim_alpha(rgba: np.ndarray, min_alpha: int = 5) -> np.ndarray:
    alpha = rgba[:, :, 3]
    ys, xs = np.where(alpha > min_alpha)
    if len(xs) == 0:
        return rgba
    return rgba[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def make_rect_sprite(vehicle_type: str, scale: int = 80) -> np.ndarray:
    info = VEHICLE_TYPES[vehicle_type]
    length = max(32, int(scale * info["length_m"] / 5.0))
    width = max(16, int(scale * info["width_m"] / 1.8))
    rgba = np.zeros((length, width, 4), dtype=np.uint8)
    color = COLORS_BGR[vehicle_type]
    rgba[:, :, :3] = color
    rgba[:, :, 3] = 255
    cv2.rectangle(rgba, (2, length - max(8, length // 5)), (width - 3, length - 3), (255, 255, 255, 255), -1)
    cv2.rectangle(rgba, (0, 0), (width - 1, length - 1), (30, 30, 30, 255), 2)
    return rgba


def _load_one_sprite(path: str) -> Optional[np.ndarray]:
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    if img.ndim != 3:
        raise ValueError(f"Sprite PNG must have 3 or 4 channels: {path}")
    if img.shape[2] == 3:
        alpha = np.full((img.shape[0], img.shape[1], 1), 255, dtype=np.uint8)
        img = np.concatenate([img, alpha], axis=2)
    if img.shape[2] != 4:
        raise ValueError(f"Unexpected sprite channel count: {path}")
    return trim_alpha(img)


def load_sprites(base_dir: str, vehicle_type: str) -> Tuple[List[np.ndarray], List[str]]:
    """
    Load all available sprites for one vehicle type.

    Supported layout:
    - <asset-dir>/passenger car/*.png  （你现在的小汽车文件夹）
    - <asset-dir>/bus.png, taxi.png, other.png, large bus.png 等根目录贴图
    """
    sprites: List[np.ndarray] = []
    sources: List[str] = []

    folder = os.path.join(base_dir, vehicle_type)
    if os.path.isdir(folder):
        filenames = sorted([
            fn for fn in os.listdir(folder)
            if fn.lower().endswith('.png') and not fn.startswith('.')
        ])
        for fn in filenames:
            path = os.path.join(folder, fn)
            img = _load_one_sprite(path)
            if img is not None:
                sprites.append(img)
                sources.append(path)

    for filename in SPRITE_FILE_CANDIDATES[vehicle_type]:
        path = safe_path(base_dir, filename)
        if os.path.exists(path):
            img = _load_one_sprite(path)
            if img is not None:
                sprites.append(img)
                sources.append(path)

    if not sprites:
        return [make_rect_sprite(vehicle_type)], [f"fallback_rect_sprite:{vehicle_type}"]

    return sprites, sources
