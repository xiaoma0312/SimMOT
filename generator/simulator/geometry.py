# -*- coding: utf-8 -*-
from __future__ import annotations

"""几何与绘制基础：RGBA 缩放/旋转、透明融合、bbox 计算。"""

import math
from typing import Tuple

import cv2
import numpy as np


def resize_rgba(rgba: np.ndarray, width: int, height: int) -> np.ndarray:
    width = max(2, int(round(width)))
    height = max(2, int(round(height)))
    return cv2.resize(rgba, (width, height), interpolation=cv2.INTER_CUBIC)


def rotate_rgba(rgba: np.ndarray, angle_deg: float) -> np.ndarray:
    h, w = rgba.shape[:2]
    center = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    cos = abs(M[0, 0])
    sin = abs(M[0, 1])
    new_w = max(2, int(h * sin + w * cos))
    new_h = max(2, int(h * cos + w * sin))
    M[0, 2] += new_w / 2.0 - center[0]
    M[1, 2] += new_h / 2.0 - center[1]
    return cv2.warpAffine(
        rgba, M, (new_w, new_h),
        flags=cv2.INTER_CUBIC,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0, 0),
    )


def alpha_blend(dst: np.ndarray, src_rgba: np.ndarray, center: Tuple[float, float]) -> None:
    h, w = dst.shape[:2]
    sh, sw = src_rgba.shape[:2]
    cx, cy = center
    x1 = int(round(cx - sw / 2))
    y1 = int(round(cy - sh / 2))
    x2 = x1 + sw
    y2 = y1 + sh

    ix1, iy1 = max(0, x1), max(0, y1)
    ix2, iy2 = min(w, x2), min(h, y2)
    if ix1 >= ix2 or iy1 >= iy2:
        return

    sx1, sy1 = ix1 - x1, iy1 - y1
    sx2, sy2 = sx1 + (ix2 - ix1), sy1 + (iy2 - iy1)

    crop = src_rgba[sy1:sy2, sx1:sx2]
    alpha = crop[:, :, 3:4].astype(np.float32) / 255.0
    src_rgb = crop[:, :, :3].astype(np.float32)
    roi = dst[iy1:iy2, ix1:ix2].astype(np.float32)
    dst[iy1:iy2, ix1:ix2] = (src_rgb * alpha + roi * (1.0 - alpha)).astype(np.uint8)


def alpha_blend_shadow(dst: np.ndarray, src_rgba: np.ndarray, center: Tuple[float, float]) -> None:
    shadow = src_rgba.copy()
    shadow[:, :, :3] = 0
    shadow[:, :, 3] = (shadow[:, :, 3].astype(np.float32) * 0.18).astype(np.uint8)
    shadow[:, :, 3] = cv2.GaussianBlur(shadow[:, :, 3], (0, 0), 2.5)
    alpha_blend(dst, shadow, (center[0] + 3, center[1] + 5))


def oriented_box_corners(cx: float, cy: float, w_px: float, h_px: float, angle_deg: float) -> np.ndarray:
    theta = math.radians(angle_deg)
    long_v = np.array([math.cos(theta), math.sin(theta)], dtype=np.float32)
    short_v = np.array([-math.sin(theta), math.cos(theta)], dtype=np.float32)
    return np.array([
        [cx - short_v[0] * w_px / 2 - long_v[0] * h_px / 2, cy - short_v[1] * w_px / 2 - long_v[1] * h_px / 2],
        [cx + short_v[0] * w_px / 2 - long_v[0] * h_px / 2, cy + short_v[1] * w_px / 2 - long_v[1] * h_px / 2],
        [cx + short_v[0] * w_px / 2 + long_v[0] * h_px / 2, cy + short_v[1] * w_px / 2 + long_v[1] * h_px / 2],
        [cx - short_v[0] * w_px / 2 + long_v[0] * h_px / 2, cy - short_v[1] * w_px / 2 + long_v[1] * h_px / 2],
    ], dtype=np.float32)


def bbox_from_corners(corners: np.ndarray) -> Tuple[float, float, float, float]:
    x1 = float(corners[:, 0].min())
    y1 = float(corners[:, 1].min())
    x2 = float(corners[:, 0].max())
    y2 = float(corners[:, 1].max())
    return x1, y1, x2 - x1, y2 - y1
