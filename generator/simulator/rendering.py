# -*- coding: utf-8 -*-
from __future__ import annotations

"""仿真可视化辅助：车道线、车辆姿态、GT overlay 信息。"""

from typing import Dict, List, Tuple

import cv2
import numpy as np

from .models import TrafficScene, Vehicle
from .utils import mps_to_kmh


def draw_lane_guides(frame: np.ndarray, scene: TrafficScene) -> None:
    for lane in scene.lanes:
        color = (80, 120, 255) if lane.direction == +1 else (255, 120, 80)
        cv2.line(
            frame,
            (int(round(lane.x_px)), int(scene.top_y_px)),
            (int(round(lane.x_px)), int(scene.bottom_y_px)),
            color,
            1,
            cv2.LINE_AA,
        )


def update_pose(v: Vehicle, scene: TrafficScene) -> None:
    v.x_px = scene.lane_x(v.lane) + v.lane_offset_norm * scene.W
    v.y_px = scene.s_to_y(v.lane, v.s_m)
    v.heading_deg = 90.0 if v.direction == +1 else 270.0


def draw_overlay_info(
    overlay: np.ndarray,
    trails: Dict[int, List[Tuple[int, int]]],
    v: Vehicle,
    corners: np.ndarray,
    bbox: Tuple[float, float, float, float],
) -> None:
    if len(trails.get(v.vid, [])) >= 2:
        cv2.polylines(overlay, [np.array(trails[v.vid], dtype=np.int32)], False, v.color, 2, cv2.LINE_AA)

    cv2.polylines(overlay, [corners.astype(np.int32)], True, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.circle(overlay, (int(round(v.x_px)), int(round(v.y_px))), 3, v.color, -1, cv2.LINE_AA)

    x, y, _, _ = bbox
    direction_text = "D" if v.direction == +1 else "U"
    label = f"ID:{v.vid} {direction_text} {mps_to_kmh(v.v_mps):.1f}km/h {v.action}"
    cv2.putText(
        overlay,
        label,
        (int(round(x)), max(16, int(round(y)) - 5)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        v.color,
        1,
        cv2.LINE_AA,
    )
