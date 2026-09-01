# -*- coding: utf-8 -*-
from __future__ import annotations

"""核心数据结构：车道、车辆、场景坐标系统。"""

from dataclasses import dataclass
from typing import List, Tuple

from .config import VEHICLE_TYPES


@dataclass
class Lane:
    lid: int
    x_px: float
    direction: int  # +1: top -> bottom, -1: bottom -> top
    name: str


@dataclass
class Vehicle:
    vid: int
    vehicle_type: str
    lane: int
    direction: int
    s_m: float
    v_mps: float
    desired_speed_mps: float
    target_free_speed_mps: float
    free_speed_floor_mps: float
    color: Tuple[int, int, int]
    sprite_index: int
    lane_offset_norm: float

    # Active vehicle-level behavior parameters, loaded from vehicle_types_default.json when provided.
    max_acc_mps2: float
    comfort_dec_mps2: float
    max_dec_mps2: float
    min_gap_m: float
    time_headway_sec: float
    sigma: float

    next_decision_time: float
    action: str = "spawned"
    active: bool = True

    acc_mps2: float = 0.0
    smoothed_acc_mps2: float = 0.0

    x_px: float = 0.0
    y_px: float = 0.0
    heading_deg: float = 90.0

    @property
    def length_m(self) -> float:
        return VEHICLE_TYPES[self.vehicle_type]["length_m"]

    @property
    def width_m(self) -> float:
        return VEHICLE_TYPES[self.vehicle_type]["width_m"]


class TrafficScene:
    def __init__(
        self,
        width: int,
        height: int,
        meters_per_pixel: float,
        vehicle_mpp: float,
        down_lanes_norm: List[float],
        up_lanes_norm: List[float],
        top_y_norm: float,
        bottom_y_norm: float,
        fps: int,
    ):
        self.W = int(width)
        self.H = int(height)
        self.meters_per_pixel = float(meters_per_pixel)
        self.vehicle_mpp = float(vehicle_mpp)

        self.top_y_px = float(top_y_norm) * height
        self.bottom_y_px = float(bottom_y_norm) * height
        self.path_len_px = max(1.0, self.bottom_y_px - self.top_y_px)
        self.visible_length_m = self.path_len_px * self.meters_per_pixel
        self.fps = int(fps)
        self.dt = 1.0 / max(1, fps)

        self.lanes: List[Lane] = []
        lid = 0
        for x in down_lanes_norm:
            self.lanes.append(Lane(lid=lid, x_px=float(x) * width, direction=+1, name=f"down_{lid}"))
            lid += 1
        for x in up_lanes_norm:
            self.lanes.append(Lane(lid=lid, x_px=float(x) * width, direction=-1, name=f"up_{lid}"))
            lid += 1

    @property
    def num_lanes(self) -> int:
        return len(self.lanes)

    def lane(self, lane_id: int) -> Lane:
        lane_id = int(max(0, min(self.num_lanes - 1, lane_id)))
        return self.lanes[lane_id]

    def lane_x(self, lane_id: int) -> float:
        return self.lane(lane_id).x_px

    def lane_direction(self, lane_id: int) -> int:
        return self.lane(lane_id).direction

    def s_to_y(self, lane_id: int, s_m: float) -> float:
        lane = self.lane(lane_id)
        if lane.direction == +1:
            return self.top_y_px + s_m / self.meters_per_pixel
        return self.bottom_y_px - s_m / self.meters_per_pixel

    def visible_s_range(self) -> Tuple[float, float]:
        return -40.0, self.visible_length_m + 40.0
