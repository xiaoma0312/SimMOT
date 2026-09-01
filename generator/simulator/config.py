# -*- coding: utf-8 -*-
"""车辆类型、贴图文件名、颜色等全局静态配置。

这个文件只放“不会随单帧状态改变”的配置；后续增加车辆类型或默认比例优先改这里。
"""

VEHICLE_TYPES = {
    "passenger car": {"length_m": 5.0, "width_m": 1.8, "desired_speed_range_kmh": (45.0, 68.0)},
    "taxi": {"length_m": 5.0, "width_m": 1.8, "desired_speed_range_kmh": (45.0, 70.0)},
    "other": {"length_m": 6.0, "width_m": 1.9, "desired_speed_range_kmh": (40.0, 62.0)},
    "bus": {"length_m": 12.0, "width_m": 2.55, "desired_speed_range_kmh": (32.0, 50.0)},
    "large bus": {"length_m": 12.5, "width_m": 2.55, "desired_speed_range_kmh": (30.0, 46.0)},
}

VEHICLE_TYPE_WEIGHTS = [
    ("passenger car", 0.56),
    ("taxi", 0.18),
    ("other", 0.16),
    ("bus", 0.07),
    ("large bus", 0.03),
]

SPRITE_FILE_CANDIDATES = {
    "passenger car": ["passenger car.png", "passenger_car.png", "car.png"],
    "taxi": ["taxi.png"],
    "other": ["other.png"],
    "bus": ["bus.png"],
    "large bus": ["large bus.png", "large_bus.png"],
}

SPRITE_HEADING_DEG = {
    "passenger car": 90.0,
    "taxi": 90.0,
    "other": 90.0,
    "bus": 90.0,
    "large bus": 90.0,
}

COLORS_BGR = {
    "passenger car": (80, 255, 80),
    "taxi": (0, 215, 255),
    "other": (255, 80, 200),
    "bus": (0, 255, 255),
    "large bus": (255, 160, 0),
}
