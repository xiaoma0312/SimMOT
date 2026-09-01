# -*- coding: utf-8 -*-
from __future__ import annotations

"""命令行入口和主仿真循环。

v10：双向直线多车道、IDM 跟驰、连续换道、目标车道补车保留。
后续如果主循环继续变大，可以再拆出 writer.py、scenario.py、validation.py。
"""

import argparse
import csv
import json
import math
import os
import random
from typing import Dict, List, Tuple

import cv2
import numpy as np

from .assets import load_sprites, make_background_if_missing
from . import config as sim_config
from .config import SPRITE_HEADING_DEG, VEHICLE_TYPES
from .demand import init_vehicles, try_spawn_vehicles
from .detection_noise import DetectionNoiseConfig, perturb_bbox, sample_false_positives, should_write_noisy_det
from .dynamics import bumper_gap, compute_idm_acceleration, enforce_no_overlap, find_leader, limit_acc_and_jerk
from .geometry import alpha_blend, alpha_blend_shadow, bbox_from_corners, oriented_box_corners, resize_rgba, rotate_rgba
from .models import TrafficScene
from .rendering import draw_lane_guides, draw_overlay_info, update_pose
from .utils import ensure_dir, mps_to_kmh, parse_norm_list, safe_path
from .validation import generate_validation_report
from .lane_change_model import LaneChangeModel


def _resolve_config_path(path: str, base_dir: str | None = None) -> str:
    """Resolve config path relative to the scenario config file directory."""
    if not path:
        return path
    if os.path.isabs(path):
        return path
    if base_dir:
        candidate = os.path.join(base_dir, path)
        if os.path.exists(candidate):
            return candidate
    return path


def _read_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
def _center_inside_image(cx: float, cy: float, width: int, height: int) -> bool:
    """Only output visible MOT labels when the object center is inside the image."""
    return 0.0 <= float(cx) < float(width) and 0.0 <= float(cy) < float(height)


def _clip_bbox_to_image(
    x: float,
    y: float,
    w: float,
    h: float,
    width: int,
    height: int,
):
    """Clip bbox to image range. Return None if the clipped bbox is invalid."""
    x1 = max(0.0, float(x))
    y1 = max(0.0, float(y))
    x2 = min(float(width), float(x) + float(w))
    y2 = min(float(height), float(y) + float(h))

    cw = x2 - x1
    ch = y2 - y1

    if cw <= 0.0 or ch <= 0.0:
        return None

    return x1, y1, cw, ch


def _find_leader_in_lane(ego, vehicles, lane_id: int):
    """Find leader in a specified global lane id. s_m increases along travel direction."""
    candidates = [
        v for v in vehicles
        if v.active and v.vid != ego.vid and v.lane == int(lane_id) and v.s_m > ego.s_m
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda z: z.s_m - ego.s_m)


def _lane_change_progress(v, t: float) -> float:
    if not getattr(v, "is_lane_changing", False):
        return 0.0
    start = float(getattr(v, "lane_change_start_time", t))
    duration = max(1e-9, float(getattr(v, "lane_change_duration", 1.0)))
    return max(0.0, min(1.0, (float(t) - start) / duration))


def _find_lane_changing_virtual_leader(ego, vehicles, t: float, lane_change_model=None):
    """Return a merging vehicle as a virtual leader for target-lane rear vehicles.

    Why this is necessary:
    - A lane-changing vehicle keeps its original ``v.lane`` until the maneuver completes.
    - Without a virtual leader, the rear vehicle in the target lane cannot anticipate the merge.
    - It may keep accelerating and then be corrected abruptly by enforce_no_overlap().

    This function makes a target-lane rear vehicle treat an ongoing merging vehicle ahead
    as its leader from the early phase of the maneuver, so IDM braking starts before any
    visual overlap occurs.
    """
    if lane_change_model is None:
        return None
    cfg = getattr(lane_change_model, "cfg", None)
    if cfg is not None and not bool(getattr(cfg, "virtual_merge_leader_enabled", True)):
        return None

    ego_lane = int(getattr(ego, "lane"))
    ego_dir = int(getattr(ego, "direction"))
    ego_s = float(getattr(ego, "s_m"))

    p_min = float(getattr(cfg, "virtual_merge_leader_progress_min", 0.02)) if cfg is not None else 0.02
    p_max = float(getattr(cfg, "virtual_merge_leader_progress_max", 1.0)) if cfg is not None else 1.0
    lookahead_m = float(getattr(cfg, "virtual_merge_leader_range_m", 120.0)) if cfg is not None else 120.0

    best = None
    best_gap = float("inf")
    for u in vehicles:
        if not getattr(u, "active", True):
            continue
        if getattr(u, "vid", None) == getattr(ego, "vid", None):
            continue
        if int(getattr(u, "direction")) != ego_dir:
            continue
        if not bool(getattr(u, "is_lane_changing", False)):
            continue

        target_lane = int(getattr(u, "lane_change_target_lane", getattr(u, "lane")))
        if target_lane != ego_lane:
            continue

        progress = _lane_change_progress(u, t)
        if progress < p_min or progress > p_max:
            continue

        ds = float(getattr(u, "s_m")) - ego_s
        if ds <= 0.0:
            continue

        # Use a conservative longitudinal gap. The real bumper_gap function can only be
        # called after choosing a leader, so here we approximate using physical lengths.
        gap = ds - 0.5 * (float(getattr(u, "length_m", 5.0)) + float(getattr(ego, "length_m", 5.0)))
        if gap > lookahead_m:
            continue

        # When the rear vehicle is faster, it must anticipate earlier. When it is not faster,
        # still keep the nearest merging vehicle as a leader if it is close enough.
        if gap < best_gap:
            best_gap = gap
            best = u

    if best is not None:
        try:
            lane_change_model.debug_counts["virtual_merge_leader_used"] = lane_change_model.debug_counts.get("virtual_merge_leader_used", 0) + 1
        except Exception:
            pass
    return best


def _find_leader_for_dynamics(ego, vehicles, t: float, lane_change_model=None):
    """Find the longitudinal leader used by IDM dynamics.

    For the changing vehicle itself, use the target-lane leader once it begins to commit.
    For target-lane rear vehicles, include the merging vehicle as a virtual leader much earlier.
    This avoids the unrealistic pattern: overlap -> enforce_no_overlap hard correction -> sudden speed drop.
    """
    base_leader = find_leader(ego, vehicles)

    if getattr(ego, "is_lane_changing", False):
        u = _lane_change_progress(ego, t)
        if u >= 0.35:
            target_leader = _find_leader_in_lane(
                ego,
                vehicles,
                int(getattr(ego, "lane_change_target_lane", ego.lane))
            )
            if target_leader is not None:
                base_leader = target_leader

    virtual_leader = _find_lane_changing_virtual_leader(ego, vehicles, t, lane_change_model)
    if virtual_leader is not None:
        if base_leader is None:
            return virtual_leader
        if float(getattr(virtual_leader, "s_m")) < float(getattr(base_leader, "s_m")):
            return virtual_leader
    return base_leader


def _apply_vehicle_types_config(vehicle_cfg_path: str, distribution: dict | None = None) -> None:
    """Update mutable vehicle dictionaries in-place so imported modules see the new values."""
    if not vehicle_cfg_path:
        return
    vehicle_cfg = _read_json(vehicle_cfg_path)

    cleaned = {}
    for name, info in vehicle_cfg.items():
        cleaned[name] = {
            "length_m": float(info["length_m"]),
            "width_m": float(info["width_m"]),
            "desired_speed_range_kmh": tuple(float(x) for x in info["desired_speed_range_kmh"]),
        }
        # Keep extra fields for documentation/output even if current dynamics still use global car-following args.
        for key, val in info.items():
            if key not in cleaned[name]:
                cleaned[name][key] = val

    sim_config.VEHICLE_TYPES.clear()
    sim_config.VEHICLE_TYPES.update(cleaned)

    if distribution:
        weights = [(name, float(w)) for name, w in distribution.items() if name in sim_config.VEHICLE_TYPES]
        total = sum(w for _, w in weights)
        if total > 0:
            sim_config.VEHICLE_TYPE_WEIGHTS[:] = [(name, w / total) for name, w in weights]


def _apply_mapping_to_args(args, mapping: dict) -> None:
    """Set argparse fields from a flat mapping when values are not None."""
    for key, val in mapping.items():
        if val is not None:
            setattr(args, key, val)


def _apply_scenario_config(args) -> None:
    """Load scenario JSON and override command-line defaults with structured config values.

    This is intentionally lightweight: it does not change the simulation logic. It only maps
    config fields to the existing argparse parameters, making the experiment reproducible.
    """
    scenario_path = getattr(args, "scenario_config", None)
    if not scenario_path:
        args.scenario_config_data = None
        args.camera_config_data = None
        args.noise_config_data = None
        return

    scenario_path = os.path.abspath(scenario_path)
    scenario_dir = os.path.dirname(scenario_path)
    cfg = _read_json(scenario_path)

    # Optional external camera config, then scenario-level camera overrides.
    camera_data = None
    camera_config_path = cfg.get("camera_config")
    if camera_config_path:
        camera_data = _read_json(_resolve_config_path(camera_config_path, scenario_dir))
    camera = {}
    if camera_data:
        camera.update(camera_data)
    camera.update(cfg.get("camera", {}))

    # Optional noise config is stored for reproducibility. Detection-noise injection is added later.
    noise_data = None
    noise_config_path = cfg.get("noise_config")
    if noise_config_path:
        noise_data = _read_json(_resolve_config_path(noise_config_path, scenario_dir))

    road = cfg.get("road", {})
    demand = cfg.get("traffic_demand", {})
    following = cfg.get("car_following", {})
    sim = cfg.get("simulation", {})

    # Load vehicle types and distribution before vehicle generation.
    vehicle_cfg_path = cfg.get("vehicle_types_config")
    if vehicle_cfg_path:
        _apply_vehicle_types_config(
            _resolve_config_path(vehicle_cfg_path, scenario_dir),
            demand.get("vehicle_type_distribution"),
        )

    # Map structured config names to existing argparse fields.
    mapping = {
        # simulation
        "fps": sim.get("fps"),
        "duration_sec": sim.get("duration_sec"),
        "num_vehicles": sim.get("num_vehicles"),
        "seed": sim.get("seed", cfg.get("random_seed")),
        # road
        "down_lanes_norm": road.get("down_lanes_norm"),
        "up_lanes_norm": road.get("up_lanes_norm"),
        "top_y_norm": road.get("top_y_norm"),
        "bottom_y_norm": road.get("bottom_y_norm"),
        "lane_offset_norm": road.get("lane_offset_norm"),
        # camera
        "meters_per_pixel": camera.get("meters_per_pixel"),
        "vehicle_mpp": camera.get("vehicle_mpp"),
        "sprite_scale_factor": camera.get("sprite_scale_factor"),
        # demand
        "flow_rate_vph_per_lane": demand.get("flow_rate_vph_per_lane"),
        "spawn_headway_min_sec": demand.get("spawn_headway_min_sec"),
        "spawn_headway_max_sec": demand.get("spawn_headway_max_sec"),
        "spawn_upstream_min_m": demand.get("spawn_upstream_min_m"),
        "spawn_upstream_max_m": demand.get("spawn_upstream_max_m"),
        "spawn_buffer_per_lane": demand.get("spawn_buffer_per_lane"),
        "spawn_follow_margin_m": demand.get("spawn_follow_margin_m"),
        "spawn_speed_sigma_kmh": demand.get("spawn_speed_sigma_kmh"),
        "upstream_buffer_m": demand.get("upstream_buffer_m"),
        "downstream_buffer_m": demand.get("downstream_buffer_m"),
        "initial_bumper_gap_m": demand.get("initial_bumper_gap_m"),
        "spawn_bumper_gap_m": demand.get("spawn_bumper_gap_m"),
        # following / free speed
        "following_variation_m": following.get("following_variation_m"),
        "min_gap_m": following.get("min_gap_m"),
        "time_headway_sec": following.get("tau_sec", following.get("time_headway_sec")),
        "max_acc_mps2": following.get("max_acc_mps2"),
        "comfort_dec_mps2": following.get("comfort_dec_mps2"),
        "max_dec_mps2": following.get("max_dec_mps2"),
        "normal_brake_limit_mps2": following.get("normal_brake_limit_mps2"),
        "close_brake_limit_mps2": following.get("close_brake_limit_mps2"),
        "soft_brake_gap_m": following.get("soft_brake_gap_m"),
        "max_jerk_mps3": following.get("max_jerk_mps3"),
        "acc_smoothing_alpha": following.get("acc_smoothing_alpha"),
        "substeps": following.get("substeps"),
        "extra_safety_gap_m": following.get("extra_safety_gap_m"),
        "free_accel_prob": following.get("free_accel_prob"),
        "free_decel_prob": following.get("free_decel_prob"),
        "free_accel_step_kmh": following.get("free_accel_step_kmh"),
        "free_decel_step_kmh": following.get("free_decel_step_kmh"),
        "free_relax_step_kmh": following.get("free_relax_step_kmh"),
        "decision_interval_min_sec": following.get("decision_interval_min_sec"),
        "decision_interval_max_sec": following.get("decision_interval_max_sec"),
        "global_min_speed_kmh": following.get("global_min_speed_kmh"),
        "global_max_speed_kmh": following.get("global_max_speed_kmh"),
        "rolling_min_speed_kmh": following.get("rolling_min_speed_kmh"),
    }
    _apply_mapping_to_args(args, mapping)

    args.scenario_config = scenario_path
    args.scenario_config_data = cfg
    args.camera_config_data = camera_data
    args.noise_config_data = noise_data


def main() -> None:
    parser = argparse.ArgumentParser(description="2D UAV-view bidirectional traffic simulation with IDM following and optional lane changing")

    parser.add_argument("--scenario-config", default=None, help="Structured JSON scenario config. If provided, it overrides matching command-line defaults.")

    parser.add_argument("--asset-dir", default=".")
    parser.add_argument("--background-image", default="background.jpg")
    parser.add_argument("--output-dir", default="./sim_accel_follow_v11")

    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration-sec", type=float, default=60.0)

    # 运动尺度：保持你前面实验中使用的 0.1m/px
    parser.add_argument("--meters-per-pixel", type=float, default=0.10)

    # 车辆尺寸：默认比 v7 放大。5m car -> about 66.7px
    parser.add_argument("--vehicle-mpp", type=float, default=0.050)
    parser.add_argument("--sprite-scale-factor", type=float, default=1.00)

    # 双向车道：左侧 top->bottom，右侧 bottom->top
    parser.add_argument("--down-lanes-norm", default="0.340,0.368,0.397,0.427,0.458,0.488,0.519")
    parser.add_argument("--up-lanes-norm", default="0.630,0.660,0.690,0.720,0.750,0.782,0.812")
    parser.add_argument("--top-y-norm", type=float, default=0.02)
    parser.add_argument("--bottom-y-norm", type=float, default=0.98)
    parser.add_argument("--lane-offset-norm", type=float, default=0.005, help="每辆车相对所属车道中心线的横向随机偏移范围，单位为图像宽度归一化值。")

    parser.add_argument("--num-vehicles", type=int, default=84)
    parser.add_argument("--seed", type=int, default=7)

    # 间距调大，避免大车贴图放大后拥挤
    parser.add_argument("--initial-bumper-gap-m", type=float, nargs=2, default=[5.5, 13.0])
    parser.add_argument("--spawn-bumper-gap-m", type=float, nargs=2, default=[6.5, 15.0])

    # 连续交通流补车参数：参考 SUMO flow/headway 和 VISSIM 安全距离思想，避免后半段车辆固定间距刷出
    parser.add_argument("--flow-rate-vph-per-lane", type=float, default=900.0, help="每车道流量，veh/h/lane，用于随机到达间隔。")
    parser.add_argument("--spawn-headway-min-sec", type=float, default=1.4)
    parser.add_argument("--spawn-headway-max-sec", type=float, default=5.8)
    parser.add_argument("--spawn-upstream-min-m", type=float, default=6.0)
    parser.add_argument("--spawn-upstream-max-m", type=float, default=70.0)
    parser.add_argument("--spawn-buffer-per-lane", type=int, default=2, help="允许每车道活跃车辆数略高于初始目标，保持后半段交通连续。")
    parser.add_argument("--following-variation-m", type=float, default=4.0, help="类 Wiedemann CC2 的跟驰距离波动项，单位 m。")
    parser.add_argument("--spawn-follow-margin-m", type=float, default=8.0)
    parser.add_argument("--spawn-speed-sigma-kmh", type=float, default=4.0)
    parser.add_argument("--upstream-buffer-m", type=float, default=90.0, help="可见区域上游的虚拟仿真缓冲区，使车辆入画前已发生跟驰。")
    parser.add_argument("--downstream-buffer-m", type=float, default=60.0, help="可见区域下游的虚拟仿真缓冲区，避免刚出画面就立刻删除。")

    # 自由流目标速度变化：更温和，避免前车突然大幅降速
    parser.add_argument("--free-accel-prob", type=float, default=0.26)
    parser.add_argument("--free-decel-prob", type=float, default=0.22)
    parser.add_argument("--free-accel-step-kmh", type=float, nargs=2, default=[2.5, 6.0])
    parser.add_argument("--free-decel-step-kmh", type=float, nargs=2, default=[3.0, 8.0])
    parser.add_argument("--free-relax-step-kmh", type=float, nargs=2, default=[1.5, 3.5])
    parser.add_argument("--decision-interval-min-sec", type=float, default=1.6)
    parser.add_argument("--decision-interval-max-sec", type=float, default=3.4)

    parser.add_argument("--global-min-speed-kmh", type=float, default=28.0)
    parser.add_argument("--global-max-speed-kmh", type=float, default=72.0)
    parser.add_argument("--rolling-min-speed-kmh", type=float, default=8.0)

    # IDM 跟驰参数：比 v7 减速更柔和
    parser.add_argument("--min-gap-m", type=float, default=3.2)
    parser.add_argument("--time-headway-sec", type=float, default=1.05)
    parser.add_argument("--max-acc-mps2", type=float, default=1.9)
    parser.add_argument("--comfort-dec-mps2", type=float, default=2.3)
    parser.add_argument("--max-dec-mps2", type=float, default=4.2)
    parser.add_argument("--normal-brake-limit-mps2", type=float, default=1.8)
    parser.add_argument("--close-brake-limit-mps2", type=float, default=3.2)
    parser.add_argument("--soft-brake-gap-m", type=float, default=13.0)
    parser.add_argument("--max-jerk-mps3", type=float, default=3.2)
    parser.add_argument("--acc-smoothing-alpha", type=float, default=0.70)
    parser.add_argument("--substeps", type=int, default=6)
    parser.add_argument("--extra-safety-gap-m", type=float, default=1.2)

    parser.add_argument("--draw-lane-guides", action="store_true")
    parser.add_argument("--save-frames", action="store_true")
    parser.add_argument(
        "--trail-max-points",
        type=int,
        default=180,
        help="Overlay trajectory length. 0 means full visible trajectory; positive value means recent N points."
    )

    cli_args = parser.parse_args()
    default_args = parser.parse_args([])
    args = argparse.Namespace(**vars(cli_args))
    _apply_scenario_config(args)

    # Explicit command-line values should override scenario-config values.
    # This keeps the config reproducible while still allowing quick smoke tests,
    # e.g., --duration-sec 5 or --output-dir ./outputs/tmp.
    for key, cli_val in vars(cli_args).items():
        if key == "scenario_config":
            continue
        default_val = getattr(default_args, key)
        if cli_val != default_val:
            setattr(args, key, cli_val)

    rng = random.Random(args.seed)
    random.seed(args.seed)
    np.random.seed(args.seed)
    noise_cfg = DetectionNoiseConfig.from_dict(args.noise_config_data)
    noise_rng = random.Random(args.seed + 1000003)

    ensure_dir(args.output_dir)
    if args.save_frames:
        ensure_dir(os.path.join(args.output_dir, "frames_clean"))
        ensure_dir(os.path.join(args.output_dir, "frames_overlay"))

    bg_path = safe_path(args.asset_dir, args.background_image)
    background = cv2.imread(bg_path, cv2.IMREAD_COLOR)
    if background is None:
        print(f"[warning] Cannot open background image: {bg_path}. Fallback background will be used.")
        background = make_background_if_missing()

    H, W = background.shape[:2]
    cv2.imwrite(os.path.join(args.output_dir, "background_used.jpg"), background)

    scene = TrafficScene(
        width=W,
        height=H,
        meters_per_pixel=args.meters_per_pixel,
        vehicle_mpp=args.vehicle_mpp,
        down_lanes_norm=parse_norm_list(args.down_lanes_norm),
        up_lanes_norm=parse_norm_list(args.up_lanes_norm),
        top_y_norm=args.top_y_norm,
        bottom_y_norm=args.bottom_y_norm,
        fps=args.fps,
    )

    # Lane-change model is controlled by scenario_config["lane_change"].
    # v3 uses explicit render-pose override after update_pose():
    # lateral x follows a quintic curve and heading follows the trajectory tangent.
    lane_change_model = None
    lane_change_cfg = (args.scenario_config_data or {}).get("lane_change", {}) if args.scenario_config_data else {}
    road_cfg = (args.scenario_config_data or {}).get("road", {}) if args.scenario_config_data else {}
    if bool(lane_change_cfg.get("enabled", False)) and bool(road_cfg.get("allow_lane_change", False)):
        lane_change_model = LaneChangeModel.from_scenario_config(args.scenario_config_data, scene=scene, seed=args.seed + 2027)
        # Used by demand.can_place(): block upstream spawning near vehicles that are
        # currently changing into or out of the same lane.
        args.spawn_lanechange_guard_m = float(lane_change_cfg.get("spawn_lanechange_guard_m", 28.0))
        args.spawn_reserve_s_max_m = float(lane_change_cfg.get("spawn_reserve_s_max_m", 95.0))
        args.spawn_reserve_hold_sec = float(lane_change_cfg.get("spawn_reserve_hold_sec", 2.5))
        args.spawn_reservation_skips = 0
        print("[lane_change] enabled:", lane_change_cfg.get("model", "IDM_MOBIL_with_quintic_lateral_trajectory"))
    else:
        args.spawn_lanechange_guard_m = 0.0
        args.spawn_reserve_s_max_m = 0.0
        args.spawn_reserve_hold_sec = 0.0
        args.spawn_reservation_skips = 0
        print("[lane_change] disabled")

    sprites: Dict[str, List[np.ndarray]] = {}
    sprite_sources: Dict[str, List[str]] = {}
    sprite_count_by_type: Dict[str, int] = {}
    for vtype in VEHICLE_TYPES:
        sprite_list, used_list = load_sprites(args.asset_dir, vtype)
        sprites[vtype] = sprite_list
        sprite_sources[vtype] = used_list
        sprite_count_by_type[vtype] = len(sprite_list)
        for idx, sprite in enumerate(sprite_list):
            cv2.imwrite(os.path.join(args.output_dir, f"sprite_check_{vtype}_{idx}.png"), sprite)

    vehicles, target_per_lane, next_vid = init_vehicles(scene, args.num_vehicles, rng, args, sprite_count_by_type)
    trails: Dict[int, List[Tuple[int, int]]] = {v.vid: [] for v in vehicles}

    clean_video_path = os.path.join(args.output_dir, "synthetic_clean.mp4")
    overlay_video_path = os.path.join(args.output_dir, "synthetic_gt_overlay.mp4")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    clean_writer = cv2.VideoWriter(clean_video_path, fourcc, float(args.fps), (W, H))
    overlay_writer = cv2.VideoWriter(overlay_video_path, fourcc, float(args.fps), (W, H))
    if not clean_writer.isOpened() or not overlay_writer.isOpened():
        raise RuntimeError("Failed to open video writer.")

    gt_path = os.path.join(args.output_dir, "gt_mot.txt")
    det_path = os.path.join(args.output_dir, "det_ideal.txt")
    det_noisy_path = os.path.join(args.output_dir, "det_noisy.txt")
    gt_csv_path = os.path.join(args.output_dir, "ground_truth.csv")
    full_csv_path = os.path.join(args.output_dir, "ground_truth_full.csv")
    behavior_path = os.path.join(args.output_dir, "behavior_log.csv")
    gap_path = os.path.join(args.output_dir, "gap_log.csv")
    config_path = os.path.join(args.output_dir, "config.json")

    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                **vars(args),
                "scenario_config_data": args.scenario_config_data,
                "camera_config_data": args.camera_config_data,
                "noise_config_data": args.noise_config_data,
                "noise_effective": noise_cfg.__dict__,
                "vehicle_type_weights": sim_config.VEHICLE_TYPE_WEIGHTS,
                "vehicle_types_effective": sim_config.VEHICLE_TYPES,
                "sprite_sources": sprite_sources,
                "visible_length_m": scene.visible_length_m,
                "lanes": [
                    {
                        "lid": lane.lid,
                        "x_px": lane.x_px,
                        "x_norm": lane.x_px / W,
                        "direction": "top_to_bottom" if lane.direction == +1 else "bottom_to_top",
                    }
                    for lane in scene.lanes
                ],
                "note": "v11 bidirectional, continuous flow spawning, Wiedemann/SUMO-style safety-distance parameters, no slow zone.",
            },
            f,
            ensure_ascii=False,
            indent=2,
        )

    total_frames = int(round(args.duration_sec * args.fps))
    substeps = max(1, int(args.substeps))
    sub_dt = scene.dt / substeps
    # Use a virtual upstream/downstream buffer for dynamics.
    # Vehicles outside the image still affect following in the buffer area.
    # MOT labels/detections are written only when the vehicle center is inside the image.
    s_min = -float(args.upstream_buffer_m)
    s_max = scene.visible_length_m + float(args.downstream_buffer_m)
    lane_next_spawn_time = [rng.uniform(0.0, args.spawn_headway_max_sec) for _ in range(scene.num_lanes)]

    min_gap_seen = float("inf")
    min_visual_gap_px_seen = float("inf")
    speed_samples_kmh: List[float] = []
    acc_samples: List[float] = []
    noisy_det_written = 0
    noisy_det_missed = 0
    noisy_false_positive_written = 0
    action_counts: Dict[str, int] = {}

    with open(gt_path, "w", encoding="utf-8") as f_gt, \
         open(det_path, "w", encoding="utf-8") as f_det, \
         open(det_noisy_path, "w", encoding="utf-8") as f_det_noisy, \
         open(gt_csv_path, "w", newline="", encoding="utf-8") as f_csv, \
         open(full_csv_path, "w", newline="", encoding="utf-8") as f_full, \
         open(behavior_path, "w", newline="", encoding="utf-8") as f_beh, \
         open(gap_path, "w", newline="", encoding="utf-8") as f_gap:

        writer = csv.writer(f_csv)
        writer_full = csv.writer(f_full)
        writer_beh = csv.writer(f_beh)
        writer_gap = csv.writer(f_gap)

        writer.writerow(["frame", "id", "vehicle_type", "lane", "direction", "speed_kmh", "acc_mps2", "cx", "cy", "bbox_x", "bbox_y", "bbox_w", "bbox_h", "action"])
        writer_full.writerow(["frame", "id", "active", "vehicle_type", "lane", "direction", "s_m", "speed_kmh", "desired_speed_kmh", "target_free_speed_kmh", "acc_mps2", "min_gap_m", "tau_sec", "max_acc_mps2", "comfort_dec_mps2", "max_dec_mps2", "leader_id", "leader_gap_m", "leader_speed_kmh", "action"])
        writer_beh.writerow(["frame", "time", "id", "lane", "direction", "leader_id", "leader_gap_m", "leader_speed_kmh", "speed_kmh", "target_free_speed_kmh", "acc_mps2", "action"])
        writer_gap.writerow(["frame", "lane", "direction", "leader_id", "follower_id", "bumper_gap_m", "visual_bumper_gap_px"])

        for frame_idx in range(1, total_frames + 1):
            frame_time = (frame_idx - 1) / float(args.fps)

            for sub in range(substeps):
                t_sub = frame_time + sub * sub_dt

                # v9: Lane-change decisions must be made BEFORE longitudinal IDM update.
                # This lets target-lane rear vehicles immediately see a merging vehicle
                # as a virtual leader in the same substep, rather than reacting only
                # after a visual overlap or enforce_no_overlap correction.
                if lane_change_model is not None:
                    lane_change_model.step(vehicles, t_sub, sub_dt)

                old_s = {v.vid: v.s_m for v in vehicles if v.active}
                acc_map: Dict[int, Tuple[float, str, float, float, int]] = {}

                for v in vehicles:
                    if not v.active:
                        continue
                    leader = _find_leader_for_dynamics(v, vehicles, t_sub, lane_change_model)
                    raw_acc, action, gap, leader_speed, leader_id = compute_idm_acceleration(v, leader, scene, t_sub, rng, args)
                    acc = limit_acc_and_jerk(v, raw_acc, sub_dt, args)
                    acc_map[v.vid] = (acc, action, gap, leader_speed, leader_id)

                for v in vehicles:
                    if not v.active:
                        continue
                    acc, action, _, _, _ = acc_map[v.vid]
                    v.s_m += v.v_mps * sub_dt + 0.5 * acc * sub_dt * sub_dt
                    v.v_mps = max(0.0, v.v_mps + acc * sub_dt)
                    v.acc_mps2 = acc
                    # Preserve lane-change/yield action labels when they are active.
                    if not str(getattr(v, "action", "")).startswith("lane_change") and str(getattr(v, "action", "")) != "yield_lane_change":
                        v.action = action

                enforce_no_overlap(vehicles, scene, old_s, args)

            # v10: spawn after lane-change decisions and one frame of dynamics.
            # This lets demand.py see vehicles that have just started merging and
            # reserve their target lanes before inserting new vehicles.
            next_vid = try_spawn_vehicles(
                vehicles, scene, rng, args, target_per_lane, next_vid, sprite_count_by_type,
                frame_time + scene.dt, lane_next_spawn_time
            )

            clean = background.copy()
            overlay = background.copy()
            if args.draw_lane_guides:
                # clean 视频不画车道中心路径线；overlay 中保留辅助线和车辆轨迹，方便检查。
                draw_lane_guides(overlay, scene)

            # remove vehicles after leaving downstream
            for v in vehicles:
                if v.active and v.s_m > s_max + 25.0:
                    v.active = False

            # gap log
            for lane_id in range(scene.num_lanes):
                lane_group = [v for v in vehicles if v.active and v.lane == lane_id]
                lane_group.sort(key=lambda z: z.s_m, reverse=True)
                for i in range(len(lane_group) - 1):
                    leader = lane_group[i]
                    follower = lane_group[i + 1]
                    gap_m = bumper_gap(follower, leader, scene, args.sprite_scale_factor, use_effective_length=True)
                    visual_gap_px = gap_m / scene.meters_per_pixel
                    writer_gap.writerow([
                        frame_idx,
                        lane_id,
                        "D" if scene.lane_direction(lane_id) == +1 else "U",
                        leader.vid,
                        follower.vid,
                        round(gap_m, 4),
                        round(visual_gap_px, 4),
                    ])
                    min_gap_seen = min(min_gap_seen, gap_m)
                    min_visual_gap_px_seen = min(min_visual_gap_px_seen, visual_gap_px)

            rows = []
            for v in vehicles:
                if not v.active:
                    continue

                leader = _find_leader_for_dynamics(v, vehicles, frame_time, lane_change_model)
                leader_id = leader.vid if leader is not None else -1
                leader_gap = bumper_gap(v, leader, scene, args.sprite_scale_factor, True) if leader is not None else float("inf")
                leader_speed = leader.v_mps if leader is not None else float("nan")
                speed_kmh = mps_to_kmh(v.v_mps)
                speed_samples_kmh.append(speed_kmh)
                acc_samples.append(v.acc_mps2)

                direction_text = "D" if v.direction == +1 else "U"
                writer_full.writerow([
                    frame_idx, v.vid, int(v.active), v.vehicle_type, v.lane, direction_text,
                    round(v.s_m, 3),
                    round(speed_kmh, 3),
                    round(mps_to_kmh(v.desired_speed_mps), 3),
                    round(mps_to_kmh(v.target_free_speed_mps), 3),
                    round(v.acc_mps2, 4),
                    round(getattr(v, "min_gap_m", args.min_gap_m), 3),
                    round(getattr(v, "time_headway_sec", args.time_headway_sec), 3),
                    round(getattr(v, "max_acc_mps2", args.max_acc_mps2), 3),
                    round(getattr(v, "comfort_dec_mps2", args.comfort_dec_mps2), 3),
                    round(getattr(v, "max_dec_mps2", args.max_dec_mps2), 3),
                    leader_id,
                    round(leader_gap, 3) if math.isfinite(leader_gap) else "inf",
                    round(mps_to_kmh(leader_speed), 3) if math.isfinite(leader_speed) else "nan",
                    v.action,
                ])
                writer_beh.writerow([
                    frame_idx,
                    f"{frame_time:.3f}",
                    v.vid,
                    v.lane,
                    direction_text,
                    leader_id,
                    f"{leader_gap:.3f}" if math.isfinite(leader_gap) else "inf",
                    f"{mps_to_kmh(leader_speed):.3f}" if math.isfinite(leader_speed) else "nan",
                    f"{speed_kmh:.3f}",
                    f"{mps_to_kmh(v.target_free_speed_mps):.3f}",
                    f"{v.acc_mps2:.4f}",
                    v.action,
                ])

                if v.s_m < s_min or v.s_m > s_max:
                    continue

                update_pose(v, scene)
                if lane_change_model is not None:
                    lane_change_model.apply_render_pose(v, scene, frame_time)

                vehicle_w_px = v.width_m / scene.vehicle_mpp * args.sprite_scale_factor
                vehicle_h_px = v.length_m / scene.vehicle_mpp * args.sprite_scale_factor
                corners = oriented_box_corners(v.x_px, v.y_px, vehicle_w_px, vehicle_h_px, v.heading_deg)
                bbox_x, bbox_y, bbox_w, bbox_h = bbox_from_corners(corners)

                # New visibility rule:
                # Only write MOT labels/detections when the vehicle center is inside the image.
                # Vehicles outside the image still participate in traffic dynamics, but are not labeled.
                if not _center_inside_image(v.x_px, v.y_px, W, H):
                    continue

                # Clip bbox to image range to avoid negative or outside-image boxes in MOT labels.
                clipped_bbox = _clip_bbox_to_image(bbox_x, bbox_y, bbox_w, bbox_h, W, H)
                if clipped_bbox is None:
                    continue

                bbox_x, bbox_y, bbox_w, bbox_h = clipped_bbox

                f_gt.write(f"{frame_idx},{v.vid},{bbox_x:.2f},{bbox_y:.2f},{bbox_w:.2f},{bbox_h:.2f},1,-1,-1,-1\n")
                f_det.write(f"{frame_idx},-1,{bbox_x:.2f},{bbox_y:.2f},{bbox_w:.2f},{bbox_h:.2f},1.000000,-1,-1,-1\n")
                noisy_bbox = perturb_bbox((bbox_x, bbox_y, bbox_w, bbox_h), W, H, noise_rng, noise_cfg)
                if noisy_bbox is None:
                    noisy_det_missed += 1
                else:
                    nx, ny, nw, nh, nconf = noisy_bbox
                    f_det_noisy.write(f"{frame_idx},-1,{nx:.2f},{ny:.2f},{nw:.2f},{nh:.2f},{nconf:.6f},-1,-1,-1\n")
                    noisy_det_written += 1
                action_counts[v.action] = action_counts.get(v.action, 0) + 1
                writer.writerow([
                    frame_idx, v.vid, v.vehicle_type, v.lane, direction_text,
                    round(speed_kmh, 3),
                    round(v.acc_mps2, 4),
                    round(v.x_px, 3),
                    round(v.y_px, 3),
                    round(bbox_x, 3),
                    round(bbox_y, 3),
                    round(bbox_w, 3),
                    round(bbox_h, 3),
                    v.action,
                ])

                base_list = sprites[v.vehicle_type]
                base = base_list[v.sprite_index % len(base_list)]
                sprite = resize_rgba(base, int(round(vehicle_w_px)), int(round(vehicle_h_px)))
                rot = SPRITE_HEADING_DEG[v.vehicle_type] - v.heading_deg
                sprite = rotate_rgba(sprite, rot)

                rows.append((v, sprite, corners, (bbox_x, bbox_y, bbox_w, bbox_h)))
                trails.setdefault(v.vid, []).append((int(round(v.x_px)), int(round(v.y_px))))
                if args.trail_max_points > 0 and len(trails[v.vid]) > args.trail_max_points:
                    trails[v.vid] = trails[v.vid][-args.trail_max_points:]

            for fp_frame, fx, fy, fw, fh, fconf in sample_false_positives(frame_idx, W, H, noise_rng, noise_cfg):
                f_det_noisy.write(f"{fp_frame},-1,{fx:.2f},{fy:.2f},{fw:.2f},{fh:.2f},{fconf:.6f},-1,-1,-1\n")
                noisy_false_positive_written += 1

            # draw vehicles
            # Shadow/overlap is minor; draw from upper y to lower y.
            rows.sort(key=lambda item: item[0].y_px)
            for v, sprite, _, _ in rows:
                alpha_blend_shadow(clean, sprite, (v.x_px, v.y_px))
                alpha_blend_shadow(overlay, sprite, (v.x_px, v.y_px))
                alpha_blend(clean, sprite, (v.x_px, v.y_px))
                alpha_blend(overlay, sprite, (v.x_px, v.y_px))

            for v, _, corners, bbox in rows:
                draw_overlay_info(overlay, trails, v, corners, bbox)

            clean_writer.write(clean)
            overlay_writer.write(overlay)

            if args.save_frames:
                cv2.imwrite(os.path.join(args.output_dir, "frames_clean", f"frame_{frame_idx:06d}.jpg"), clean)
                cv2.imwrite(os.path.join(args.output_dir, "frames_overlay", f"frame_{frame_idx:06d}.jpg"), overlay)

    clean_writer.release()
    overlay_writer.release()

    lane_change_log_path = None
    if lane_change_model is not None:
        lane_change_log_path = lane_change_model.save_log(args.output_dir)

    summary = {
        "description": "2D UAV-view bidirectional traffic simulation: continuous stochastic flow spawning, Wiedemann/SUMO-style safety-distance parameters, IDM following, emergency braking allowed, no slow zone.",
        "scenario_config": args.scenario_config,
        "scenario_name": (args.scenario_config_data or {}).get("scenario_name") if args.scenario_config_data else None,
        "visible_length_m": round(scene.visible_length_m, 3),
        "meters_per_pixel": args.meters_per_pixel,
        "vehicle_mpp": args.vehicle_mpp,
        "sprite_scale_factor": args.sprite_scale_factor,
        "lane_offset_norm": args.lane_offset_norm,
        "num_lanes": scene.num_lanes,
        "lanes": [
            {
                "lid": lane.lid,
                "x_norm": round(lane.x_px / W, 4),
                "direction": "top_to_bottom" if lane.direction == +1 else "bottom_to_top",
            }
            for lane in scene.lanes
        ],
        "min_bumper_gap_m": None if not math.isfinite(min_gap_seen) else round(min_gap_seen, 4),
        "min_visual_bumper_gap_px": None if not math.isfinite(min_visual_gap_px_seen) else round(min_visual_gap_px_seen, 4),
        "avg_speed_kmh": None if len(speed_samples_kmh) == 0 else round(float(np.mean(speed_samples_kmh)), 3),
        "p05_speed_kmh": None if len(speed_samples_kmh) == 0 else round(float(np.percentile(speed_samples_kmh, 5)), 3),
        "p50_speed_kmh": None if len(speed_samples_kmh) == 0 else round(float(np.percentile(speed_samples_kmh, 50)), 3),
        "p95_speed_kmh": None if len(speed_samples_kmh) == 0 else round(float(np.percentile(speed_samples_kmh, 95)), 3),
        "min_speed_kmh": None if len(speed_samples_kmh) == 0 else round(float(np.min(speed_samples_kmh)), 3),
        "min_acc_mps2": None if len(acc_samples) == 0 else round(float(np.min(acc_samples)), 4),
        "p01_acc_mps2": None if len(acc_samples) == 0 else round(float(np.percentile(acc_samples, 1)), 4),
        "detection_noise": {
            "config": noise_cfg.__dict__,
            "det_noisy_written": noisy_det_written,
            "det_noisy_missed": noisy_det_missed,
            "false_positive_written": noisy_false_positive_written,
        },
        "action_counts_visible": action_counts,
        "spawn_reservation_skips": int(getattr(args, "spawn_reservation_skips", 0)),
        "outputs": {
            "synthetic_clean.mp4": "clean rendered video",
            "synthetic_gt_overlay.mp4": "GT overlay video with boxes, IDs, speed, action",
            "gt_mot.txt": "MOT ground truth",
            "det_ideal.txt": "ideal detections",
            "det_noisy.txt": "controlled noisy detections generated from noise_config; same as ideal when noise_none",
            "ground_truth.csv": "visible labels",
            "ground_truth_full.csv": "all active vehicle states",
            "behavior_log.csv": "vehicle behavior states",
            "gap_log.csv": "leader-follower gap log",
            "lane_change_log.csv": "lane-change start/complete/abort event log" if lane_change_log_path else "not generated",
        },
    }

    with open(os.path.join(args.output_dir, "README.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    with open(os.path.join(args.output_dir, "safety_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    validation_report_path = generate_validation_report(args.output_dir)

    print("Done.")
    print(f"  clean video:   {clean_video_path}")
    print(f"  overlay video: {overlay_video_path}")
    print(f"  MOT gt:        {gt_path}")
    print(f"  ideal det:     {det_path}")
    print(f"  noisy det:     {det_noisy_path}")
    print(f"  full csv:      {full_csv_path}")
    print(f"  behavior log:  {behavior_path}")
    print(f"  gap log:       {gap_path}")
    if lane_change_log_path is not None:
        print(f"  lane-change:   {lane_change_log_path}")
    print(f"  summary:       {os.path.join(args.output_dir, 'safety_summary.json')}")
    print(f"  validation:    {validation_report_path}")
    print(f"  output dir:    {args.output_dir}")
