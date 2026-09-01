# -*- coding: utf-8 -*-
from __future__ import annotations

"""纵向动力学与跟驰模型：IDM、自由流目标速度、jerk 限制、防重叠。"""

import math
from typing import Dict, List, Optional, Tuple

from .models import TrafficScene, Vehicle
from .utils import clamp, kmh_to_mps


def rendered_length_equiv_m(v: Vehicle, scene: TrafficScene, sprite_scale_factor: float) -> float:
    """
    渲染车长折算到纵向道路坐标中的长度。
    用它做最终防重叠，避免视觉贴图相互覆盖。
    """
    rendered_len_px = (v.length_m / scene.vehicle_mpp) * sprite_scale_factor
    return rendered_len_px * scene.meters_per_pixel


def effective_length_m(v: Vehicle, scene: TrafficScene, sprite_scale_factor: float) -> float:
    return max(v.length_m, rendered_length_equiv_m(v, scene, sprite_scale_factor))


def find_leader(ego: Vehicle, vehicles: List[Vehicle]) -> Optional[Vehicle]:
    candidates = [
        v for v in vehicles
        if v.active and v.vid != ego.vid and v.lane == ego.lane and v.s_m > ego.s_m
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda z: z.s_m - ego.s_m)


def bumper_gap(ego: Vehicle, leader: Vehicle, scene: TrafficScene, sprite_scale_factor: float, use_effective_length: bool = True) -> float:
    if use_effective_length:
        ego_len = effective_length_m(ego, scene, sprite_scale_factor)
        lead_len = effective_length_m(leader, scene, sprite_scale_factor)
    else:
        ego_len = ego.length_m
        lead_len = leader.length_m
    return leader.s_m - ego.s_m - 0.5 * (lead_len + ego_len)


def decide_free_target_speed(v: Vehicle, t: float, rng: random.Random, args) -> str:
    if t < v.next_decision_time:
        return "hold_target"

    v.next_decision_time = t + rng.uniform(args.decision_interval_min_sec, args.decision_interval_max_sec)
    rv = rng.random()

    if rv < args.free_decel_prob:
        step = kmh_to_mps(rng.uniform(args.free_decel_step_kmh[0], args.free_decel_step_kmh[1]))
        v.target_free_speed_mps = max(v.free_speed_floor_mps, v.target_free_speed_mps - step)
        return "free_intend_decelerate"

    if rv < args.free_decel_prob + args.free_accel_prob:
        step = kmh_to_mps(rng.uniform(args.free_accel_step_kmh[0], args.free_accel_step_kmh[1]))
        v.target_free_speed_mps = min(v.desired_speed_mps, v.target_free_speed_mps + step)
        return "free_intend_accelerate"

    relax = kmh_to_mps(rng.uniform(args.free_relax_step_kmh[0], args.free_relax_step_kmh[1]))
    if v.target_free_speed_mps < v.desired_speed_mps:
        v.target_free_speed_mps = min(v.desired_speed_mps, v.target_free_speed_mps + relax)
    else:
        v.target_free_speed_mps = max(v.desired_speed_mps, v.target_free_speed_mps - relax)
    return "free_intend_cruise"


def compute_idm_acceleration(v: Vehicle, leader: Optional[Vehicle], scene: TrafficScene, t: float, rng: random.Random, args) -> Tuple[float, str, float, float, int]:
    base_action = decide_free_target_speed(v, t, rng, args)

    v0 = max(kmh_to_mps(args.rolling_min_speed_kmh), v.target_free_speed_mps)
    a = getattr(v, "max_acc_mps2", args.max_acc_mps2)
    b = getattr(v, "comfort_dec_mps2", args.comfort_dec_mps2)
    T = getattr(v, "time_headway_sec", args.time_headway_sec)
    s0 = getattr(v, "min_gap_m", args.min_gap_m)
    delta = 4.0

    if leader is None:
        raw_acc = a * (1.0 - (v.v_mps / max(0.1, v0)) ** delta)
        if raw_acc > 0.10:
            action = "free_accelerate"
        elif raw_acc < -0.10:
            action = "free_decelerate"
        else:
            action = "free_cruise"
        if base_action.startswith("free_intend"):
            action = base_action + "+" + action
        return raw_acc, action, float("inf"), float("nan"), -1

    gap = max(0.05, bumper_gap(v, leader, scene, args.sprite_scale_factor, use_effective_length=True))
    dv = v.v_mps - leader.v_mps

    # IDM desired dynamic gap
    s_star = s0 + max(
        0.0,
        v.v_mps * T + (v.v_mps * dv) / (2.0 * math.sqrt(max(0.1, a * b)))
    )
    raw_acc = a * (1.0 - (v.v_mps / max(0.1, v0)) ** delta - (s_star / gap) ** 2)

    # 减速不要过猛：不是危险距离时不允许一下子大制动
    if gap > args.soft_brake_gap_m:
        raw_acc = max(raw_acc, -args.normal_brake_limit_mps2)
    elif gap > s0:
        raw_acc = max(raw_acc, -args.close_brake_limit_mps2)

    # 常规跟驰避免无意义刹停；但紧急距离内允许刹停，避免碰撞和重叠
    rolling = kmh_to_mps(args.rolling_min_speed_kmh)
    emergency_gap = s0 * 0.75
    if v.v_mps < rolling and gap > emergency_gap:
        raw_acc = max(raw_acc, 0.0)

    if gap < emergency_gap:
        raw_acc = min(raw_acc, -getattr(v, "max_dec_mps2", args.max_dec_mps2))
        action = "emergency_follow_brake"
    elif raw_acc < -0.20:
        action = "follow_decelerate"
    elif abs(raw_acc) <= 0.20:
        action = "follow_match"
    else:
        action = "follow_recover"

    return raw_acc, action, gap, leader.v_mps, leader.vid


def limit_acc_and_jerk(v: Vehicle, raw_acc: float, dt: float, args) -> float:
    max_acc = getattr(v, "max_acc_mps2", args.max_acc_mps2)
    max_dec = getattr(v, "max_dec_mps2", args.max_dec_mps2)

    target_acc = clamp(raw_acc, -max_dec, max_acc)
    max_delta = args.max_jerk_mps3 * dt
    acc = clamp(target_acc, v.smoothed_acc_mps2 - max_delta, v.smoothed_acc_mps2 + max_delta)
    acc = args.acc_smoothing_alpha * acc + (1.0 - args.acc_smoothing_alpha) * v.smoothed_acc_mps2
    v.smoothed_acc_mps2 = acc
    return acc


def enforce_no_overlap(vehicles: List[Vehicle], scene: TrafficScene, old_s: Dict[int, float], args) -> None:
    for lane_id in range(scene.num_lanes):
        group = [v for v in vehicles if v.active and v.lane == lane_id]
        if len(group) < 2:
            continue
        group.sort(key=lambda z: old_s.get(z.vid, z.s_m), reverse=True)
        leader = group[0]
        for follower in group[1:]:
            lead_len = effective_length_m(leader, scene, args.sprite_scale_factor)
            foll_len = effective_length_m(follower, scene, args.sprite_scale_factor)
            min_center_gap = 0.5 * (lead_len + foll_len) + args.min_gap_m + args.extra_safety_gap_m
            max_allowed_s = leader.s_m - min_center_gap
            if follower.s_m > max_allowed_s:
                follower.s_m = max_allowed_s

                # 安全夹紧只在确实太近时触发；此时允许降到很低甚至停止
                target_v = max(0.0, leader.v_mps - 0.4)
                follower.v_mps = min(follower.v_mps, target_v)
                follower.smoothed_acc_mps2 = min(follower.smoothed_acc_mps2, -1.0)
                follower.action = "safety_clamp_no_overlap"
            leader = follower
