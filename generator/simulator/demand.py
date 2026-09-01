# -*- coding: utf-8 -*-
from __future__ import annotations

"""交通需求与车辆生成：初始布车、连续补车、补车速度协调、变道目标车道插入保留。"""

import math
import random
from typing import Dict, List, Optional, Tuple

from .config import COLORS_BGR, VEHICLE_TYPES
from .dynamics import effective_length_m
from .models import TrafficScene, Vehicle
from .utils import choose_weighted_vehicle_type, clamp, kmh_to_mps


def create_vehicle(vid: int, lane_id: int, s_m: float, scene: TrafficScene, rng: random.Random, args, sprite_count_by_type: Dict[str, int]) -> Vehicle:
    vtype = choose_weighted_vehicle_type(rng)
    info = VEHICLE_TYPES[vtype]
    lo, hi = info["desired_speed_range_kmh"]
    desired_kmh = rng.uniform(lo, hi)

    sigma = float(info.get("sigma", 0.0))
    if sigma > 0:
        desired_kmh *= rng.uniform(max(0.1, 1.0 - sigma), 1.0 + sigma)
    desired_kmh = min(desired_kmh, float(info.get("max_speed_kmh", args.global_max_speed_kmh)))

    init_kmh = clamp(desired_kmh + rng.uniform(-6.0, 3.0), args.global_min_speed_kmh, args.global_max_speed_kmh)
    free_floor_kmh = max(args.rolling_min_speed_kmh, desired_kmh - rng.uniform(10.0, 18.0))
    target_kmh = clamp(init_kmh + rng.uniform(-3.0, 3.0), free_floor_kmh, desired_kmh)

    n_sprites = max(1, int(sprite_count_by_type.get(vtype, 1)))
    return Vehicle(
        vid=vid,
        vehicle_type=vtype,
        lane=lane_id,
        direction=scene.lane_direction(lane_id),
        s_m=s_m,
        v_mps=kmh_to_mps(init_kmh),
        desired_speed_mps=kmh_to_mps(desired_kmh),
        target_free_speed_mps=kmh_to_mps(target_kmh),
        free_speed_floor_mps=kmh_to_mps(free_floor_kmh),
        color=COLORS_BGR[vtype],
        sprite_index=rng.randrange(n_sprites),
        lane_offset_norm=rng.uniform(-args.lane_offset_norm, args.lane_offset_norm),
        max_acc_mps2=float(info.get("accel_mps2", args.max_acc_mps2)),
        comfort_dec_mps2=float(info.get("decel_mps2", args.comfort_dec_mps2)),
        max_dec_mps2=float(info.get("emergency_decel_mps2", args.max_dec_mps2)),
        min_gap_m=float(info.get("min_gap_m", args.min_gap_m)),
        time_headway_sec=float(info.get("tau_sec", args.time_headway_sec)),
        sigma=sigma,
        next_decision_time=rng.uniform(0.3, 1.8),
        action="spawned",
    )


def _occupies_lane_for_spawn(other: Vehicle, lane_id: int) -> bool:
    """Return True if other should block insertion in lane_id.

    A lane-changing vehicle keeps ``other.lane`` as its source lane until the maneuver
    completes. If spawning only checks ``other.lane``, a new vehicle can be inserted
    into the target lane exactly while another vehicle is merging into it, producing
    the visual overlap observed near the upstream entry.

    Therefore a lane-changing vehicle blocks BOTH its source and target lanes for
    insertion and placement checks.
    """
    if int(other.lane) == int(lane_id):
        return True
    if bool(getattr(other, "is_lane_changing", False)):
        src = int(getattr(other, "lane_change_source_lane", other.lane))
        tgt = int(getattr(other, "lane_change_target_lane", other.lane))
        return int(lane_id) == src or int(lane_id) == tgt
    return False




def _target_lane_spawn_reserved(lane_id: int, vehicles: List[Vehicle], frame_time: float, args) -> bool:
    """Return True if upstream insertion into lane_id should be delayed.

    This is NOT an entrance lane-change ban. A vehicle is allowed to start a
    lane change near the upstream part of the scene. However, once it starts
    merging into a target lane, the target lane is temporarily reserved so that
    a new vehicle is not spawned into the same merge path. This mirrors the
    insertion-check idea used in microscopic traffic simulators: insertion is
    delayed until the target lane is safe.
    """
    reserve_s_max = float(getattr(args, "spawn_reserve_s_max_m", 0.0))
    reserve_hold_sec = float(getattr(args, "spawn_reserve_hold_sec", 0.0))
    if reserve_s_max <= 0.0 or reserve_hold_sec <= 0.0:
        return False

    for other in vehicles:
        if not getattr(other, "active", True):
            continue
        if not bool(getattr(other, "is_lane_changing", False)):
            continue
        tgt = int(getattr(other, "lane_change_target_lane", getattr(other, "lane", -999)))
        if tgt != int(lane_id):
            continue
        # Reserve only for upstream/entry-region lane changes, where new vehicles
        # could be inserted behind or beside the merge path.
        s = float(getattr(other, "s_m", 0.0))
        if s <= reserve_s_max:
            return True
    return False


def can_place(lane_id: int, s_m: float, v: Vehicle, vehicles: List[Vehicle], scene: TrafficScene, min_bumper_gap_m: float, args) -> bool:
    # Use a stronger insertion guard than ordinary car-following. Newly spawned
    # vehicles should never appear inside a lane-change swept region.
    spawn_extra = float(getattr(args, "spawn_lanechange_guard_m", 0.0))
    for other in vehicles:
        if not other.active or not _occupies_lane_for_spawn(other, lane_id):
            continue
        ego_len = effective_length_m(v, scene, args.sprite_scale_factor)
        oth_len = effective_length_m(other, scene, args.sprite_scale_factor)
        extra = spawn_extra if bool(getattr(other, "is_lane_changing", False)) else 0.0
        min_center_gap = 0.5 * (ego_len + oth_len) + min_bumper_gap_m + extra
        if abs(other.s_m - s_m) < min_center_gap:
            return False
    return True


def desired_safe_gap_m(speed_mps: float, args) -> float:
    """
    VISSIM/Wiedemann-style safety-distance component used for vehicle insertion and diagnostics.
    This is not a full Wiedemann implementation; it maps CC0+CC1*v to the IDM parameters
    already used by the simulator.
    """
    return args.min_gap_m + args.time_headway_sec * max(0.0, speed_mps)


def sample_spawn_headway_sec(rng: random.Random, args) -> float:
    """
    Stochastic vehicle-arrival headway, similar to defining a flow rate in microscopic simulators.
    The exponential draw avoids perfectly periodic vehicle insertion, while min/max clipping
    keeps the visual sequence stable for MOT experiments.
    """
    mean_headway = 3600.0 / max(1.0, float(args.flow_rate_vph_per_lane))
    h = rng.expovariate(1.0 / mean_headway)
    return clamp(h, args.spawn_headway_min_sec, args.spawn_headway_max_sec)


def adjust_spawn_speed_by_leader(v: Vehicle, leader: Optional[Vehicle], gap_m: float, rng: random.Random, args) -> None:
    """
    Make newly inserted vehicles enter the simulation in a car-following-compatible state.
    Without this, later vehicles are born with independent free-flow speeds, so the rear part
    of the video looks like fixed-interval spawning rather than continuous following.
    """
    if leader is None or not math.isfinite(gap_m):
        return

    desired_gap = desired_safe_gap_m(leader.v_mps, args) + rng.uniform(0.0, args.following_variation_m)
    close_enough_to_follow = gap_m <= desired_gap + args.spawn_follow_margin_m
    if close_enough_to_follow:
        speed_sigma = kmh_to_mps(args.spawn_speed_sigma_kmh)
        new_speed = leader.v_mps + rng.uniform(-speed_sigma, speed_sigma * 0.5)
        new_speed = clamp(
            new_speed,
            kmh_to_mps(args.rolling_min_speed_kmh),
            min(v.desired_speed_mps, leader.v_mps + kmh_to_mps(3.0)),
        )
        v.v_mps = new_speed
        v.target_free_speed_mps = min(v.desired_speed_mps, max(v.free_speed_floor_mps, new_speed + kmh_to_mps(1.0)))
        v.action = "spawned_following"


def init_vehicles(scene: TrafficScene, num_vehicles: int, rng: random.Random, args, sprite_count_by_type: Dict[str, int]) -> Tuple[List[Vehicle], List[int], int]:
    vehicles: List[Vehicle] = []
    next_vid = 1

    target_per_lane = [num_vehicles // scene.num_lanes for _ in range(scene.num_lanes)]
    for i in range(num_vehicles % scene.num_lanes):
        target_per_lane[i % scene.num_lanes] += 1

    for lane_id, count in enumerate(target_per_lane):
        s = rng.uniform(-10.0, 8.0)
        for _ in range(count):
            v = create_vehicle(next_vid, lane_id, s, scene, rng, args, sprite_count_by_type)
            tries = 0
            while not can_place(lane_id, s, v, vehicles, scene, args.initial_bumper_gap_m[0], args) and tries < 50:
                s += rng.uniform(4.0, 8.0)
                v = create_vehicle(next_vid, lane_id, s, scene, rng, args, sprite_count_by_type)
                tries += 1
            vehicles.append(v)
            next_vid += 1

            ego_len = effective_length_m(v, scene, args.sprite_scale_factor)
            gap = rng.uniform(args.initial_bumper_gap_m[0], args.initial_bumper_gap_m[1])
            s += ego_len + gap

    return vehicles, target_per_lane, next_vid


def try_spawn_vehicles(
    vehicles: List[Vehicle],
    scene: TrafficScene,
    rng: random.Random,
    args,
    target_per_lane: List[int],
    next_vid: int,
    sprite_count_by_type: Dict[str, int],
    frame_time: float,
    lane_next_spawn_time: List[float],
) -> int:
    """
    Continuous upstream vehicle generation.

    v10 used only a target active-count rule plus a fixed upstream gap. That made late vehicles
    look like they were inserted at fixed spacing. This version treats spawning as a traffic-flow
    arrival process and places new vehicles behind the current upstream leader using a
    Wiedemann/SUMO-style safety distance: s_safe = min_gap + time_headway * speed, plus a
    following-variation term.
    """
    for lane_id in range(scene.num_lanes):
        if frame_time < lane_next_spawn_time[lane_id]:
            continue

        # v10: target-lane insertion reservation. If another vehicle is currently
        # merging into this lane near the upstream entry, delay insertion instead
        # of spawning a new vehicle into the merge path.
        if _target_lane_spawn_reserved(lane_id, vehicles, frame_time, args):
            hold = float(getattr(args, "spawn_reserve_hold_sec", 2.5))
            lane_next_spawn_time[lane_id] = max(lane_next_spawn_time[lane_id], frame_time + hold)
            try:
                args.spawn_reservation_skips = int(getattr(args, "spawn_reservation_skips", 0)) + 1
            except Exception:
                pass
            continue

        active_lane = [v for v in vehicles if v.active and _occupies_lane_for_spawn(v, lane_id)]
        max_active_lane = max(1, target_per_lane[lane_id] + args.spawn_buffer_per_lane)
        if len(active_lane) >= max_active_lane:
            lane_next_spawn_time[lane_id] = frame_time + min(1.0, args.spawn_headway_min_sec)
            continue

        # nearest vehicle downstream of the upstream boundary; s always increases along travel direction
        upstream_leader = min(active_lane, key=lambda z: z.s_m, default=None)
        candidate = create_vehicle(next_vid, lane_id, 0.0, scene, rng, args, sprite_count_by_type)

        if upstream_leader is None:
            spawn_s = -rng.uniform(args.spawn_upstream_min_m, args.spawn_upstream_max_m)
            leader_gap = float("inf")
        else:
            lead_len = effective_length_m(upstream_leader, scene, args.sprite_scale_factor)
            cand_len = effective_length_m(candidate, scene, args.sprite_scale_factor)

            desired_gap = desired_safe_gap_m(upstream_leader.v_mps, args)
            desired_gap += rng.uniform(0.0, args.following_variation_m)
            desired_center_gap = 0.5 * (lead_len + cand_len) + desired_gap

            # Place the new vehicle behind the upstream leader at a realistic following distance.
            # It remains just outside / near the upstream boundary instead of appearing deep inside the frame.
            spawn_s = upstream_leader.s_m - desired_center_gap
            spawn_s = clamp(spawn_s, -args.spawn_upstream_max_m, -args.spawn_upstream_min_m)
            leader_gap = upstream_leader.s_m - spawn_s - 0.5 * (lead_len + cand_len)

            # If even the clamped upstream position is still too close, postpone rather than force insertion.
            min_entry_gap = args.min_gap_m + args.extra_safety_gap_m
            if leader_gap < min_entry_gap:
                lane_next_spawn_time[lane_id] = frame_time + 0.5
                continue

        candidate.s_m = spawn_s
        adjust_spawn_speed_by_leader(candidate, upstream_leader, leader_gap, rng, args)

        check_gap = max(args.min_gap_m, min(args.spawn_bumper_gap_m))
        if can_place(lane_id, candidate.s_m, candidate, vehicles, scene, check_gap, args):
            vehicles.append(candidate)
            next_vid += 1
            lane_next_spawn_time[lane_id] = frame_time + sample_spawn_headway_sec(rng, args)
        else:
            lane_next_spawn_time[lane_id] = frame_time + 0.5

    return next_vid
