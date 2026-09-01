# -*- coding: utf-8 -*-
from __future__ import annotations

"""Curved lane-change model for the current 2D UAV traffic simulator (v10 spawn-reservation + virtual-merge-leader safety).

v10 deliberately does NOT forbid lane changing near the upstream entrance. Instead,
when a vehicle starts merging into a target lane, the demand/spawn module reserves
that target lane for a short time so that a newly inserted vehicle is not spawned
inside the merge path. This matches microscopic-simulation practice more closely
than simply banning entry-zone lane changes.

Key properties for the paper/experiment:
1. Longitudinal motion is still governed by the existing IDM car-following module.
2. Lane-change decision follows an IDM/MOBIL-style safety and incentive logic.
3. Only adjacent lanes with the same direction are selectable.
4. Target-lane conflicts are checked against vehicles already in the target lane and
   vehicles currently changing into / out of that lane.
5. Once a lane change starts, the target is locked and the maneuver completes smoothly;
   it will not snap back to the source lane.
6. A rendered-size swept 2D safety envelope is checked over the whole maneuver horizon to prevent
   partial visual overlap with vehicles in the source/target lanes.
7. The safety envelope uses rendered vehicle footprint, not only physical vehicle width, because
   this simulator intentionally scales sprites by vehicle_mpp for visual clarity.
8. Lateral motion is a quintic smoothstep curve. Rendering pose uses the trajectory tangent,
   so the vehicle nose gradually rotates into the lane-change direction and then straightens.

This implementation does not copy SUMO/CARLA source code. It implements the standard ideas
of safety-gap checking, incentive-based lane changing, and continuous lateral trajectory.
"""

import csv
import json
import math
import os
import random
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

EPS = 1e-9


@dataclass
class LaneChangeConfig:
    enabled: bool = False
    model: str = "IDM_MOBIL_with_quintic_curved_lateral_trajectory"

    decision_interval_min_sec: float = 1.2
    decision_interval_max_sec: float = 2.6

    strategic_desire_prob: float = 0.12
    random_desire_prob: float = 0.02
    allow_safe_discretionary_change: bool = True
    safe_discretionary_prob: float = 0.06

    target_policy: str = "adjacent_only"

    front_gap_min_m: float = 12.0
    rear_gap_min_m: float = 10.0
    discretionary_front_gap_min_m: float = 8.0
    discretionary_rear_gap_min_m: float = 7.0
    conflict_gap_m: float = 30.0
    reciprocal_conflict_gap_m: float = 45.0

    # Predictive safety check prevents partial overlaps during the lane-change horizon.
    projected_safety_check: bool = True
    projected_time_step_sec: float = 0.5
    projected_horizon_extra_sec: float = 1.0
    projected_front_gap_min_m: float = 10.0
    projected_rear_gap_min_m: float = 9.0
    # 2D swept-envelope check in the road plane. This is stricter than simple front/rear gaps
    # and prevents visual overlap during the lateral transition.
    projected_2d_safety_check: bool = True
    swept_lateral_clearance_m: float = 0.8
    swept_longitudinal_clearance_m: float = 3.0
    lane_change_effective_width_gain_m: float = 1.6
    lane_change_effective_length_gain_m: float = 2.6
    # Use rendered footprint instead of nominal physical dimensions for overlap checks.
    # In this simulator vehicle_mpp may differ from meters_per_pixel, so the object displayed
    # on the video can occupy more road-plane meters than width_m/length_m.
    use_rendered_footprint_for_safety: bool = True
    rendered_footprint_safety_factor: float = 1.25
    sprite_scale_factor_for_safety: float = 1.0
    min_lateral_gap_m: float = 1.2
    same_direction_side_guard_gap_m: float = 2.0

    # v7 dynamic target-lane acceptance: reject a lane change if a faster rear vehicle
    # in the target lane would catch the ego vehicle before/during the maneuver.
    target_rear_min_ttc_sec: float = 8.0
    target_front_min_ttc_sec: float = 5.5
    target_rear_dynamic_headway_sec: float = 3.2
    target_front_dynamic_headway_sec: float = 1.8
    target_rear_speed_buffer_m: float = 8.0
    target_front_speed_buffer_m: float = 5.0
    dynamic_gap_use_rendered_length: bool = True

    # v7 cooperative yielding: once a lane change has started, the rear vehicle in the
    # target lane should gently yield rather than ignore the merging vehicle until it is
    # committed. This prevents late hard correction by enforce_no_overlap().
    cooperative_yield_enabled: bool = True
    cooperative_yield_range_m: float = 65.0
    cooperative_yield_min_gap_m: float = 16.0
    cooperative_yield_time_headway_sec: float = 1.4
    cooperative_yield_decel_mps2: float = 1.2
    cooperative_yield_progress_min: float = 0.12
    cooperative_yield_progress_max: float = 0.98

    # v8 runner-side anticipation: target-lane rear vehicles treat the merging vehicle
    # as a virtual IDM leader from the early phase of the lateral maneuver.
    virtual_merge_leader_enabled: bool = True
    virtual_merge_leader_progress_min: float = 0.02
    virtual_merge_leader_progress_max: float = 1.0
    virtual_merge_leader_range_m: float = 120.0

    # v10 spawn reservation parameters. These are read by runner/demand via
    # scenario config. They do not forbid entrance-zone lane changes; instead they
    # prevent new vehicles from being inserted into a target lane that is currently
    # being occupied by an upstream lane-change maneuver.
    spawn_lanechange_guard_m: float = 28.0
    spawn_reserve_s_max_m: float = 95.0
    spawn_reserve_hold_sec: float = 2.5
    spawn_reserve_target_only: bool = True

    # Clip virtual acceleration values used only for MOBIL/incentive evaluation.
    # Longitudinal dynamics are still controlled by runner.py and dynamics.py.
    acc_eval_clip_mps2: float = 3.5

    min_current_gap_trigger_m: float = 35.0
    speed_gain_threshold_kmh: float = 2.0
    gap_gain_threshold_m: float = 4.0

    safe_deceleration_mps2: float = 3.5
    politeness: float = 0.25
    incentive_threshold_mps2: float = 0.05
    right_bias_mps2: float = 0.0

    duration_min_sec: float = 4.2
    duration_max_sec: float = 5.8
    cooldown_min_sec: float = 7.0
    cooldown_max_sec: float = 13.0
    max_lane_changes_per_vehicle: int = 1
    lane_width_m: float = 3.5

    # v3 default: no mid-change snap-back. If you later need aborting, implement a reverse path,
    # not a teleport. For now, completed smooth maneuvers are more stable for dataset generation.
    abort_if_unsafe_during_change: bool = False
    mid_change_safety_check: bool = False

    # Rendering / heading control.
    max_heading_delta_deg: float = 14.0
    heading_smoothing_alpha: float = 0.35

    idm_min_gap_m: float = 3.2
    idm_tau_sec: float = 1.05
    idm_acc_mps2: float = 1.9
    idm_comfort_dec_mps2: float = 2.3
    idm_delta: float = 4.0
    idm_desired_speed_mps: float = 16.0

    log_file: str = "lane_change_log.csv"

    @staticmethod
    def from_dict(d: Optional[Dict[str, Any]], car_following: Optional[Dict[str, Any]] = None) -> "LaneChangeConfig":
        d = d or {}
        cf = car_following or {}
        cfg = LaneChangeConfig()
        for k, v in d.items():
            if hasattr(cfg, k):
                setattr(cfg, k, v)
        cfg.idm_min_gap_m = float(cf.get("min_gap_m", cfg.idm_min_gap_m))
        cfg.idm_tau_sec = float(cf.get("tau_sec", cf.get("time_headway_sec", cfg.idm_tau_sec)))
        cfg.idm_acc_mps2 = float(cf.get("max_acc_mps2", cfg.idm_acc_mps2))
        cfg.idm_comfort_dec_mps2 = float(cf.get("comfort_dec_mps2", cfg.idm_comfort_dec_mps2))
        cfg.idm_desired_speed_mps = float(cf.get("global_max_speed_kmh", 57.6)) / 3.6
        return cfg


def _vid(v: Any) -> int:
    return int(getattr(v, "vid"))


def _lane(v: Any) -> int:
    return int(getattr(v, "lane"))


def _direction(v: Any) -> int:
    return int(getattr(v, "direction"))


def _s(v: Any) -> float:
    return float(getattr(v, "s_m"))


def _v(v: Any) -> float:
    return float(getattr(v, "v_mps"))


def _length(v: Any) -> float:
    return float(getattr(v, "length_m"))


@dataclass
class CandidateDecision:
    target_lane: int
    score: float
    reason: str
    current_gap_m: float
    target_front_gap_m: float
    target_rear_gap_m: float
    ego_acc_gain_mps2: float
    mobil_incentive_mps2: float
    target_rear_acc_after_mps2: float


class LaneChangeModel:
    def __init__(self, cfg: LaneChangeConfig, scene: Any, seed: int = 0):
        self.cfg = cfg
        self.scene = scene
        self.rng = random.Random(seed)
        self.events: List[Dict[str, Any]] = []
        self.debug_counts: Dict[str, int] = {
            "active_steps": 0,
            "consider_checks": 0,
            "no_desire": 0,
            "no_candidate_lane": 0,
            "candidate_evaluations": 0,
            "reject_front_gap": 0,
            "reject_rear_gap": 0,
            "reject_rear_decel": 0,
            "reject_incentive": 0,
            "reject_target_conflict": 0,
            "reject_reciprocal_conflict": 0,
            "reject_projected_front_gap": 0,
            "reject_projected_rear_gap": 0,
            "reject_projected_collision": 0,
            "reject_projected_2d_overlap": 0,
            "reject_dynamic_rear_gap": 0,
            "reject_dynamic_front_gap": 0,
            "reject_rear_ttc": 0,
            "reject_front_ttc": 0,
            "cooperative_yield_events": 0,
            "virtual_merge_leader_used": 0,
            "reject_entry_exit_zone": 0,
            "start": 0,
            "complete": 0,
            "abort": 0,
        }

        self.lane_x_norm: Dict[int, float] = {
            int(lane.lid): float(lane.x_px) / float(scene.W) for lane in scene.lanes
        }
        self.lanes_by_direction: Dict[int, List[int]] = {+1: [], -1: []}
        for lane in scene.lanes:
            self.lanes_by_direction[int(lane.direction)].append(int(lane.lid))
        for d in self.lanes_by_direction:
            self.lanes_by_direction[d].sort(key=lambda lid: self.lane_x_norm[lid])

    @classmethod
    def from_scenario_config(cls, scenario_config: Dict[str, Any], scene: Any, seed: Optional[int] = None) -> "LaneChangeModel":
        if seed is None:
            seed = int(scenario_config.get("random_seed", scenario_config.get("simulation", {}).get("seed", 0)))
        cfg = LaneChangeConfig.from_dict(scenario_config.get("lane_change", {}), scenario_config.get("car_following", {}))
        return cls(cfg, scene, seed=seed)

    def _ensure_state(self, v: Any, now: float) -> None:
        if not hasattr(v, "base_lane_offset_norm"):
            v.base_lane_offset_norm = float(getattr(v, "lane_offset_norm", 0.0))
        if not hasattr(v, "is_lane_changing"):
            v.is_lane_changing = False
        if not hasattr(v, "lane_change_count"):
            v.lane_change_count = 0
        if not hasattr(v, "next_lane_change_decision_time"):
            v.next_lane_change_decision_time = now + self.rng.uniform(
                self.cfg.decision_interval_min_sec, self.cfg.decision_interval_max_sec)
        if not hasattr(v, "lane_change_cooldown_until"):
            v.lane_change_cooldown_until = 0.0
        if not hasattr(v, "lane_change_heading_deg"):
            v.lane_change_heading_deg = 90.0 if _direction(v) == +1 else 270.0
        if not bool(getattr(v, "is_lane_changing", False)):
            # Keep vehicles centered around their assigned lane when not changing.
            v.lane_offset_norm = float(v.base_lane_offset_norm)
            v.lateral_speed_mps = 0.0
            v.lateral_acc_mps2 = 0.0

    def candidate_lanes(self, v: Any) -> List[int]:
        direction = _direction(v)
        cur = _lane(v)
        lane_list = self.lanes_by_direction.get(direction, [])
        if cur not in lane_list:
            return []
        idx = lane_list.index(cur)
        out: List[int] = []
        if idx - 1 >= 0:
            out.append(lane_list[idx - 1])
        if idx + 1 < len(lane_list):
            out.append(lane_list[idx + 1])
        return out

    def _vehicle_occupies_lane_for_gap(self, u: Any, lane_id: int) -> bool:
        if _lane(u) == lane_id:
            return True
        if bool(getattr(u, "is_lane_changing", False)):
            src = int(getattr(u, "lane_change_source_lane", _lane(u)))
            tgt = int(getattr(u, "lane_change_target_lane", _lane(u)))
            # During a lane change, treat the vehicle as occupying both source and target for safety.
            return lane_id == src or lane_id == tgt
        return False

    def same_lane_vehicles_for_gap(self, vehicles: Iterable[Any], direction: int, lane_id: int) -> List[Any]:
        return [
            u for u in vehicles
            if getattr(u, "active", True)
            and _direction(u) == direction
            and self._vehicle_occupies_lane_for_gap(u, lane_id)
        ]

    def find_leader_follower(self, vehicles: Iterable[Any], ego: Any, lane_id: int) -> Tuple[Optional[Any], Optional[Any], float, float]:
        ego_s = _s(ego)
        direction = _direction(ego)
        leader = None
        follower = None
        front_gap = float("inf")
        rear_gap = float("inf")
        for u in self.same_lane_vehicles_for_gap(vehicles, direction, lane_id):
            if _vid(u) == _vid(ego):
                continue
            ds = _s(u) - ego_s
            if ds >= 0:
                gap = ds - 0.5 * (_length(u) + _length(ego))
                if gap < front_gap:
                    front_gap = gap
                    leader = u
            else:
                gap = -ds - 0.5 * (_length(u) + _length(ego))
                if gap < rear_gap:
                    rear_gap = gap
                    follower = u
        return leader, follower, front_gap, rear_gap

    def _target_conflict(self, vehicles: Sequence[Any], ego: Any, target_lane: int) -> bool:
        """Check target-lane reservation conflicts.

        Besides ordinary target-lane occupation, this function explicitly rejects
        reciprocal lane changes, e.g., A: lane 1 -> 2 while B nearby is lane 2 -> 1.
        Such crossing maneuvers easily look unnatural in a top-view rendering and
        should be avoided for high-confidence synthetic data generation.
        """
        ego_s = _s(ego)
        ego_src = _lane(ego)
        for u in vehicles:
            if not getattr(u, "active", True) or _vid(u) == _vid(ego) or _direction(u) != _direction(ego):
                continue
            if not bool(getattr(u, "is_lane_changing", False)):
                continue
            u_tgt = int(getattr(u, "lane_change_target_lane", _lane(u)))
            u_src = int(getattr(u, "lane_change_source_lane", _lane(u)))
            ds_abs = abs(_s(u) - ego_s)

            # Stronger constraint for reciprocal exchange: ego A->B and other B->A.
            if u_src == int(target_lane) and u_tgt == int(ego_src):
                if ds_abs < float(self.cfg.reciprocal_conflict_gap_m):
                    self.debug_counts["reject_reciprocal_conflict"] += 1
                    return True

            # General reservation: target lane or current conflict region is already occupied by a lane changer.
            if int(target_lane) in (u_tgt, u_src) or u_tgt == int(target_lane):
                if ds_abs < float(self.cfg.conflict_gap_m):
                    self.debug_counts["reject_target_conflict"] += 1
                    return True
        return False

    def _predict_s(self, v: Any, tau: float) -> float:
        a = float(getattr(v, "acc_mps2", 0.0))
        a = max(-float(self.cfg.acc_eval_clip_mps2), min(float(self.cfg.acc_eval_clip_mps2), a))
        return _s(v) + max(0.0, _v(v)) * tau + 0.5 * a * tau * tau


    def _lane_center_x_m(self, lane_id: int, v: Optional[Any] = None) -> float:
        base = float(self.lane_x_norm[int(lane_id)])
        offset = 0.0
        if v is not None:
            offset = float(getattr(v, "base_lane_offset_norm", getattr(v, "lane_offset_norm", 0.0)))
        return (base + offset) * float(self.scene.W) * float(self.scene.meters_per_pixel)

    def _planned_x_m_for_candidate(self, ego: Any, target_lane: int, tau: float, duration: Optional[float] = None) -> float:
        duration = float(duration if duration is not None else self.cfg.duration_max_sec)
        duration = max(duration, EPS)
        u = max(0.0, min(1.0, float(tau) / duration))
        src_x = self._lane_center_x_m(_lane(ego), ego)
        tgt_x = self._lane_center_x_m(int(target_lane), ego)
        return src_x + (tgt_x - src_x) * self.smoothstep5(u)

    def _predicted_x_m(self, v: Any, tau: float) -> float:
        if bool(getattr(v, "is_lane_changing", False)):
            start_x_norm = float(getattr(v, "lane_change_start_x_norm", self.lane_x_norm[_lane(v)]))
            target_x_norm = float(getattr(v, "lane_change_target_x_norm", self.lane_x_norm[_lane(v)]))
            duration = max(float(getattr(v, "lane_change_duration", self.cfg.duration_max_sec)), EPS)
            # tau is measured from the current decision time. Existing lane-change state uses elapsed progress.
            # A conservative approximation starts from its current x if exact start time is unavailable here.
            # If lane-change_start_time exists, use the stored absolute progression relative to the current call is
            # not available, so the call site supplies only future tau. We conservatively sweep between source and target.
            u0 = 0.0
            u = max(0.0, min(1.0, u0 + float(tau) / duration))
            x_norm = start_x_norm + (target_x_norm - start_x_norm) * self.smoothstep5(u)
            return x_norm * float(self.scene.W) * float(self.scene.meters_per_pixel)
        return self._lane_center_x_m(_lane(v), v)

    def _rendered_dimension_scale(self) -> float:
        """Return road-plane scale for rendered sprite footprint.

        Rendering uses: pixel_size = physical_size / vehicle_mpp * sprite_scale_factor.
        When safety checks are performed in road-plane meters based on meters_per_pixel,
        the rendered footprint becomes: physical_size * meters_per_pixel / vehicle_mpp.
        If this is ignored, the lane-change planner may accept maneuvers that are safe for
        nominal vehicle dimensions but overlap in the generated video.
        """
        if not bool(self.cfg.use_rendered_footprint_for_safety):
            return 1.0
        scene_mpp = max(EPS, float(getattr(self.scene, "meters_per_pixel", 0.10)))
        vehicle_mpp = max(EPS, float(getattr(self.scene, "vehicle_mpp", scene_mpp)))
        return scene_mpp / vehicle_mpp * float(self.cfg.sprite_scale_factor_for_safety)

    def _effective_half_width_m(self, v: Any, changing: bool = False) -> float:
        w = float(getattr(v, "width_m", 1.8)) * self._rendered_dimension_scale()
        w *= float(self.cfg.rendered_footprint_safety_factor)
        gain = float(self.cfg.lane_change_effective_width_gain_m) if changing else 0.0
        return 0.5 * (w + gain)

    def _effective_half_length_m(self, v: Any, changing: bool = False) -> float:
        length = float(getattr(v, "length_m", 4.5)) * self._rendered_dimension_scale()
        length *= float(self.cfg.rendered_footprint_safety_factor)
        gain = float(self.cfg.lane_change_effective_length_gain_m) if changing else 0.0
        return 0.5 * (length + gain)

    def _effective_length_m_for_gap(self, v: Any, changing: bool = False) -> float:
        """Effective rendered length used in target-lane gap acceptance.

        The ordinary physical length is too small for this simulator because rendered sprites
        are enlarged by vehicle_mpp. Using the rendered length keeps the planner consistent
        with the visible vehicle footprint and the generated bbox.
        """
        return 2.0 * self._effective_half_length_m(v, changing=changing)

    def _dynamic_target_gap_acceptance(self, ego: Any, tgt_leader: Optional[Any], tgt_follower: Optional[Any],
                                       tgt_front_gap: float, tgt_rear_gap: float, duration: Optional[float] = None) -> bool:
        """SUMO-like stringent gap acceptance using speed difference and TTC.

        Static front/rear gaps may look safe at the decision time, while a faster rear vehicle
        in the target lane can catch the merging vehicle during the 4--6 s lateral maneuver.
        This check rejects such cases before the lane change starts. It is intentionally
        conservative because the current simulator does not let target-lane vehicles fully
        anticipate the merge unless cooperative_yield is enabled.
        """
        duration = float(duration if duration is not None else self.cfg.duration_max_sec)
        horizon = duration + float(self.cfg.projected_horizon_extra_sec)

        # Rendered-footprint correction. Physical gap is reduced by the additional rendered
        # half-lengths of the two vehicles.
        ego_len_eff = self._effective_length_m_for_gap(ego, changing=True) if self.cfg.dynamic_gap_use_rendered_length else _length(ego)

        if tgt_follower is not None:
            fol_len_eff = self._effective_length_m_for_gap(tgt_follower, changing=False) if self.cfg.dynamic_gap_use_rendered_length else _length(tgt_follower)
            extra_len = max(0.0, 0.5 * (ego_len_eff + fol_len_eff) - 0.5 * (_length(ego) + _length(tgt_follower)))
            visual_rear_gap = float(tgt_rear_gap) - extra_len
            closing = max(0.0, _v(tgt_follower) - _v(ego))
            # Required rear gap contains a static part, a time-headway part and the closing distance
            # over the planned lateral maneuver. This keeps a faster target-lane rear vehicle from
            # entering the ego vehicle's swept envelope.
            required_rear = max(float(self.cfg.rear_gap_min_m), float(self.cfg.projected_rear_gap_min_m))
            required_rear += float(self.cfg.target_rear_speed_buffer_m)
            required_rear += closing * min(horizon, float(self.cfg.target_rear_dynamic_headway_sec))
            if visual_rear_gap < required_rear:
                self.debug_counts["reject_dynamic_rear_gap"] += 1
                return False
            if closing > 0.1:
                ttc = visual_rear_gap / max(closing, EPS)
                if ttc < float(self.cfg.target_rear_min_ttc_sec):
                    self.debug_counts["reject_rear_ttc"] += 1
                    return False

        if tgt_leader is not None:
            lead_len_eff = self._effective_length_m_for_gap(tgt_leader, changing=False) if self.cfg.dynamic_gap_use_rendered_length else _length(tgt_leader)
            extra_len = max(0.0, 0.5 * (ego_len_eff + lead_len_eff) - 0.5 * (_length(ego) + _length(tgt_leader)))
            visual_front_gap = float(tgt_front_gap) - extra_len
            closing_front = max(0.0, _v(ego) - _v(tgt_leader))
            required_front = max(float(self.cfg.front_gap_min_m), float(self.cfg.projected_front_gap_min_m))
            required_front += float(self.cfg.target_front_speed_buffer_m)
            required_front += closing_front * min(horizon, float(self.cfg.target_front_dynamic_headway_sec))
            if visual_front_gap < required_front:
                self.debug_counts["reject_dynamic_front_gap"] += 1
                return False
            if closing_front > 0.1:
                ttc = visual_front_gap / max(closing_front, EPS)
                if ttc < float(self.cfg.target_front_min_ttc_sec):
                    self.debug_counts["reject_front_ttc"] += 1
                    return False
        return True

    def _projected_2d_swept_safety(self, vehicles: Sequence[Any], ego: Any, target_lane: int,
                                   duration: Optional[float] = None) -> bool:
        """Reject lane changes whose swept 2D envelope overlaps another vehicle.

        Front/rear gap checks operate only along s. In the rendered top-view, a vehicle changes
        lane diagonally and its oriented box occupies both the source and target side for several
        seconds. This check predicts center positions in (x, s) and rejects cases where the
        enlarged rectangular envelopes overlap during the maneuver horizon.
        """
        if not bool(self.cfg.projected_2d_safety_check):
            return True
        duration = float(duration if duration is not None else self.cfg.duration_max_sec)
        horizon = duration + float(self.cfg.projected_horizon_extra_sec)
        dt = max(0.1, float(self.cfg.projected_time_step_sec))
        ego_dir = _direction(ego)
        ego_src = _lane(ego)
        ego_tgt = int(target_lane)
        affected_lanes = {ego_src, ego_tgt}
        ego_hw = self._effective_half_width_m(ego, changing=True)
        ego_hl = self._effective_half_length_m(ego, changing=True)
        lat_clear = float(self.cfg.swept_lateral_clearance_m)
        long_clear = float(self.cfg.swept_longitudinal_clearance_m)
        t = 0.0
        while t <= horizon + 1e-9:
            ego_s_t = self._predict_s(ego, t)
            ego_x_t = self._planned_x_m_for_candidate(ego, ego_tgt, t, duration=duration)
            for u in vehicles:
                if not getattr(u, "active", True) or _vid(u) == _vid(ego) or _direction(u) != ego_dir:
                    continue
                # Only vehicles in/near source or target lane can geometrically overlap the swept path.
                u_lane = _lane(u)
                u_src = int(getattr(u, "lane_change_source_lane", u_lane))
                u_tgt = int(getattr(u, "lane_change_target_lane", u_lane))
                if u_lane not in affected_lanes and u_src not in affected_lanes and u_tgt not in affected_lanes:
                    continue
                other_s_t = self._predict_s(u, t)
                other_x_t = self._predicted_x_m(u, t)
                other_changing = bool(getattr(u, "is_lane_changing", False))
                other_hw = self._effective_half_width_m(u, changing=other_changing)
                other_hl = self._effective_half_length_m(u, changing=other_changing)
                lateral_required = ego_hw + other_hw + max(lat_clear, float(self.cfg.min_lateral_gap_m))
                longitudinal_required = ego_hl + other_hl + long_clear
                # When the ego vehicle is between source and target lanes, it must keep a side
                # clearance from any vehicle occupying either lane. This mimics SUMO sublane-style
                # lateral-gap reasoning rather than a pure longitudinal gap check.
                if abs(ego_x_t - other_x_t) < lateral_required and abs(ego_s_t - other_s_t) < longitudinal_required:
                    self.debug_counts["reject_projected_2d_overlap"] += 1
                    return False
            t += dt
        return True

    def _projected_target_safety(self, vehicles: Sequence[Any], ego: Any, target_lane: int, front_min: float, rear_min: float, duration: Optional[float] = None) -> bool:
        """Predict gaps during the planned lane-change horizon.

        The static gap check may pass at the start time, while another vehicle in the
        target lane catches up during the 4--6 s maneuver. This predictive check
        rejects those cases before the lane change starts, preventing partial overlaps.
        """
        if not bool(self.cfg.projected_safety_check):
            return True

        horizon = max(float(self.cfg.duration_max_sec), float(self.cfg.duration_min_sec)) + float(self.cfg.projected_horizon_extra_sec)
        dt = max(0.1, float(self.cfg.projected_time_step_sec))
        front_thr = max(float(front_min), float(self.cfg.projected_front_gap_min_m))
        rear_thr = max(float(rear_min), float(self.cfg.projected_rear_gap_min_m))
        direction = _direction(ego)
        ego_len = _length(ego)

        t = 0.0
        while t <= horizon + 1e-9:
            ego_s_t = self._predict_s(ego, t)
            for u in vehicles:
                if not getattr(u, "active", True) or _vid(u) == _vid(ego) or _direction(u) != direction:
                    continue
                if not self._vehicle_occupies_lane_for_gap(u, int(target_lane)):
                    continue
                other_s_t = self._predict_s(u, t)
                ds = other_s_t - ego_s_t
                min_body_gap = 0.5 * (_length(u) + ego_len)
                clearance = abs(ds) - min_body_gap
                if clearance < 0.8:
                    self.debug_counts["reject_projected_collision"] += 1
                    return False
                if ds >= 0.0 and clearance < front_thr:
                    self.debug_counts["reject_projected_front_gap"] += 1
                    return False
                if ds < 0.0 and clearance < rear_thr:
                    self.debug_counts["reject_projected_rear_gap"] += 1
                    return False
            t += dt
        return self._projected_2d_swept_safety(vehicles, ego, target_lane, duration=duration)

    def idm_acc(self, ego: Any, leader: Optional[Any], gap_m: Optional[float] = None) -> float:
        cfg = self.cfg
        v = max(0.0, _v(ego))
        desired = max(0.1, float(getattr(ego, "target_free_speed_mps", cfg.idm_desired_speed_mps)))
        a = float(getattr(ego, "max_acc_mps2", cfg.idm_acc_mps2))
        b = float(getattr(ego, "comfort_dec_mps2", cfg.idm_comfort_dec_mps2))
        T = float(getattr(ego, "time_headway_sec", cfg.idm_tau_sec))
        s0 = float(getattr(ego, "min_gap_m", cfg.idm_min_gap_m))
        free_term = (v / desired) ** cfg.idm_delta
        if leader is None or gap_m is None or math.isinf(gap_m):
            out = a * (1.0 - free_term)
            return max(-float(self.cfg.acc_eval_clip_mps2), min(float(self.cfg.acc_eval_clip_mps2), out))
        gap = max(float(gap_m), 0.1)
        dv = v - max(0.0, _v(leader))
        s_star = s0 + max(0.0, v * T + (v * dv) / (2.0 * math.sqrt(max(a * b, EPS))))
        out = a * (1.0 - free_term - (s_star / gap) ** 2)
        return max(-float(self.cfg.acc_eval_clip_mps2), min(float(self.cfg.acc_eval_clip_mps2), out))

    def _acc_of_follower(self, follower: Optional[Any], leader: Optional[Any], gap_m: float) -> float:
        if follower is None:
            return 0.0
        return self.idm_acc(follower, leader, gap_m)

    def evaluate_candidate(self, vehicles: Sequence[Any], ego: Any, target_lane: int, allow_discretionary: bool = False) -> Optional[CandidateDecision]:
        if self._target_conflict(vehicles, ego, target_lane):
            return None

        cur_lane = _lane(ego)
        cur_leader, cur_follower, cur_front_gap, cur_rear_gap = self.find_leader_follower(vehicles, ego, cur_lane)
        tgt_leader, tgt_follower, tgt_front_gap, tgt_rear_gap = self.find_leader_follower(vehicles, ego, target_lane)

        self.debug_counts["candidate_evaluations"] += 1
        front_min = self.cfg.discretionary_front_gap_min_m if allow_discretionary else self.cfg.front_gap_min_m
        rear_min = self.cfg.discretionary_rear_gap_min_m if allow_discretionary else self.cfg.rear_gap_min_m
        if tgt_front_gap < front_min:
            self.debug_counts["reject_front_gap"] += 1
            return None
        if tgt_rear_gap < rear_min:
            self.debug_counts["reject_rear_gap"] += 1
            return None

        planned_duration = float(self.cfg.duration_max_sec)
        if not self._dynamic_target_gap_acceptance(ego, tgt_leader, tgt_follower, tgt_front_gap, tgt_rear_gap, duration=planned_duration):
            return None

        if not self._projected_target_safety(vehicles, ego, target_lane, front_min, rear_min, duration=planned_duration):
            return None

        ego_acc_old = self.idm_acc(ego, cur_leader, cur_front_gap)
        ego_acc_new = self.idm_acc(ego, tgt_leader, tgt_front_gap)
        ego_gain = ego_acc_new - ego_acc_old

        new_follower_acc_before = self._acc_of_follower(tgt_follower, tgt_leader, tgt_rear_gap)
        new_follower_acc_after = self._acc_of_follower(tgt_follower, ego, tgt_rear_gap)
        if new_follower_acc_after < -abs(self.cfg.safe_deceleration_mps2):
            self.debug_counts["reject_rear_decel"] += 1
            return None

        old_follower_acc_before = self._acc_of_follower(cur_follower, ego, cur_rear_gap)
        old_after_gap = cur_front_gap + cur_rear_gap + _length(ego)
        old_follower_acc_after = self._acc_of_follower(cur_follower, cur_leader, old_after_gap)

        social_term = self.cfg.politeness * ((new_follower_acc_after - new_follower_acc_before) +
                                             (old_follower_acc_after - old_follower_acc_before))
        bias = self.cfg.right_bias_mps2 if target_lane > cur_lane else 0.0
        mobil_incentive = ego_gain + social_term + bias

        if cur_leader is not None and tgt_leader is not None:
            target_speed_gain_kmh = (_v(tgt_leader) - _v(cur_leader)) * 3.6
        elif cur_leader is not None and tgt_leader is None:
            target_speed_gain_kmh = self.cfg.speed_gain_threshold_kmh + 1.0
        else:
            target_speed_gain_kmh = 0.0
        gap_gain = tgt_front_gap - cur_front_gap

        has_pressure = cur_front_gap < self.cfg.min_current_gap_trigger_m
        has_speed_gain = target_speed_gain_kmh >= self.cfg.speed_gain_threshold_kmh
        has_gap_gain = gap_gain >= self.cfg.gap_gain_threshold_m
        passes_mobil = mobil_incentive >= self.cfg.incentive_threshold_mps2

        if not (passes_mobil and (has_pressure or has_speed_gain or has_gap_gain)):
            if not (allow_discretionary and self.cfg.allow_safe_discretionary_change):
                self.debug_counts["reject_incentive"] += 1
                return None

        reasons = []
        if has_pressure:
            reasons.append("front_pressure")
        if has_speed_gain:
            reasons.append("speed_gain")
        if has_gap_gain:
            reasons.append("gap_gain")
        if passes_mobil:
            reasons.append("mobil_incentive")
        if allow_discretionary and self.cfg.allow_safe_discretionary_change:
            reasons.append("safe_discretionary")
        reason = "+".join(reasons) if reasons else "accepted"

        score = mobil_incentive + 0.03 * max(0.0, gap_gain) + 0.02 * max(0.0, target_speed_gain_kmh)
        return CandidateDecision(
            target_lane=int(target_lane),
            score=float(score),
            reason=reason,
            current_gap_m=float(cur_front_gap),
            target_front_gap_m=float(tgt_front_gap),
            target_rear_gap_m=float(tgt_rear_gap),
            ego_acc_gain_mps2=float(ego_gain),
            mobil_incentive_mps2=float(mobil_incentive),
            target_rear_acc_after_mps2=float(new_follower_acc_after),
        )

    @staticmethod
    def smoothstep5(u: float) -> float:
        u = max(0.0, min(1.0, float(u)))
        return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5

    @staticmethod
    def smoothstep5_d1(u: float) -> float:
        u = max(0.0, min(1.0, float(u)))
        return 30.0 * u**2 * (1.0 - u) ** 2

    @staticmethod
    def smoothstep5_d2(u: float) -> float:
        u = max(0.0, min(1.0, float(u)))
        return 60.0 * u - 180.0 * u**2 + 120.0 * u**3

    def _schedule_next(self, v: Any, now: float) -> None:
        v.next_lane_change_decision_time = now + self.rng.uniform(
            self.cfg.decision_interval_min_sec, self.cfg.decision_interval_max_sec)

    def _x_norm_with_offset(self, lane_id: int, v: Any) -> float:
        return self.lane_x_norm[int(lane_id)] + float(getattr(v, "base_lane_offset_norm", 0.0))

    def _start(self, v: Any, decision: CandidateDecision, now: float) -> None:
        source = _lane(v)
        target = int(decision.target_lane)
        duration = self.rng.uniform(self.cfg.duration_min_sec, self.cfg.duration_max_sec)
        v.is_lane_changing = True
        v.lane_change_source_lane = source
        v.lane_change_target_lane = target
        v.lane_change_start_time = now
        v.lane_change_duration = duration
        v.lane_change_start_x_norm = self._x_norm_with_offset(source, v)
        v.lane_change_target_x_norm = self._x_norm_with_offset(target, v)
        v.lane_change_reason = decision.reason
        v.action = "lane_change_start"
        self.debug_counts["start"] += 1
        self.events.append({
            "event": "start", "time_sec": now, "vehicle_id": _vid(v),
            "direction": "D" if _direction(v) == +1 else "U",
            "source_lane": source, "target_lane": target, "duration_sec": duration,
            "current_gap_m": decision.current_gap_m, "target_front_gap_m": decision.target_front_gap_m,
            "target_rear_gap_m": decision.target_rear_gap_m, "ego_acc_gain_mps2": decision.ego_acc_gain_mps2,
            "mobil_incentive_mps2": decision.mobil_incentive_mps2,
            "target_rear_acc_after_mps2": decision.target_rear_acc_after_mps2,
            "reason": decision.reason,
        })

    def _complete(self, v: Any, now: float) -> None:
        source = int(getattr(v, "lane_change_source_lane", _lane(v)))
        target = int(getattr(v, "lane_change_target_lane", _lane(v)))
        v.lane = target
        v.lane_offset_norm = float(getattr(v, "base_lane_offset_norm", 0.0))
        v.is_lane_changing = False
        v.lane_change_count = int(getattr(v, "lane_change_count", 0)) + 1
        v.lane_change_cooldown_until = now + self.rng.uniform(self.cfg.cooldown_min_sec, self.cfg.cooldown_max_sec)
        self._schedule_next(v, now)
        v.lateral_speed_mps = 0.0
        v.lateral_acc_mps2 = 0.0
        v.action = "lane_change_complete"
        self.debug_counts["complete"] += 1
        self.events.append({
            "event": "complete", "time_sec": now, "vehicle_id": _vid(v),
            "direction": "D" if _direction(v) == +1 else "U",
            "source_lane": source, "target_lane": target, "duration_sec": float(getattr(v, "lane_change_duration", 0.0)),
            "current_gap_m": "", "target_front_gap_m": "", "target_rear_gap_m": "",
            "ego_acc_gain_mps2": "", "mobil_incentive_mps2": "", "target_rear_acc_after_mps2": "",
            "reason": getattr(v, "lane_change_reason", "complete"),
        })

    def _update_ongoing(self, v: Any, now: float) -> None:
        # No snap-back abort in v3. The target is locked after start and completes smoothly.
        start = float(getattr(v, "lane_change_start_time"))
        duration = max(float(getattr(v, "lane_change_duration")), EPS)
        u = (now - start) / duration
        if u >= 1.0:
            self._complete(v, now)
            return
        v.action = f"lane_change_to_{int(getattr(v, 'lane_change_target_lane'))}"

    def should_consider(self, v: Any, now: float) -> bool:
        if not self.cfg.enabled:
            return False
        if bool(getattr(v, "is_lane_changing", False)):
            return False
        if int(getattr(v, "lane_change_count", 0)) >= int(self.cfg.max_lane_changes_per_vehicle):
            return False
        if now < float(getattr(v, "lane_change_cooldown_until", 0.0)):
            return False
        if now < float(getattr(v, "next_lane_change_decision_time", 0.0)):
            return False

        # v10: Do not ban entrance-zone lane changes. Entrance conflicts are handled
        # by the spawn-reservation logic in demand.py. We still keep the change from
        # starting extremely close to the downstream end, because a maneuver that
        # cannot finish before the vehicle leaves the scene is not useful for MOT.
        s_now = _s(v)
        visible_len = float(getattr(self.scene, "visible_length_m", 0.0))
        end_margin = float(getattr(self.cfg, "end_s_margin_for_lane_change_m", 0.0)) if hasattr(self.cfg, "end_s_margin_for_lane_change_m") else 0.0
        if end_margin > 0.0 and visible_len > 0.0 and s_now > visible_len - end_margin:
            self.debug_counts["reject_entry_exit_zone"] = self.debug_counts.get("reject_entry_exit_zone", 0) + 1
            self._schedule_next(v, now)
            return False
        return True

    def try_start(self, vehicles: Sequence[Any], v: Any, now: float) -> None:
        self._schedule_next(v, now)
        p_strategic = max(0.0, float(self.cfg.strategic_desire_prob))
        p_random = max(0.0, float(self.cfg.random_desire_prob))
        p_disc = max(0.0, float(self.cfg.safe_discretionary_prob))
        roll = self.rng.random()
        if roll > p_strategic + p_random + p_disc:
            self.debug_counts["no_desire"] += 1
            return

        candidates = self.candidate_lanes(v)
        if not candidates:
            self.debug_counts["no_candidate_lane"] += 1
            return

        allow_discretionary = roll > p_strategic + p_random
        decisions: List[CandidateDecision] = []
        for target in candidates:
            dec = self.evaluate_candidate(vehicles, v, target, allow_discretionary=allow_discretionary)
            if dec is not None:
                decisions.append(dec)

        if not decisions and self.cfg.allow_safe_discretionary_change and p_disc > 0.0:
            # Fallback: try safety-constrained discretionary mode once. Still no unsafe random jump.
            if self.rng.random() < p_disc:
                for target in candidates:
                    dec = self.evaluate_candidate(vehicles, v, target, allow_discretionary=True)
                    if dec is not None:
                        decisions.append(dec)

        if not decisions:
            return
        decisions.sort(key=lambda d: d.score, reverse=True)
        self._start(v, decisions[0], now)

    def _progress_of(self, v: Any, now: float) -> float:
        if not bool(getattr(v, "is_lane_changing", False)):
            return 0.0
        start = float(getattr(v, "lane_change_start_time", now))
        duration = max(float(getattr(v, "lane_change_duration", self.cfg.duration_max_sec)), EPS)
        return max(0.0, min(1.0, (float(now) - start) / duration))

    def _find_target_rear_for_ongoing(self, vehicles: Sequence[Any], ego: Any) -> Tuple[Optional[Any], float]:
        target_lane = int(getattr(ego, "lane_change_target_lane", _lane(ego)))
        ego_s = _s(ego)
        follower = None
        rear_gap = float("inf")
        for u in vehicles:
            if not getattr(u, "active", True) or _vid(u) == _vid(ego) or _direction(u) != _direction(ego):
                continue
            if _lane(u) != target_lane:
                continue
            ds = ego_s - _s(u)
            if ds <= 0.0:
                continue
            gap = ds - 0.5 * (_length(u) + _length(ego))
            if gap < rear_gap:
                rear_gap = gap
                follower = u
        return follower, rear_gap

    def _apply_cooperative_yield(self, vehicles: Sequence[Any], now: float, dt: float) -> None:
        """Gently slow target-lane rear vehicles while ego is merging.

        Without this anticipatory response, the target-lane rear vehicle may keep accelerating
        until the lane-change vehicle is marked as being in its lane. Then enforce_no_overlap()
        has to correct the situation abruptly, creating the unrealistic 'flash-back' effect.
        """
        if not bool(self.cfg.cooperative_yield_enabled):
            return
        for ego in vehicles:
            if not getattr(ego, "active", True) or not bool(getattr(ego, "is_lane_changing", False)):
                continue
            u = self._progress_of(ego, now)
            if u < float(self.cfg.cooperative_yield_progress_min) or u > float(self.cfg.cooperative_yield_progress_max):
                continue
            follower, rear_gap = self._find_target_rear_for_ongoing(vehicles, ego)
            if follower is None or not math.isfinite(rear_gap):
                continue
            if rear_gap > float(self.cfg.cooperative_yield_range_m):
                continue
            closing = _v(follower) - _v(ego)
            desired_gap = float(self.cfg.cooperative_yield_min_gap_m) + max(0.0, _v(follower)) * float(self.cfg.cooperative_yield_time_headway_sec)
            if closing <= 0.0 and rear_gap >= desired_gap:
                continue
            # Compute a gentle target speed. Do not teleport speed; limit the speed reduction by
            # cooperative_yield_decel_mps2 * dt.
            target_speed = max(0.0, _v(ego) + max(0.0, rear_gap - float(self.cfg.cooperative_yield_min_gap_m)) / max(float(self.cfg.cooperative_yield_time_headway_sec), EPS))
            old_v = _v(follower)
            if target_speed < old_v:
                new_v = max(target_speed, old_v - float(self.cfg.cooperative_yield_decel_mps2) * max(float(dt), EPS))
                follower.v_mps = max(0.0, new_v)
                follower.acc_mps2 = min(float(getattr(follower, "acc_mps2", 0.0)), (new_v - old_v) / max(float(dt), EPS))
                follower.action = "yield_lane_change"
                self.debug_counts["cooperative_yield_events"] += 1

    def step(self, vehicles: Sequence[Any], now: float, dt: float) -> None:
        if not self.cfg.enabled:
            return
        active = [v for v in vehicles if getattr(v, "active", True)]
        self.debug_counts["active_steps"] += len(active)
        for v in active:
            self._ensure_state(v, now)
            if bool(getattr(v, "is_lane_changing", False)):
                self._update_ongoing(v, now)
        self._apply_cooperative_yield(active, now, dt)
        for v in active:
            self._ensure_state(v, now)
            if self.should_consider(v, now):
                self.debug_counts["consider_checks"] += 1
                self.try_start(active, v, now)

    def _trajectory_state(self, v: Any, scene: Any, now: float) -> Optional[Tuple[float, float, float, float]]:
        if not bool(getattr(v, "is_lane_changing", False)):
            return None
        start = float(getattr(v, "lane_change_start_time"))
        duration = max(float(getattr(v, "lane_change_duration")), EPS)
        u = max(0.0, min(1.0, (now - start) / duration))
        source_x = float(getattr(v, "lane_change_start_x_norm"))
        target_x = float(getattr(v, "lane_change_target_x_norm"))
        dx_norm = target_x - source_x
        p = self.smoothstep5(u)
        dp = self.smoothstep5_d1(u)
        ddp = self.smoothstep5_d2(u)
        x_norm = source_x + dx_norm * p
        x_px = x_norm * float(scene.W)
        dx_px_dt = dx_norm * float(scene.W) * dp / duration
        dx_m_dt = dx_px_dt * float(scene.meters_per_pixel)
        dx_m_dtt = dx_norm * float(scene.W) * ddp / (duration * duration) * float(scene.meters_per_pixel)
        return x_px, dx_px_dt, dx_m_dt, dx_m_dtt

    def apply_render_pose(self, v: Any, scene: Any, now: float) -> None:
        """Override x and heading after rendering.update_pose(v, scene).

        runner.py should call:
            update_pose(v, scene)
            lane_change_model.apply_render_pose(v, scene, frame_time)

        This avoids pure parallel translation: heading follows the tangent of the lane-change path.
        """
        st = self._trajectory_state(v, scene, now)
        if st is None:
            # Ensure straight vehicles are exactly straight.
            v.heading_deg = 90.0 if _direction(v) == +1 else 270.0
            return

        x_px, dx_px_dt, dx_m_dt, dx_m_dtt = st
        v.x_px = float(x_px)
        # y_px is already set by update_pose from s_m.
        dy_px_dt = (_direction(v) * max(0.1, _v(v))) / max(float(scene.meters_per_pixel), EPS)
        raw_heading = math.degrees(math.atan2(dy_px_dt, dx_px_dt))
        if raw_heading < 0.0:
            raw_heading += 360.0

        base_heading = 90.0 if _direction(v) == +1 else 270.0
        # Limit visual yaw so the vehicle nose turns gently, not sideways.
        # Compute signed shortest-angle delta around base heading.
        delta = (raw_heading - base_heading + 180.0) % 360.0 - 180.0
        delta = max(-float(self.cfg.max_heading_delta_deg), min(float(self.cfg.max_heading_delta_deg), delta))
        target_heading = (base_heading + delta) % 360.0
        prev_heading = float(getattr(v, "lane_change_heading_deg", base_heading))
        # Smooth heading to avoid jitter.
        dprev = (target_heading - prev_heading + 180.0) % 360.0 - 180.0
        new_heading = (prev_heading + float(self.cfg.heading_smoothing_alpha) * dprev) % 360.0
        v.lane_change_heading_deg = new_heading
        v.heading_deg = new_heading
        v.lateral_speed_mps = float(dx_m_dt)
        v.lateral_acc_mps2 = float(dx_m_dtt)

    def save_log(self, output_dir: str) -> str:
        os.makedirs(output_dir, exist_ok=True)
        path = os.path.join(output_dir, self.cfg.log_file)
        fieldnames = [
            "event", "time_sec", "vehicle_id", "direction", "source_lane", "target_lane", "duration_sec",
            "current_gap_m", "target_front_gap_m", "target_rear_gap_m", "ego_acc_gain_mps2",
            "mobil_incentive_mps2", "target_rear_acc_after_mps2", "reason"
        ]
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for e in self.events:
                writer.writerow(e)
        debug_path = os.path.join(output_dir, "lane_change_debug_summary.json")
        with open(debug_path, "w", encoding="utf-8") as jf:
            json.dump(self.debug_counts, jf, ensure_ascii=False, indent=2)
        return path
