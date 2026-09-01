# -*- coding: utf-8 -*-
from __future__ import annotations

import csv
import json
import math
import os
from collections import Counter, defaultdict
from typing import Dict, List, Any, Optional

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None


def _safe_float(x, default: Optional[float] = None) -> Optional[float]:
    try:
        v = float(x)
        if math.isfinite(v):
            return v
        return default
    except Exception:
        return default


def _safe_int(x, default: int = 0) -> int:
    try:
        return int(float(x))
    except Exception:
        return default


def _read_json(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _count_lines(path: str) -> int:
    if not os.path.exists(path):
        return 0
    with open(path, "r", encoding="utf-8") as f:
        return sum(1 for _ in f)


def _stats(values: List[float]) -> Dict[str, Any]:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]

    if arr.size == 0:
        return {
            "count": 0,
            "min": None,
            "p01": None,
            "p05": None,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
        }

    return {
        "count": int(arr.size),
        "min": round(float(np.min(arr)), 4),
        "p01": round(float(np.percentile(arr, 1)), 4),
        "p05": round(float(np.percentile(arr, 5)), 4),
        "mean": round(float(np.mean(arr)), 4),
        "p50": round(float(np.percentile(arr, 50)), 4),
        "p95": round(float(np.percentile(arr, 95)), 4),
        "p99": round(float(np.percentile(arr, 99)), 4),
        "max": round(float(np.max(arr)), 4),
    }


def _ratio_dict(counter: Counter) -> Dict[str, Dict[str, Any]]:
    total = sum(counter.values())
    out = {}
    for k, v in sorted(counter.items(), key=lambda x: (-x[1], x[0])):
        out[k] = {
            "count": int(v),
            "ratio": round(float(v) / total, 6) if total > 0 else 0.0,
        }
    return out


def _get_image_size(output_dir: str):
    bg_path = os.path.join(output_dir, "background_used.jpg")
    if cv2 is None or not os.path.exists(bg_path):
        return None, None

    img = cv2.imread(bg_path)
    if img is None:
        return None, None

    h, w = img.shape[:2]
    return int(w), int(h)


def _analyze_ground_truth(output_dir: str, image_w: Optional[int], image_h: Optional[int]) -> Dict[str, Any]:
    gt_csv = os.path.join(output_dir, "ground_truth.csv")

    frame_counts = Counter()
    visible_ids = set()
    action_counts = Counter()
    vehicle_type_counts = Counter()
    lane_counts = Counter()

    speed_kmh = []
    acc_mps2 = []
    bbox_w = []
    bbox_h = []
    bbox_area = []
    bbox_aspect = []

    bbox_outside_count = 0
    bbox_nonpositive_count = 0
    total_rows = 0

    if not os.path.exists(gt_csv):
        return {"available": False, "reason": "ground_truth.csv not found"}

    with open(gt_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            total_rows += 1

            frame = _safe_int(row.get("frame"), 0)
            vid = _safe_int(row.get("id"), -1)
            lane = _safe_int(row.get("lane"), -1)

            frame_counts[frame] += 1
            visible_ids.add(vid)
            lane_counts[str(lane)] += 1

            vehicle_type = row.get("vehicle_type", "unknown")
            action = row.get("action", "unknown")
            vehicle_type_counts[vehicle_type] += 1
            action_counts[action] += 1

            sp = _safe_float(row.get("speed_kmh"))
            ac = _safe_float(row.get("acc_mps2"))
            if sp is not None:
                speed_kmh.append(sp)
            if ac is not None:
                acc_mps2.append(ac)

            x = _safe_float(row.get("bbox_x"))
            y = _safe_float(row.get("bbox_y"))
            w = _safe_float(row.get("bbox_w"))
            h = _safe_float(row.get("bbox_h"))

            if w is not None and h is not None:
                bbox_w.append(w)
                bbox_h.append(h)
                bbox_area.append(w * h)
                if h > 1e-6:
                    bbox_aspect.append(w / h)

                if w <= 0 or h <= 0:
                    bbox_nonpositive_count += 1

            if image_w is not None and image_h is not None:
                if x is not None and y is not None and w is not None and h is not None:
                    if x < 0 or y < 0 or x + w > image_w or y + h > image_h:
                        bbox_outside_count += 1

    visible_per_frame = list(frame_counts.values())

    # 尺度变化统计：后面你做近远距离尺寸函数时，这个指标很有用
    bw_stats = _stats(bbox_w)
    bh_stats = _stats(bbox_h)
    area_stats = _stats(bbox_area)

    scale_ratio_w = None
    scale_ratio_area = None
    if bw_stats["p05"] not in (None, 0) and bw_stats["p95"] is not None:
        scale_ratio_w = round(float(bw_stats["p95"]) / float(bw_stats["p05"]), 4)
    if area_stats["p05"] not in (None, 0) and area_stats["p95"] is not None:
        scale_ratio_area = round(float(area_stats["p95"]) / float(area_stats["p05"]), 4)

    return {
        "available": True,
        "visible_label_rows": int(total_rows),
        "visible_unique_ids": int(len(visible_ids)),
        "frames_with_visible_targets": int(len(frame_counts)),
        "visible_targets_per_frame": _stats(visible_per_frame),
        "speed_kmh_visible": _stats(speed_kmh),
        "acc_mps2_visible": _stats(acc_mps2),
        "vehicle_type_distribution_visible": _ratio_dict(vehicle_type_counts),
        "lane_distribution_visible": _ratio_dict(lane_counts),
        "action_distribution_visible": _ratio_dict(action_counts),
        "bbox": {
            "width_px": bw_stats,
            "height_px": bh_stats,
            "area_px2": area_stats,
            "aspect_ratio": _stats(bbox_aspect),
            "bbox_outside_image_count": int(bbox_outside_count),
            "bbox_outside_image_ratio": round(bbox_outside_count / total_rows, 6) if total_rows > 0 else 0.0,
            "bbox_nonpositive_count": int(bbox_nonpositive_count),
            "scale_ratio_width_p95_over_p05": scale_ratio_w,
            "scale_ratio_area_p95_over_p05": scale_ratio_area,
        },
    }


def _analyze_full_states(output_dir: str) -> Dict[str, Any]:
    full_csv = os.path.join(output_dir, "ground_truth_full.csv")

    if not os.path.exists(full_csv):
        return {"available": False, "reason": "ground_truth_full.csv not found"}

    speed_all = []
    acc_all = []
    target_speed = []
    desired_speed = []
    leader_gap = []
    action_counts = Counter()

    with open(full_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sp = _safe_float(row.get("speed_kmh"))
            ac = _safe_float(row.get("acc_mps2"))
            ts = _safe_float(row.get("target_free_speed_kmh"))
            ds = _safe_float(row.get("desired_speed_kmh"))
            gap = _safe_float(row.get("leader_gap_m"))

            if sp is not None:
                speed_all.append(sp)
            if ac is not None:
                acc_all.append(ac)
            if ts is not None:
                target_speed.append(ts)
            if ds is not None:
                desired_speed.append(ds)
            if gap is not None:
                leader_gap.append(gap)

            action_counts[row.get("action", "unknown")] += 1

    return {
        "available": True,
        "speed_kmh_all_active": _stats(speed_all),
        "acc_mps2_all_active": _stats(acc_all),
        "target_free_speed_kmh": _stats(target_speed),
        "desired_speed_kmh": _stats(desired_speed),
        "leader_gap_m_from_full_state": _stats(leader_gap),
        "action_distribution_all_active": _ratio_dict(action_counts),
    }


def _analyze_gap_log(output_dir: str) -> Dict[str, Any]:
    gap_csv = os.path.join(output_dir, "gap_log.csv")

    if not os.path.exists(gap_csv):
        return {"available": False, "reason": "gap_log.csv not found"}

    bumper_gaps = []
    visual_gaps = []
    negative_gap_count = 0
    very_small_gap_count = 0

    with open(gap_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            gap = _safe_float(row.get("bumper_gap_m"))
            vgap = _safe_float(row.get("visual_bumper_gap_px"))

            if gap is not None:
                bumper_gaps.append(gap)
                if gap < -0.05:
                    negative_gap_count += 1
                if gap < 0.5:
                    very_small_gap_count += 1

            if vgap is not None:
                visual_gaps.append(vgap)

    return {
        "available": True,
        "bumper_gap_m": _stats(bumper_gaps),
        "visual_bumper_gap_px": _stats(visual_gaps),
        "negative_gap_count": int(negative_gap_count),
        "very_small_gap_lt_0_5m_count": int(very_small_gap_count),
        "gap_rows": int(len(bumper_gaps)),
    }


def _analyze_detection_files(output_dir: str) -> Dict[str, Any]:
    ideal_path = os.path.join(output_dir, "det_ideal.txt")
    noisy_path = os.path.join(output_dir, "det_noisy.txt")
    gt_path = os.path.join(output_dir, "gt_mot.txt")

    ideal_count = _count_lines(ideal_path)
    noisy_count = _count_lines(noisy_path)
    gt_count = _count_lines(gt_path)

    return {
        "gt_mot_rows": int(gt_count),
        "det_ideal_rows": int(ideal_count),
        "det_noisy_rows": int(noisy_count),
        "det_noisy_over_ideal_ratio": round(noisy_count / ideal_count, 6) if ideal_count > 0 else None,
    }


def generate_validation_report(output_dir: str) -> str:
    """
    Read simulation outputs and generate validation_report.json.

    This report is used to verify simulation sanity:
    - motion statistics
    - car-following gap statistics
    - bbox validity
    - visible target density
    - detection file counts
    - warnings for potential problems
    """

    config = _read_json(os.path.join(output_dir, "config.json"))
    safety_summary = _read_json(os.path.join(output_dir, "safety_summary.json"))

    image_w, image_h = _get_image_size(output_dir)

    gt_analysis = _analyze_ground_truth(output_dir, image_w, image_h)
    full_analysis = _analyze_full_states(output_dir)
    gap_analysis = _analyze_gap_log(output_dir)
    det_analysis = _analyze_detection_files(output_dir)

    warnings = []

    # 1. 车距检查
    if gap_analysis.get("available"):
        neg_gap = gap_analysis.get("negative_gap_count", 0)
        min_gap = gap_analysis.get("bumper_gap_m", {}).get("min")
        if neg_gap > 0:
            warnings.append(f"Detected {neg_gap} negative bumper-gap rows. Check no-overlap logic.")
        if min_gap is not None and min_gap < -0.05:
            warnings.append(f"Minimum bumper gap is {min_gap} m, which indicates possible overlap.")

    # 2. bbox 检查
    if gt_analysis.get("available"):
        bbox_info = gt_analysis.get("bbox", {})
        if bbox_info.get("bbox_nonpositive_count", 0) > 0:
            warnings.append("Found non-positive bbox width or height.")
        outside_ratio = bbox_info.get("bbox_outside_image_ratio", 0.0)
        # 部分目标从画面边缘进入/离开时 bbox 越界是正常的，所以这里只做提醒，不直接判错。
        if outside_ratio > 0.2:
            warnings.append(
                f"High bbox outside-image ratio: {outside_ratio}. "
                "This may be normal for entering/leaving vehicles, but check the visible range."
            )

    # 3. 加速度检查
    if full_analysis.get("available"):
        acc_stats = full_analysis.get("acc_mps2_all_active", {})
        min_acc = acc_stats.get("min")
        max_acc = acc_stats.get("max")
        if min_acc is not None and min_acc < -8.0:
            warnings.append(f"Very large braking acceleration detected: {min_acc} m/s^2.")
        if max_acc is not None and max_acc > 5.0:
            warnings.append(f"Very large acceleration detected: {max_acc} m/s^2.")

    # 4. 检测文件检查
    if det_analysis.get("det_ideal_rows", 0) == 0:
        warnings.append("det_ideal.txt is empty.")
    if det_analysis.get("gt_mot_rows", 0) == 0:
        warnings.append("gt_mot.txt is empty.")

    report = {
        "report_name": "validation_report",
        "description": (
            "Automatic validation report for the 2D UAV-view traffic simulation. "
            "This report checks simulation sanity, label validity, motion statistics, "
            "car-following gaps, bbox scale, and detection file consistency."
        ),
        "output_dir": os.path.abspath(output_dir),
        "image_size": {
            "width": image_w,
            "height": image_h,
        },
        "scenario": {
            "scenario_config": config.get("scenario_config"),
            "scenario_name": (
                config.get("scenario_config_data", {}).get("scenario_name")
                if isinstance(config.get("scenario_config_data"), dict)
                else None
            ),
            "num_lanes": safety_summary.get("num_lanes"),
            "visible_length_m": safety_summary.get("visible_length_m"),
            "meters_per_pixel": safety_summary.get("meters_per_pixel"),
            "vehicle_mpp": safety_summary.get("vehicle_mpp"),
        },
        "ground_truth_visible": gt_analysis,
        "ground_truth_all_active": full_analysis,
        "car_following_gap": gap_analysis,
        "detection_files": det_analysis,
        "warnings": warnings,
        "passed_basic_validation": len(warnings) == 0,
        "note": (
            "Warnings do not necessarily mean the simulation is invalid. "
            "For example, bbox outside-image rows may appear when vehicles enter or leave the frame."
        ),
    }

    out_path = os.path.join(output_dir, "validation_report.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    return out_path