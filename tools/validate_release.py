#!/usr/bin/env python3
"""Validate all 15 SimMOT v1.0 sequences and write public-safe reports."""

from __future__ import annotations

import argparse
import configparser
import csv
import json
import math
import struct
from collections import Counter, defaultdict
from datetime import datetime, timezone
from itertools import zip_longest
from pathlib import Path


SEQUENCES = [
    f"R{road}_{lanes}lane_L{level}"
    for road, lanes in ((1, 3), (2, 5), (3, 7))
    for level in range(1, 6)
]
EXPECTED_FRAMES = 1800
EXPECTED_FPS = 30
EXPECTED_WIDTH = 1980
EXPECTED_HEIGHT = 1020
BOUND_TOLERANCE_PX = 0.011

VISIBLE_STATE_HEADER = [
    "frame", "id", "vehicle_type", "lane", "direction", "speed_kmh",
    "acc_mps2", "cx", "cy", "bbox_x", "bbox_y", "bbox_w", "bbox_h", "action",
]
FULL_STATE_HEADER = [
    "frame", "id", "active", "vehicle_type", "lane", "direction", "s_m",
    "speed_kmh", "desired_speed_kmh", "target_free_speed_kmh", "acc_mps2",
    "min_gap_m", "tau_sec", "max_acc_mps2", "comfort_dec_mps2",
    "max_dec_mps2", "leader_id", "leader_gap_m", "leader_speed_kmh", "action",
]


def jpeg_dimensions(path: Path) -> tuple[int, int]:
    """Read JPEG dimensions from the SOF marker without decoding pixel data."""
    sof_markers = {
        0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
        0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF,
    }
    with path.open("rb") as handle:
        if handle.read(2) != b"\xff\xd8":
            raise ValueError("missing JPEG SOI marker")
        while True:
            byte = handle.read(1)
            if not byte:
                raise ValueError("JPEG SOF marker not found")
            if byte != b"\xff":
                continue
            while byte == b"\xff":
                byte = handle.read(1)
            marker = byte[0]
            if marker in (0x01, *range(0xD0, 0xDA)):
                continue
            length_bytes = handle.read(2)
            if len(length_bytes) != 2:
                raise ValueError("truncated JPEG segment")
            length = struct.unpack(">H", length_bytes)[0]
            if length < 2:
                raise ValueError("invalid JPEG segment length")
            if marker in sof_markers:
                payload = handle.read(5)
                if len(payload) != 5:
                    raise ValueError("truncated JPEG SOF segment")
                height, width = struct.unpack(">HH", payload[1:])
                return width, height
            handle.seek(length - 2, 1)


def parse_seqinfo(path: Path) -> dict[str, int | str]:
    parser = configparser.ConfigParser()
    parser.read(path, encoding="utf-8")
    section = parser["Sequence"]
    return {
        "name": section["name"],
        "imDir": section["imDir"],
        "frameRate": int(section["frameRate"]),
        "seqLength": int(section["seqLength"]),
        "imWidth": int(section["imWidth"]),
        "imHeight": int(section["imHeight"]),
        "imExt": section["imExt"],
    }


def is_minus_one(value: str) -> bool:
    try:
        return float(value) == -1.0
    except ValueError:
        return False


def validate_mot_pair(
    gt_path: Path,
    det_path: Path,
    width: int,
    height: int,
) -> tuple[dict, list[str], Counter, dict[int, set[int]]]:
    issues: list[str] = []
    invalid_gt_rows = 0
    invalid_det_rows = 0
    bbox_nonfinite = 0
    bbox_nonpositive = 0
    bbox_outside = 0
    duplicate_frame_ids = 0
    det_gt_mismatches = 0
    frame_order_errors = 0
    gt_rows = 0
    det_rows = 0
    previous_frame = 0
    frame_counts: Counter[int] = Counter()
    id_frames: dict[int, set[int]] = defaultdict(set)
    seen_frame_ids: set[tuple[int, int]] = set()

    with gt_path.open("r", encoding="utf-8") as gt_handle, det_path.open(
        "r", encoding="utf-8"
    ) as det_handle:
        for line_number, pair in enumerate(zip_longest(gt_handle, det_handle), start=1):
            gt_line, det_line = pair
            if gt_line is not None:
                gt_rows += 1
            if det_line is not None:
                det_rows += 1
            if gt_line is None or det_line is None:
                det_gt_mismatches += 1
                continue

            gt = [part.strip() for part in gt_line.rstrip("\r\n").split(",")]
            det = [part.strip() for part in det_line.rstrip("\r\n").split(",")]
            if len(gt) != 10:
                invalid_gt_rows += 1
                continue
            if len(det) != 10:
                invalid_det_rows += 1
                continue

            try:
                frame = int(gt[0])
                vehicle_id = int(gt[1])
                x, y, w, h = (float(value) for value in gt[2:6])
                gt_score = float(gt[6])
                det_frame = int(det[0])
                det_id = int(det[1])
                det_score = float(det[6])
                det_bbox = tuple(float(value) for value in det[2:6])
            except ValueError:
                invalid_gt_rows += 1
                invalid_det_rows += 1
                continue

            if frame < previous_frame:
                frame_order_errors += 1
            previous_frame = frame
            if not 1 <= frame <= EXPECTED_FRAMES or vehicle_id <= 0 or gt_score != 1.0:
                invalid_gt_rows += 1
            if not all(is_minus_one(value) for value in gt[7:10]):
                invalid_gt_rows += 1
            if det_frame != frame or det_id != -1 or det_score != 1.0:
                invalid_det_rows += 1
            if not all(is_minus_one(value) for value in det[7:10]):
                invalid_det_rows += 1

            if not all(math.isfinite(value) for value in (x, y, w, h, *det_bbox)):
                bbox_nonfinite += 1
            if w <= 0.0 or h <= 0.0:
                bbox_nonpositive += 1
            if (
                x < -BOUND_TOLERANCE_PX
                or y < -BOUND_TOLERANCE_PX
                or x + w > width + BOUND_TOLERANCE_PX
                or y + h > height + BOUND_TOLERANCE_PX
            ):
                bbox_outside += 1

            key = (frame, vehicle_id)
            if key in seen_frame_ids:
                duplicate_frame_ids += 1
            seen_frame_ids.add(key)
            frame_counts[frame] += 1
            id_frames[vehicle_id].add(frame)

            if gt[0] != det[0] or gt[2:6] != det[2:6]:
                det_gt_mismatches += 1

    frame_coverage = sorted(frame_counts)
    missing_label_frames = sorted(set(range(1, EXPECTED_FRAMES + 1)) - set(frame_coverage))
    noncontiguous_ids = 0
    for frames in id_frames.values():
        if max(frames) - min(frames) + 1 != len(frames):
            noncontiguous_ids += 1

    checks = {
        "gt_row_count": gt_rows,
        "det_row_count": det_rows,
        "unique_vehicle_ids": len(id_frames),
        "label_frame_min": min(frame_coverage) if frame_coverage else None,
        "label_frame_max": max(frame_coverage) if frame_coverage else None,
        "missing_label_frame_count": len(missing_label_frames),
        "invalid_gt_row_count": invalid_gt_rows,
        "invalid_det_row_count": invalid_det_rows,
        "nonfinite_bbox_count": bbox_nonfinite,
        "nonpositive_bbox_count": bbox_nonpositive,
        "bbox_outside_image_count": bbox_outside,
        "duplicate_frame_id_count": duplicate_frame_ids,
        "noncontiguous_visible_id_count": noncontiguous_ids,
        "frame_order_error_count": frame_order_errors,
        "clean_det_gt_bbox_mismatch_count": det_gt_mismatches,
        "clean_det_matches_gt": det_gt_mismatches == 0 and gt_rows == det_rows,
    }
    for key, value in checks.items():
        if key.endswith("_count") and value != 0 and key not in {"gt_row_count", "det_row_count"}:
            issues.append(f"{key}={value}")
    if gt_rows != det_rows:
        issues.append(f"gt_row_count={gt_rows} differs from det_row_count={det_rows}")
    if not checks["clean_det_matches_gt"]:
        issues.append("clean detection frame/bbox values do not exactly match GT")
    return checks, issues, frame_counts, id_frames


def validate_state_files(
    sequence_dir: Path,
    gt_path: Path,
    expected_gt_rows: int,
) -> tuple[dict, list[str]]:
    issues: list[str] = []
    visible_path = sequence_dir / "state" / "ground_truth.csv"
    full_path = sequence_dir / "state" / "ground_truth_full.csv"
    lane_change_path = sequence_dir / "state" / "lane_change_log.csv"
    result = {
        "ground_truth_csv_present": visible_path.is_file(),
        "ground_truth_full_csv_present": full_path.is_file(),
        "lane_change_log_present": lane_change_path.is_file(),
        "ground_truth_csv_rows": 0,
        "ground_truth_full_csv_rows": 0,
        "visible_state_frame_id_mismatch_count": 0,
        "visible_state_bbox_difference_over_0_011px_count": 0,
        "full_state_invalid_frame_or_id_count": 0,
    }
    if not all((visible_path.is_file(), full_path.is_file(), lane_change_path.is_file())):
        issues.append("one or more required state files are missing")
        return result, issues

    with visible_path.open("r", encoding="utf-8", newline="") as visible_handle, gt_path.open(
        "r", encoding="utf-8"
    ) as gt_handle:
        reader = csv.DictReader(visible_handle)
        if reader.fieldnames != VISIBLE_STATE_HEADER:
            issues.append("ground_truth.csv header differs from the documented schema")
        for state_row, gt_line in zip_longest(reader, gt_handle):
            if state_row is None or gt_line is None:
                result["visible_state_frame_id_mismatch_count"] += 1
                continue
            result["ground_truth_csv_rows"] += 1
            gt = [part.strip() for part in gt_line.rstrip("\r\n").split(",")]
            if state_row["frame"] != gt[0] or state_row["id"] != gt[1]:
                result["visible_state_frame_id_mismatch_count"] += 1
            state_bbox = [float(state_row[key]) for key in ("bbox_x", "bbox_y", "bbox_w", "bbox_h")]
            gt_bbox = [float(value) for value in gt[2:6]]
            if any(abs(a - b) > BOUND_TOLERANCE_PX for a, b in zip(state_bbox, gt_bbox)):
                result["visible_state_bbox_difference_over_0_011px_count"] += 1

    with full_path.open("r", encoding="utf-8", newline="") as full_handle:
        reader = csv.DictReader(full_handle)
        if reader.fieldnames != FULL_STATE_HEADER:
            issues.append("ground_truth_full.csv header differs from the documented schema")
        for row in reader:
            result["ground_truth_full_csv_rows"] += 1
            try:
                frame = int(row["frame"])
                vehicle_id = int(row["id"])
                active = int(row["active"])
            except ValueError:
                result["full_state_invalid_frame_or_id_count"] += 1
                continue
            if not 1 <= frame <= EXPECTED_FRAMES or vehicle_id <= 0 or active != 1:
                result["full_state_invalid_frame_or_id_count"] += 1

    if result["ground_truth_csv_rows"] != expected_gt_rows:
        issues.append(
            "ground_truth.csv row count differs from GT: "
            f"{result['ground_truth_csv_rows']} != {expected_gt_rows}"
        )
    for key in (
        "visible_state_frame_id_mismatch_count",
        "visible_state_bbox_difference_over_0_011px_count",
        "full_state_invalid_frame_or_id_count",
    ):
        if result[key] != 0:
            issues.append(f"{key}={result[key]}")
    return result, issues


def validate_sequence(release: Path, sequence: str) -> tuple[dict, dict]:
    sequence_dir = release / sequence
    issues: list[str] = []
    seqinfo_path = sequence_dir / "seqinfo.ini"
    try:
        seqinfo = parse_seqinfo(seqinfo_path)
    except Exception as exc:  # report, do not hide malformed metadata
        seqinfo = {}
        issues.append(f"seqinfo.ini could not be parsed: {exc}")

    expected_seqinfo = {
        "name": sequence,
        "imDir": "img1",
        "frameRate": EXPECTED_FPS,
        "seqLength": EXPECTED_FRAMES,
        "imWidth": EXPECTED_WIDTH,
        "imHeight": EXPECTED_HEIGHT,
        "imExt": ".jpg",
    }
    if seqinfo != expected_seqinfo:
        issues.append("seqinfo.ini values differ from the SimMOT v1.0 contract")

    image_dir = sequence_dir / "img1"
    expected_names = [f"{index:06d}.jpg" for index in range(1, EXPECTED_FRAMES + 1)]
    actual_names = sorted(path.name for path in image_dir.glob("*.jpg")) if image_dir.is_dir() else []
    missing_names = sorted(set(expected_names) - set(actual_names))
    extra_names = sorted(set(actual_names) - set(expected_names))
    empty_images = 0
    invalid_dimensions = 0
    unreadable_images = 0
    for name in expected_names:
        path = image_dir / name
        if not path.is_file():
            continue
        if path.stat().st_size <= 0:
            empty_images += 1
            continue
        try:
            dimensions = jpeg_dimensions(path)
            if dimensions != (EXPECTED_WIDTH, EXPECTED_HEIGHT):
                invalid_dimensions += 1
        except Exception:
            unreadable_images += 1
    image_checks = {
        "expected_image_count": EXPECTED_FRAMES,
        "actual_image_count": len(actual_names),
        "missing_image_count": len(missing_names),
        "extra_image_count": len(extra_names),
        "empty_image_count": empty_images,
        "unreadable_jpeg_count": unreadable_images,
        "wrong_image_dimensions_count": invalid_dimensions,
        "filename_first": actual_names[0] if actual_names else None,
        "filename_last": actual_names[-1] if actual_names else None,
        "expected_width": EXPECTED_WIDTH,
        "expected_height": EXPECTED_HEIGHT,
    }
    for key in (
        "missing_image_count", "extra_image_count", "empty_image_count",
        "unreadable_jpeg_count", "wrong_image_dimensions_count",
    ):
        if image_checks[key] != 0:
            issues.append(f"{key}={image_checks[key]}")

    gt_path = sequence_dir / "gt" / "gt.txt"
    det_path = sequence_dir / "det" / "det.txt"
    if not gt_path.is_file() or not det_path.is_file():
        issues.append("gt/gt.txt or det/det.txt is missing")
        mot_checks = {
            "gt_row_count": 0,
            "det_row_count": 0,
            "unique_vehicle_ids": 0,
            "clean_det_matches_gt": False,
        }
        frame_counts: Counter[int] = Counter()
    else:
        mot_checks, mot_issues, frame_counts, _ = validate_mot_pair(
            gt_path, det_path, EXPECTED_WIDTH, EXPECTED_HEIGHT
        )
        issues.extend(mot_issues)

    state_checks, state_issues = validate_state_files(
        sequence_dir, gt_path, int(mot_checks["gt_row_count"])
    ) if gt_path.is_file() else ({}, ["state files were not checked because GT is missing"])
    issues.extend(state_issues)

    config_path = release / "configs" / "scenarios" / f"{sequence}.json"
    config_ok = False
    if config_path.is_file():
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config_ok = (
                config.get("dataset_version") == "SimMOT v1.0"
                and config.get("scenario_name") == sequence
            )
            if sequence == "R1_3lane_L4":
                demand = config["traffic_demand"]
                simulation = config["simulation"]
                config_ok = config_ok and (
                    simulation["num_vehicles"] == 58
                    and float(demand["flow_rate_vph_per_lane"]) == 960.0
                    and float(demand["spawn_headway_min_sec"]) == 1.6
                    and float(demand["spawn_headway_max_sec"]) == 5.2
                    and [float(value) for value in demand["initial_bumper_gap_m"]] == [7.0, 18.0]
                )
        except Exception:
            config_ok = False
    if not config_ok:
        issues.append("scenario configuration is missing or inconsistent")

    report = {
        "dataset": "SimMOT",
        "dataset_version": "1.0",
        "sequence": sequence,
        "validation_schema_version": "1.0",
        "passed": len(issues) == 0,
        "sequence_info": seqinfo,
        "images": image_checks,
        "mot_labels_and_clean_detections": mot_checks,
        "state_files": state_checks,
        "scenario_config_consistent": config_ok,
        "issues": issues,
    }
    road_group, lane_text, level = sequence.split("_")
    lanes_per_direction = int(lane_text.removesuffix("lane"))
    summary_row = {
        "sequence": sequence,
        "road_group": road_group,
        "lanes_per_direction": lanes_per_direction,
        "total_lanes": lanes_per_direction * 2,
        "traffic_level": level,
        "frames": EXPECTED_FRAMES,
        "fps": EXPECTED_FPS,
        "width": EXPECTED_WIDTH,
        "height": EXPECTED_HEIGHT,
        "duration_sec": f"{EXPECTED_FRAMES / EXPECTED_FPS:.1f}",
        "gt_box_count": mot_checks["gt_row_count"],
        "vehicle_id_count": mot_checks["unique_vehicle_ids"],
        "mean_objects_per_frame": f"{mot_checks['gt_row_count'] / EXPECTED_FRAMES:.4f}",
        "max_objects_per_frame": max(frame_counts.values(), default=0),
        "validation_passed": report["passed"],
    }
    return report, summary_row


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--release-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="SimMOT_v1.0 directory (defaults to the parent of tools/).",
    )
    args = parser.parse_args()
    release = args.release_root.resolve()

    reports = []
    rows = []
    for sequence in SEQUENCES:
        report, row = validate_sequence(release, sequence)
        reports.append(report)
        rows.append(row)
        report_path = release / sequence / "validation_report.json"
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"{sequence}: {'PASS' if report['passed'] else 'FAIL'}")

    fieldnames = list(rows[0].keys())
    with (release / "sequences.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    total_boxes = sum(int(row["gt_box_count"]) for row in rows)
    total_ids = sum(int(row["vehicle_id_count"]) for row in rows)
    validation_summary = {
        "dataset": "SimMOT",
        "dataset_version": "1.0",
        "generated_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
        "sequence_count": len(rows),
        "passed_sequence_count": sum(bool(report["passed"]) for report in reports),
        "all_sequences_passed": all(bool(report["passed"]) for report in reports),
        "total_frames": len(rows) * EXPECTED_FRAMES,
        "total_duration_sec": len(rows) * EXPECTED_FRAMES / EXPECTED_FPS,
        "total_gt_boxes": total_boxes,
        "total_visible_vehicle_trajectories": total_ids,
        "checks": [
            "1,800 JPEG frames per sequence with continuous six-digit filenames",
            "JPEG readability and 1980x1020 dimensions",
            "MOT row schema, frame range/order, finite positive in-image bbox values",
            "positive sequence-local GT IDs and no duplicate (frame,id) pairs",
            "clean detection ID=-1, score=1.0, and exact frame/bbox agreement with GT",
            "visible/all-active state CSV schemas and visible-state agreement with GT",
            "scenario metadata and the corrected R1_3lane_L4 parameters",
        ],
    }
    (release / "validation_summary.json").write_text(
        json.dumps(validation_summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(validation_summary, ensure_ascii=False, indent=2))
    if not validation_summary["all_sequences_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
