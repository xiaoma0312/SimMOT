# State data fields and units

SimMOT provides two complementary state tables. They must not be described as
if either one were a complete dump of every runtime property.

## `ground_truth.csv`

This table contains one row for every visible target written to `gt/gt.txt`.
Its bounding boxes retain three decimal places, whereas MOT boxes retain two.

| Field | Unit / values | Definition |
|---|---|---|
| `frame` | frame | One-based frame ID. |
| `id` | integer | Positive sequence-local vehicle ID. |
| `vehicle_type` | category | `passenger car`, `taxi`, `other`, `bus`, or `large bus`. |
| `lane` | integer | Zero-based global lane ID within the sequence. |
| `direction` | `D` / `U` | `D`: image top to bottom; `U`: image bottom to top. |
| `speed_kmh` | km/h | Vehicle longitudinal speed. |
| `acc_mps2` | m/s^2 | Vehicle longitudinal acceleration. |
| `cx`, `cy` | pixel | Vehicle center in image coordinates. |
| `bbox_x`, `bbox_y` | pixel | Clipped axis-aligned box upper-left position. |
| `bbox_w`, `bbox_h` | pixel | Clipped axis-aligned box width and height. |
| `action` | category | Internal categorical state assigned by the simulator at the current frame. It is provided for inspection and is not part of the MOT evaluation annotation. |

For a sequence with `n` lanes per direction, lane IDs `0` through `n-1` are
top-to-bottom lanes and IDs `n` through `2n-1` are bottom-to-top lanes.

## `ground_truth_full.csv`

This table contains every active vehicle at every frame, including vehicles
whose centers are outside the image but inside the simulation buffers.

| Field | Unit / values | Definition |
|---|---|---|
| `frame` | frame | One-based frame ID. |
| `id` | integer | Sequence-local vehicle ID. |
| `active` | `1` | Vehicle is active; inactive vehicles are not written. |
| `vehicle_type` | category | Vehicle appearance/parameter class. |
| `lane` | integer | Zero-based global lane ID. During a lane change, this remains the source lane until completion. |
| `direction` | `D` / `U` | Direction of travel. |
| `s_m` | m | Longitudinal road coordinate, increasing in the direction of travel. |
| `speed_kmh` | km/h | Current longitudinal speed. |
| `desired_speed_kmh` | km/h | Vehicle-specific desired speed. |
| `target_free_speed_kmh` | km/h | Current free-flow target speed. |
| `acc_mps2` | m/s^2 | Current longitudinal acceleration. |
| `min_gap_m` | m | Vehicle-specific minimum gap parameter. |
| `tau_sec` | s | Vehicle-specific desired time-headway parameter. |
| `max_acc_mps2` | m/s^2 | Vehicle-specific maximum acceleration parameter. |
| `comfort_dec_mps2` | m/s^2 | Comfortable deceleration magnitude parameter. |
| `max_dec_mps2` | m/s^2 | Maximum/emergency deceleration magnitude parameter. |
| `leader_id` | integer / `-1` | Current leader ID, or `-1` when none is assigned. |
| `leader_gap_m` | m / `inf` | Bumper-to-bumper longitudinal gap to the leader. |
| `leader_speed_kmh` | km/h / `nan` | Leader speed, or `nan` when no leader exists. |
| `action` | category | Internal categorical state assigned by the simulator at the current frame. It is provided for inspection and is not part of the MOT evaluation annotation. |

Action strings such as `follow_decelerate`, `follow_match`, `free_accelerate`,
`lane_change_to_*`, and `safety_clamp_no_overlap` are generator diagnostics.
They are not a closed behavioral taxonomy, a supervised class label set, or a
variable used by the MOT-format evaluation.

## `lane_change_log.csv`

This event table records lane-change starts and completions. Decision diagnostics
are populated on `start` rows and are blank on `complete` rows. `inf` denotes
that no corresponding front or rear vehicle was present within the evaluated
lane context.

| Field | Unit / values | Definition |
|---|---|---|
| `event` | `start` / `complete` | Lane-change event type. A start near the end of a sequence may have no completion row within the 60 s window. |
| `time_sec` | s | Simulation time at which the event was logged. |
| `vehicle_id` | integer | Sequence-local ID of the lane-changing vehicle. |
| `direction` | `D` / `U` | Vehicle travel direction. |
| `source_lane` | integer | Zero-based global lane ID before the maneuver. |
| `target_lane` | integer | Zero-based adjacent lane ID selected for the maneuver. |
| `duration_sec` | s | Vehicle-specific planned duration of the smooth lane-change trajectory. |
| `current_gap_m` | m / blank / `inf` | Front gap in the current lane at the start decision. |
| `target_front_gap_m` | m / blank / `inf` | Front gap in the target lane at the start decision. |
| `target_rear_gap_m` | m / blank / `inf` | Rear gap in the target lane at the start decision. |
| `ego_acc_gain_mps2` | m/s^2 / blank | Predicted ego-vehicle acceleration gain associated with the target lane. |
| `mobil_incentive_mps2` | m/s^2 / blank | MOBIL-style incentive score used by the lane-change decision. |
| `target_rear_acc_after_mps2` | m/s^2 / blank | Predicted acceleration of the target-lane rear vehicle after the merge. |
| `reason` | text | Plus-delimited internal decision triggers; copied to the completion event. |

## Properties stored elsewhere

- `heading_deg` exists at runtime but is not a column in either state CSV.
- Physical vehicle `length_m` and `width_m` are defined by vehicle type in
  `configs/vehicle_types_default.json`.
- Lane-change start/completion events are in `lane_change_log.csv`.
- MOT box coordinates are the authoritative two-decimal public tracking labels.

## Other units

| Quantity | Unit |
|---|---|
| `meters_per_pixel` | m/pixel |
| `vehicle_mpp` | m/pixel |
| flow | veh/h/lane |
| frame rate | frame/s |
| heading in generator runtime | degree |
