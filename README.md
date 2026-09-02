# SimMOT v1.0

SimMOT is a synthetic UAV-view multi-vehicle tracking dataset with known
vehicle states and deterministic, state-derived theoretical ground-truth
labels. SimMOT v1.0 is the canonical name of this release.

## Release scope

This release contains 15 base sequences:

```text
R1_3lane_L1 ... R1_3lane_L5
R2_5lane_L1 ... R2_5lane_L5
R3_7lane_L1 ... R3_7lane_L5
```

R1, R2, and R3 contain 3, 5, and 7 lanes per direction (6, 10, and 14 total
lanes). L1-L5 denote the five traffic levels. Each sequence contains 1,800
frames at 30 frame/s, has a duration of 60 s, and uses 1980 x 1020 pixel images.
The full release contains 27,000 frames, 553,646 ground-truth boxes, 2,390
sequence-local visible vehicle trajectories, and 900 s of imagery.

`sequences.csv` is the single source for sequence-level counts. Do not maintain
these counts manually in downstream documentation.

The release contains the base data together with the controlled perturbation
utilities and frozen level definitions documented below. Pre-generated
perturbation TXT files, UAVDT experiments, tracker outputs, historical code,
audit packages, and other paper-engineering outputs are not part of the core
SimMOT v1.0 data.

## Distribution

The GitHub repository is intentionally lightweight: it contains the project
homepage, generator, configurations, documentation, tools, validation summaries,
and release metadata, but not the 15 sequence directories or their 27,000 JPEG
frames. SimMOT v1.0 is distributed as 15 sequence-level archives, one archive
per sequence:

```text
SimMOT_v1.0_R1_3lane_L1.zip ... SimMOT_v1.0_R1_3lane_L5.zip
SimMOT_v1.0_R2_5lane_L1.zip ... SimMOT_v1.0_R2_5lane_L5.zip
SimMOT_v1.0_R3_7lane_L1.zip ... SimMOT_v1.0_R3_7lane_L5.zip
```

Each ZIP contains exactly one top-level sequence directory and preserves that
sequence's internal layout without additional archive entries. Every ZIP is
smaller than 2 GB. Archive sizes and SHA-256 values are listed in
[RELEASE_PACKAGES.csv](RELEASE_PACKAGES.csv). The full-tree `SHA256SUMS.csv`
remains the canonical file manifest after the selected archives are extracted
directly into the lightweight repository root. Persistent download URLs/DOI must
be added after deposit; no temporary or invented identifier is supplied here.
See [docs/DISTRIBUTION.md](docs/DISTRIBUTION.md) for verification, extraction,
and hosting details.

## Sequence layout

```text
R1_3lane_L1/
  img1/
    000001.jpg
    ...
    001800.jpg
  gt/gt.txt
  det/det.txt
  state/ground_truth.csv
  state/ground_truth_full.csv
  state/lane_change_log.csv
  seqinfo.ini
  validation_report.json
```

Release-level perturbation resources are organized separately from the 15 core
sequences:

```text
configs/
  perturbations/
    paper_levels.json
docs/
  PERTURBATIONS.md
tools/
  perturbations/
    make_miss.py
    make_jitter.py
    make_shift.py
    make_scale.py
```

The JPEG frames are the primary image data. Videos are intentionally excluded
from the release and may be supplied separately as previews.

The published JPEGs were extracted from the clean rendered MP4 videos; they
were not produced by the generator's optional direct-frame output path. The
exact extraction command was not preserved, so the distributed JPEG files and
their SHA-256 hashes define the canonical image release. See
`docs/IMAGE_PREPARATION.md`.

## Ground-truth definition

State-derived theoretical ground-truth labels are deterministically generated
from the simulated vehicle states, vehicle geometry, heading, and
image-coordinate mapping. For every vehicle, including a lane-changing vehicle,
the generator constructs a rotated vehicle rectangle and then takes its
axis-aligned enclosing rectangle as the final bounding box.

A target is recorded only when its center satisfies `0 <= cx < 1980` and
`0 <= cy < 1020`. If the vehicle center is inside the image while part of its
body is outside, the box is clipped to the continuous image bounds
`[0,1980] x [0,1020]`. A clipped box is written only when `w > 0` and `h > 0`.

These labels are the theoretical reference under the stated SimMOT model. They
are not claimed to be physical ground truth for real UAV imagery.

## MOT files

Both files contain ten comma-separated fields:

```text
frame,id,x,y,w,h,score,-1,-1,-1
```

In `gt/gt.txt`, `id` is the positive sequence-local vehicle identity and
`score=1`. In `det/det.txt`, `id=-1` is an unused field and `score=1.0`; the
file is the clean, unperturbed detection input. Its frame and bounding-box
values correspond exactly, row by row, to `gt/gt.txt`.

Coordinates use pixels with the origin at the image's upper-left corner. `x,y`
are the box's upper-left coordinates, `w,h` are its width and height, and
`frame` starts at 1. MOT bounding-box values retain two decimal places. See
`docs/ANNOTATION_FORMAT.md` for the full convention.

The release uses a MOTChallenge-style tracking layout and a MOT-style ten-column
text format. Direct compatibility with every MOTChallenge/TrackEval tool is not
claimed because individual tools can assign different semantics to confidence,
class, and visibility columns. Convert or configure those fields when required
by a particular evaluation implementation.

## Controlled detection perturbations

SimMOT provides utilities for four controlled detection-input perturbations:
independent missed detections, center-only jitter, fixed two-dimensional shift,
and synchronized width/height scale perturbation. The 15 core sequences retain
only the clean `det/det.txt`; perturbation files are derived separately by the
scripts in `tools/perturbations/` and are not stored in the core sequence
directories. Definitions, levels, seeds, output rules, and examples are given
in [docs/PERTURBATIONS.md](docs/PERTURBATIONS.md).

## Vehicle identities

Each simulation run starts an independent, monotonically increasing identity
counter at 1. An assigned ID is never reused within the sequence. It remains
unchanged during car following and lane changes. Vehicles outside the image
retain their IDs while active but are absent from the visible GT until their
centers enter the image. After a vehicle exits downstream it is deactivated and
does not re-enter. A new sequence starts a new ID namespace, so IDs must not be
joined across sequences.

## Configurations

The exact 15 base scenario configurations are under `configs/scenarios/`.
`configs/scenario_parameter_table.csv` is generated from those JSON files.
For `R1_3lane_L4`, the actual configuration is:

```text
num_vehicles = 58
flow_rate_vph_per_lane = 960 veh/h/lane
spawn_headway = 1.6-5.2 s
initial_bumper_gap = 7-18 m
```

This corrected sequence-specific configuration takes precedence over older
summary tables.

Some exact generator model identifiers retain internal implementation tags such
as `v10` or `v11`. Those strings identify submodel implementations and are not
dataset version numbers; the only public dataset version is **SimMOT v1.0**.

## State tables and units

`state/ground_truth.csv` describes visible, labeled vehicles. The MOT GT and
this table have the same row count. `state/ground_truth_full.csv` describes all
active vehicles, including vehicles in the upstream and downstream simulation
buffers. Neither table alone contains every runtime property: heading is a
runtime field and physical length/width are defined in
`configs/vehicle_types_default.json`.

The principal units are: `s_m` and gap fields in metres, speed fields in km/h,
acceleration fields in m/s^2, `tau_sec` in seconds, bounding boxes and image
centers in pixels, flow in veh/h/lane, and frame rate in frame/s. The complete
field definitions are in `docs/STATE_FIELDS.md`.

## Reproducibility and validation

`generator/` contains only the final generator implementation used by these
base sequences and the author-created raster assets. It excludes historical
backups and paper-only perturbation experiments. See `generator/README.md`.
The exact original package versions were not archived; the known current test
environment and compatible dependency ranges are documented in
`docs/ENVIRONMENT.md`.

All 15 sequences are checked with one validation program:

```bash
python tools/validate_release.py
```

The validation covers frame names and dimensions, MOT schemas, finite and
positive in-image boxes, IDs, GT/clean-detection correspondence, state tables,
and scenario configuration consistency. `validation_summary.json` contains the
release-wide result; each sequence has its own path-free
`validation_report.json`. `SHA256SUMS.csv` provides the final file manifest.

## Asset provenance

Background and vehicle image assets used in SimMOT were created by the authors.
See `ASSET_PROVENANCE.md`.

## License

SimMOT uses separate licenses for source code and dataset content.

### Source code

The source code in this repository, including the dataset generator,
validation utilities, data-processing utilities, and controlled perturbation
tools, is released under the MIT License. See [`LICENSE`](LICENSE).

### Dataset and visual assets

The SimMOT dataset, including generated image sequences, ground-truth
annotations, clean detection inputs, vehicle-state files, sequence metadata,
validation data, dataset configurations, documentation, and author-created
visual assets, is released under the Creative Commons Attribution 4.0
International License (CC BY 4.0). See [`LICENSE-DATA`](LICENSE-DATA).

Users may use, redistribute, and adapt the CC BY 4.0 licensed material,
including for commercial purposes, provided that appropriate attribution is
given and any modifications are indicated.

For academic use, please also cite SimMOT using the information provided in
[`CITATION.cff`](CITATION.cff). A DOI or repository URL can be added after the
public archive is created.

## 中文说明

本目录是唯一的公开数据母版，正式名称为 **SimMOT v1.0**。公开范围仅包括
15组基础序列、理论真实标签、无扰动检测、状态表、生成配置、最终生成代码、
作者自制素材、验证文件、受控检测扰动工具和参数定义；不包括UAVDT实验、
预生成扰动输入、扰动实验结果、跟踪器输出、历史备份和其他论文工程文件。

许可证采用双许可结构：源代码使用MIT许可证；数据集、生成图像、标注、状态
数据、配置、文档和作者自制视觉素材使用CC BY 4.0许可证。详见根目录的
`LICENSE`和`LICENSE-DATA`。

GitHub主仓库仅包含代码、配置、文档、工具和数据元信息，不普通提交15个序列
及其27,000张JPEG。完整数据按15个正式序列分别打包为15个ZIP，每个ZIP仅含
一个序列且小于2 GB。文件大小和SHA-256见`RELEASE_PACKAGES.csv`，下载与
解压说明见`docs/DISTRIBUTION.md`。
