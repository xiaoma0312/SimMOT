# Controlled detection perturbations

This directory documents the controlled detection-input perturbations used with
SimMOT clean detections.

## Scope

The core SimMOT dataset remains unchanged. The scripts read each sequence's
clean `det/det.txt` and write derived perturbation files to a separate
`--output-root`.

The public utilities intentionally do **not** overwrite the core dataset.

## Perturbation definitions

### Missed detections

Each detection row is independently removed with probability `p`.

Standard paper levels:

- 0.05
- 0.10
- 0.20
- 0.30

### Random center jitter

Only the bounding-box center is perturbed:

`dcx ~ N(0, sigma^2)`

`dcy ~ N(0, sigma^2)`

Width and height remain unchanged.

Standard paper levels:

- 1 px
- 3 px
- 5 px
- 8 px

After perturbation, the complete axis-aligned bounding box is constrained to
the image region. Therefore boxes at the image boundary may experience a
smaller realized displacement than the sampled center displacement.

### Fixed position shift

The requested deterministic offset `(dx, dy)` is added to the box position.
Width and height remain unchanged.

Standard paper levels:

- `(5, 0)` px
- `(0, 5)` px
- `(5, 5)` px
- `(10, 10)` px

The box is then constrained to the image region. Near a boundary, the realized
offset can therefore be smaller than the requested offset.

### Random scale perturbation

For amplitude `a`, one scale factor is sampled independently for each box:

`s ~ Uniform(1-a, 1+a)`

The same factor is applied to width and height:

`w' = s * w`

`h' = s * h`

The original center is used before boundary clipping.

Standard paper amplitudes:

- 0.05
- 0.10
- 0.20

If a scaled box crosses the image boundary, it is clipped. Consequently the
final center and realized scale of a boundary box can change slightly.

## Random seeds

The historical experiments used frozen perturbation text files. Those frozen
files are the authoritative inputs for reproducing reported paper results.

For deterministic perturbation generation, the public scripts use an explicit
`--seed` together with the same MD5-based per-sequence, per-perturbation, and
per-level seed-derivation rule used in the historical seed-stability workflow.
This allows the released scripts to reproduce the historical `seed21` and
`seed42` perturbation inputs byte-for-byte for the sequences and perturbation
levels used in the reported experiments.

The frozen baseline perturbation files remain the authoritative baseline inputs
used for the reported experiments. The historical baseline generation did not
use one unified seed-derivation rule across all random perturbation types.

The seed-stability experiment used three samples described as:

- baseline
- seed21
- seed42

Only `miss30`, `jitter08`, and `scale20` were included in the random-sample
stability analysis. Fixed position shift is deterministic.

## Sequences used in the reported experiments

The paper perturbation comparison used:

- `R1_3lane_L3`
- `R1_3lane_L5`
- `R3_7lane_L3`

The public utilities can also be applied to any SimMOT sequence containing
`det/det.txt` (and `seqinfo.ini` where image dimensions are required).

## Examples

Generate all standard jitter levels for the three sequences used in the
reported experiments:

```bash
python tools/perturbations/make_jitter.py \
  --data-root /path/to/SimMOT_v1.0 \
  --output-root /path/to/SimMOT_derived \
  --sequences R1_3lane_L3 R1_3lane_L5 R3_7lane_L3 \
  --seed 21
```

Generate 30% missed detections only:

```bash
python tools/perturbations/make_miss.py \
  --data-root /path/to/SimMOT_v1.0 \
  --output-root /path/to/SimMOT_derived \
  --rates 0.30 \
  --seed 42
```

Generate the `(10,10)` px deterministic shift:

```bash
python tools/perturbations/make_shift.py \
  --data-root /path/to/SimMOT_v1.0 \
  --output-root /path/to/SimMOT_derived \
  --levels xy10
```

Generate the 20% scale-amplitude condition:

```bash
python tools/perturbations/make_scale.py \
  --data-root /path/to/SimMOT_v1.0 \
  --output-root /path/to/SimMOT_derived \
  --amplitudes 0.20 \
  --seed 21
```

## Important distinction

`generator/simulator/detection_noise.py`, when present in the simulator source,
is an optional simulator-level noise utility. It is not the implementation
used to generate the frozen controlled-perturbation inputs reported in the
paper.

The four utilities in `tools/perturbations/` document the audited perturbation
definitions used for those experiments.
