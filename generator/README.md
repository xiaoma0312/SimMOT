# SimMOT v1.0 generator

This directory contains the final code path corresponding to the 15 public base
sequences. Historical backups, stress tests, and paper-only perturbation
configurations are excluded.

Install dependencies:

```bash
python -m pip install -r requirements.txt
```

Run one base scenario from this directory:

```bash
python main.py --scenario-config ../configs/scenarios/R1_3lane_L1.json --asset-dir ./assets --background-image background.jpg --output-dir ./outputs/R1_3lane_L1
```

The generator's native output names include `gt_mot.txt` and `det_ideal.txt`.
In the published MOT layout these become `gt/gt.txt` and `det/det.txt`.

Random seed 7 is stored in every base scenario configuration. The generator
produces state-derived labels deterministically for that software environment;
encoded video/JPEG bytes can vary with platform and codec versions. The
published files and `SHA256SUMS.csv` are authoritative for byte-level identity.

The exact Python, NumPy, and OpenCV versions used for the original simulation
runs were not archived. `requirements.txt` therefore gives compatible ranges
rather than invented exact pins. See `../docs/ENVIRONMENT.md` for the current
tested release-preparation environment.

The optional `--save-frames` path writes direct rendered frames under
`frames_clean/`. It was not the preparation path for the published `img1/`
JPEGs, which were extracted from `synthetic_clean.mp4`. These two paths are not
expected to produce byte-identical images.

The bundled background and vehicle PNG assets were created by the authors.
