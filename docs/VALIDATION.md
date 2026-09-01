# Validation protocol

The same automated procedure is applied to all 15 sequences. Run it from the
release root:

```bash
python tools/validate_release.py
```

The validator checks:

1. Exactly 1,800 JPEG files named `000001.jpg` through `001800.jpg`.
2. JPEG readability and 1980 x 1020 dimensions for every frame.
3. `seqinfo.ini` values and one-based frame metadata.
4. Ten-column MOT row structure and nondecreasing frame IDs.
5. Finite bounding-box values, `w>0`, `h>0`, and image-bound compliance.
6. Positive GT IDs, no duplicate `(frame,id)` rows, and contiguous visible ID runs.
7. Clean-detection `id=-1`, `score=1.0`, and exact frame/bbox agreement with GT.
8. Visible and all-active state table schemas.
9. Visible-state row/ID/frame correspondence with MOT GT.
10. Scenario identity and the corrected `R1_3lane_L4` parameters.

Each sequence receives a concise `validation_report.json` containing only
release-relative facts and counts. No usernames, drive letters, or local paths
are written. `validation_summary.json` records the release totals and overall
pass status. `sequences.csv` is regenerated from the validated files.

After all release files are final, regenerate checksums with:

```bash
python tools/generate_sha256_manifest.py
```

`SHA256SUMS.csv` intentionally excludes itself and contains the hash, byte size,
and release-relative path of every other file.
