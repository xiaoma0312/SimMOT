# Distribution and download layout

## Lightweight GitHub repository

The GitHub repository contains source code, the small author-created generator
assets, configurations, documentation, tools, licences, validation summaries,
and data/release metadata. The 15 sequence directories and their 27,000 JPEG
frames are not committed to ordinary Git history and do not require Git LFS.

The bundled `generator/assets/` directory contains 23 files and is approximately
0.46 MiB, so it remains part of the lightweight repository. A separate generator
asset archive is not required.

`SHA256SUMS.csv` describes the complete reconstructed SimMOT v1.0 release tree,
including sequence files that are intentionally absent from a Git clone.

## Full data archives

The complete sequence data are packaged by road scale:

| Archive | Sequences | Size (GiB) |
|---|---|---:|
| `SimMOT_v1.0_R1_3lane.zip` | `R1_3lane_L1`-`R1_3lane_L5` | 3.963 |
| `SimMOT_v1.0_R2_5lane.zip` | `R2_5lane_L1`-`R2_5lane_L5` | 4.083 |
| `SimMOT_v1.0_R3_7lane.zip` | `R3_7lane_L1`-`R3_7lane_L5` | 4.203 |

Each ZIP is self-contained with respect to its data licence and integrity
metadata. It contains:

```text
LICENSE-DATA
VERSION
PACKAGE_SHA256SUMS_<road>.csv
<five sequence directories>/
```

The archive-level SHA-256 values are recorded in `RELEASE_PACKAGES.csv` and in
the separately distributed `PACKAGE_ARCHIVES_SHA256.csv` file.

## Hosting boundary

All three road archives exceed GitHub's documented
[2 GiB per-release-asset limit](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases#storage-and-bandwidth-quotas)
and therefore must not be attached directly to a GitHub Release. Git LFS has
separate plan-dependent per-file limits, but those limits do not change the
ordinary GitHub Release asset limit described above.

The archives are intended for a Zenodo dataset record. Together they comprise
three files totalling 13,152,384,209 bytes (approximately 13.15 GB), within
Zenodo's default allowance of
[up to 100 files and 50 GB per record](https://help.zenodo.org/docs/deposit/manage-files/).
Zenodo provides the persistent landing page and DOI required for the formal
dataset release.

Persistent download URLs and the dataset DOI are intentionally left unset until
the Zenodo deposit exists. After deposit, add the assigned identifiers to this
document and `CITATION.cff`; do not substitute temporary cloud links.

## Reconstruction

Clone or download the lightweight `SimMOT` repository, then extract any or all
road archives directly into that repository root. Extracting all three adds the
15 sequence directories beside `generator/`, `configs/`, `docs/`, and `tools/`,
reconstructing the complete release tree without renaming. Identical copies of
`LICENSE-DATA` and `VERSION` may be safely overwritten. The repository
`.gitignore` prevents the extracted sequence directories from being staged by
ordinary Git operations.

After extraction, verify the complete tree using:

```bash
python tools/validate_release.py
```

For byte-level verification, compare files against `SHA256SUMS.csv`. Each road
archive also carries its own internal `PACKAGE_SHA256SUMS_<road>.csv` for
independent per-file verification.
