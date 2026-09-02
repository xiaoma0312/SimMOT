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

## Sequence-level data archives

SimMOT v1.0 is distributed as 15 sequence-level archives, one archive per
sequence:

| Archive pattern | Sequence levels | ZIP size range (bytes) |
|---|---|---:|
| `SimMOT_v1.0_R1_3lane_L[1-5].zip` | `R1_3lane_L1`-`R1_3lane_L5` | 824,289,342-875,055,452 |
| `SimMOT_v1.0_R2_5lane_L[1-5].zip` | `R2_5lane_L1`-`R2_5lane_L5` | 841,483,041-921,973,824 |
| `SimMOT_v1.0_R3_7lane_L[1-5].zip` | `R3_7lane_L1`-`R3_7lane_L5` | 853,185,215-952,005,387 |

Each ZIP contains exactly one top-level sequence directory:

```text
<sequence>/
├── det/
├── gt/
├── img1/
├── state/
├── seqinfo.ini
├── sequence_meta.json
└── validation_report.json
```

No licence, package manifest, or second sequence is inserted into an individual
ZIP, so the sequence-internal directory structure remains identical to the
canonical release tree. The data licence is supplied at repository and deposit
level in `LICENSE-DATA`.

Archive filenames, exact byte sizes, and SHA-256 values are recorded in both
`RELEASE_PACKAGES.csv` and the separately distributed
`PACKAGE_ARCHIVES_SHA256.csv`. Each table has the columns `filename`,
`size_bytes`, and `sha256`.

## Integrity and size checks

All 15 archives were checked by fully reading and decompressing every ZIP entry,
verifying CRC integrity, comparing the archive file set with the corresponding
source sequence, and comparing every extracted file's size and SHA-256 with the
canonical `SHA256SUMS.csv`. Each archive contains 1,807 files.

The smallest archive is 824,289,342 bytes and the largest is 952,005,387 bytes.
All are below both 2,000,000,000 bytes (2 GB, decimal) and 2 GiB.

## Hosting

Because every archive is below GitHub's documented
[2 GiB per-release-asset limit](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases#storage-and-bandwidth-quotas),
the 15 ZIP files may be attached to a GitHub Release. They should not be
committed to ordinary Git history.

The archives together total 13,150,975,169 bytes (approximately 13.15 GB), so
they may alternatively be deposited together in one Zenodo dataset record,
within Zenodo's default allowance of
[up to 100 files and 50 GB per record](https://help.zenodo.org/docs/deposit/manage-files/).
Zenodo is suitable when a persistent dataset DOI is required.

Persistent download URLs and the dataset DOI are intentionally left unset until
the selected deposit exists. After deposit, add the assigned identifiers to this
document and `CITATION.cff`; do not substitute temporary cloud links.

## Reconstruction

Clone or download the lightweight `SimMOT` repository, then extract any selected
sequence ZIP directly into that repository root. Extracting all 15 archives adds
the 15 sequence directories beside `generator/`, `configs/`, `docs/`, and
`tools/`, reconstructing the complete release tree without renaming. The
repository `.gitignore` prevents the extracted sequence directories from being
staged by ordinary Git operations.

After extraction, verify the complete tree using:

```bash
python tools/validate_release.py
```

For byte-level verification, first compare the downloaded ZIP files against
`PACKAGE_ARCHIVES_SHA256.csv`, then compare extracted files against
`SHA256SUMS.csv`.
