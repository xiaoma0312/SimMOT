# Software environment record

## Original generation environment

The exact Python, NumPy, OpenCV, and codec package versions used for the original
SimMOT simulation runs and MP4 writing were not archived. SimMOT v1.0 therefore
does not claim an exact historical environment lockfile. No version numbers are
back-filled from guesswork.

The generator requires Python 3.10 or newer because its source uses modern type
annotation syntax. Compatible dependency ranges are recorded in
`generator/requirements.txt`:

```text
numpy>=1.24,<3.0
opencv-python>=4.8,<5.0
```

These are compatibility ranges, not a claim about the exact original versions.

## Current release-preparation test environment

The generator command-line interface, release validator, and packaging tools
were tested during preparation of this local release with:

```text
Python 3.12.7 (Anaconda build)
NumPy 1.26.4
opencv-python 4.12.0.88 (cv2 reports 4.12.0)
```

This record demonstrates a currently working inspection environment. It does
not replace the missing historical generation record.

## Reproducibility boundary

State updates and label geometry are controlled by the released configurations
and random seeds. Byte-identical MP4/JPEG rendering is not guaranteed across
operating systems, OpenCV builds, encoders, or codec versions. The distributed
data files and `SHA256SUMS.csv` are authoritative for byte-level identity.
