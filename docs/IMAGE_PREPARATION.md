# Published image preparation

The SimMOT v1.0 image preparation chain was:

```text
simulation rendering and synchronized label generation
                         |
                         v
              synthetic_clean.mp4
                         |
                         v
             video frame extraction
                         |
                         v
          000001.jpg ... 001800.jpg
                         |
                         v
                 published img1/
```

The simulations, vehicle states, ground truth, configurations, and clean videos
were generated on the author's computer. AutoDL was used only to split the clean
videos into image frames; it was not the simulation or label-generation
environment.

The exact frame-extraction command and software version were not preserved and
are therefore not reconstructed here. Published JPEG frames were extracted from
the clean rendered videos. The distributed image files and SHA-256 manifest
define the canonical SimMOT v1.0 image release.

The generator also exposes an optional `--save-frames` mode that writes direct
rendered frames. That mode was not used to produce the published `img1/` files.
Directly written frames bypass video encoding and therefore are not expected to
be pixel- or byte-identical to JPEGs extracted after MP4 encoding. This
difference does not change the synchronized state-derived box coordinates, but
it matters for exact image bytes and hashes.

No image regeneration is required for v1.0. The existing 27,000 distributed
JPEG files are the authoritative image data used by this release.
