# Annotation and detection format

SimMOT uses a MOTChallenge-style directory layout and a MOT-style ten-column
text representation. This wording describes the layout and does not guarantee
drop-in compatibility with every MOTChallenge or TrackEval configuration.

## Coordinate system

- Image size: 1980 x 1020 pixels.
- Origin: upper-left image corner.
- Positive x direction: right.
- Positive y direction: down.
- `x,y`: upper-left corner of an axis-aligned bounding box.
- `w,h`: bounding-box width and height.
- Frame IDs: one-based (`1`-`1800`).
- MOT bounding-box precision: two decimal places.

The generator uses floating-point coordinates. It does not convert corners to
integers before computing width and height.

## State-derived theoretical box

Vehicle physical width and length are converted to image dimensions using
`vehicle_mpp=0.05 m/pixel`. Vehicle centers are mapped to the image plane using
`meters_per_pixel=0.10 m/pixel` for longitudinal motion and the configured lane
coordinates for lateral position.

The vehicle center, rendered width/length, and current heading define a rotated
rectangle. The minimum and maximum x/y values of its four vertices define the
axis-aligned enclosing box. This same rule applies to straight-driving and
lane-changing vehicles.

## Visibility and clipping

A label is written only if the vehicle center satisfies:

```text
0 <= cx < 1980
0 <= cy < 1020
```

Vehicles in the upstream/downstream simulation buffers continue to affect the
dynamics but are not labeled while their centers are outside the image.

When a center is inside but the vehicle body is partially outside, the
unrounded box corners are clipped to the continuous bounds
`[0,1980] x [0,1020]`. Boxes with non-positive clipped width or height are not
written. Independent two-decimal serialization can create a numerical boundary
difference of at most 0.01 pixel when `x+w` or `y+h` is recomputed from the text.

## Ground truth

`gt/gt.txt` uses:

```text
frame,id,x,y,w,h,score,-1,-1,-1
```

- `frame`: integer frame ID.
- `id`: positive sequence-local vehicle identity.
- `x,y,w,h`: theoretical bounding box in pixels.
- `score`: always `1`.
- final three fields: unused and always `-1`.

## Clean detections

`det/det.txt` uses the same ten-column layout, but its semantics differ:

- `id`: always `-1` because detections do not carry track identities.
- `score`: always `1.000000`.
- `frame,x,y,w,h`: identical to the corresponding GT row.
- final three fields: unused and always `-1`.

The `-1` detection ID is a format placeholder, not a real track identity.
This file contains clean, unperturbed detections rather than learned detector
outputs.
