# IMX283 experimental crop modes

This branch is an experimental extension of `cinemate-dev`.

It preserves the existing 12-bit and 10-bit readout modes, adds per-mode V4L2 crop reporting compatible with the approach used by the IMX585 experimental cropped-modes driver, and adds native 12-bit binned readout modes alongside fixed centered Mode-0 crops.

## Added Mode-0 12-bit non-binned crops

- 5184x3456
- 4608x3072
- 4096x2732
- 4000x2668
- 3936x2624
- 4096x2304
- 4096x2160
- 3936x2214
- 3936x2176
- 3840x2160
- 4096x2048
- 3840x1920
- 4096x1716
- 3840x1608
- 3000x3000
- 2736x1824
- 2048x1152
- 1920x1080

The crop variants all use IMX283 Mode 0:
- 12-bit
- 1x1, no binning
- centered crop of the 5472x3648 active area

The branch enables the sensor's arbitrary vertical-crop registers (MDSEL3/MDSEL4, VWINPOS and VWIDCUT) for Mode 0.

This is a hardware-validation branch. The crop dimensions/alignment and resulting timing should be verified on the actual sensor before treating every advertised resolution as production-ready.

The original `cinemate-dev` branch is not modified.

## Sensor-mode reporting

The driver now keeps the V4L2 TRY crop selection synchronized with the selected mode. This is important for libcamera: the reported output format (which may include optical-black pixels) and the sensor active crop are distinct pieces of information, and both must change together when a fixed crop mode is selected.

The mode table also records the IMX283 horizontal/vertical binning ratios. Current native 12-bit additions are:

- Mode 2A: 2736x1538 active pixels, 2x2 binning
- Mode 3: 1824x1216 active pixels, 3x3 binning

Sony documents additional 12-bit readout modes 4 (1824x370) and 5 (1824x190), and a 10-bit mode 6 (2736x1538). These are deliberately not enabled yet because their timing/crop programming needs hardware validation before exposing them to libcamera.

The Sony IMX283 product information also explicitly documents arbitrary horizontal and vertical cropping, and lists the native readout modes and maximum frame rates. See the Sony product information referenced in the project notes before treating the high-speed modes as production-ready.


## Newly enabled native Sony readout modes

The driver now exposes all readout modes listed in Sony's IMX283 documentation:

| Mode | Native recording pixels | Bit depth | Sony maximum |
|---|---:|---:|---:|
| 0 | 5472×3648 | 12 | 21.40 fps |
| 1 | 5472×3648 | 10 | 25.48 fps |
| 1A | 5472×3078 | 10 | 30.17 fps |
| 1S | 3000×3000 | 10 | 42.96 fps |
| 2 | 2736×1824 | 12 | 51.80 fps |
| 2A | 2736×1538 | 12 | 60.27 fps |
| 3 | 1824×1216 | 12 | 60.36 fps |
| 4 | 1824×370 | 12 | 240.21 fps |
| 5 | 1824×190 | 12 | 452.03 fps |
| 6 | 2736×1538 | 10 | 60.01 fps |

Modes 1S, 4, 5 and 6 are newly exposed on this branch. Their timing values are derived from Sony's published maximum frame rates and the driver's existing 72 MHz HMAX representation. **These four modes are intentionally experimental: the HMAX/VMAX values and crop/subsampling interpretation must be validated on hardware.**

Sony documents arbitrary horizontal and vertical cropping, so the driver continues to report the selected analog crop through the V4L2 sub-device selection API. The very-high-speed modes 4 and 5 use vertical subsampling rather than ordinary 3×3 vertical binning; their reported vbin_ratio is therefore kept at 1.
