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
