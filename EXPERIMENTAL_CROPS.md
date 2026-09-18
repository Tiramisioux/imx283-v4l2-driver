# IMX283 experimental 12-bit crop modes

This branch is an experimental extension of `cinemate-dev`.

It preserves the existing 12-bit and 10-bit readout modes and adds fixed, centered crop variants based on the IMX283 Mode 0 12-bit 1x1 readout.

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
