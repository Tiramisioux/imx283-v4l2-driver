# IMX283 driver for Apertar

This is the page for people running the IMX283 on an Apertar camera (Raspberry
Pi 5 or CM5, Apertar-Core). It covers which branch to install, what goes in
`config.txt` and `/etc/modprobe.d`, which sensor modes the driver offers, and
what has not been confirmed on hardware yet. `README.md` stays the reference
for the generic install and the overlay options. `EXPERIMENTAL_CROPS.md` is
out of date; the mode tables in `imx283.c` are the authority.

## Which branch

Use **`cinemate-port`**. It is `cinemate-modes` (the IMX283 modes of the
CineMate stack) plus the fixes made for Apertar since: the mode tables, the
overlay and the module parameters are the ones of `cinemate-modes`. What
differs:

- `CROP_DEFAULT` returns the full active area. Before, it returned the crop of
  whatever mode was selected, so libcamera's `ScalerCropMaximum` and the IPA's
  sensor geometry depended on the mode a previous process had left the sensor
  in (and could go negative).
- Mode, crop and clock lines that were logged at error level on every stream
  start are debug messages now.
- An unsupported media-bus code no longer reads past the end of the code table.

## Install (on the Pi)

The driver lives in `~/drivers/imx283-v4l2-driver`. If it is not there yet:

```bash
mkdir -p ~/drivers
git clone https://github.com/Tiramisioux/imx283-v4l2-driver.git ~/drivers/imx283-v4l2-driver
```

Then, every time you install or update:

```bash
cd ~/drivers/imx283-v4l2-driver
git fetch
git checkout cinemate-port
git pull
./setup.sh
```

`setup.sh` removes any earlier DKMS copy of the module, then builds and
installs this one for the running kernel; its post-install step
(`dkms.postinst`) builds `imx283.dtbo` from `imx283-overlay.dts` and installs
it to `/boot/overlays/`. It needs `dkms`, `git` and the kernel headers, and a
kernel of 6.12 or newer (the driver uses `<linux/unaligned.h>`). Reboot
afterwards.

## config.txt

On Raspberry Pi OS Bookworm the file is `/boot/firmware/config.txt` (older
images used `/boot/config.txt`). Turn camera auto-detect off and load the
overlay:

```
camera_auto_detect=0
dtoverlay=imx283[,cam0][,always-on][,link-frequency=360000000]
```

The brackets are optional parameters, comma separated:

| Parameter | When to use it |
| --------- | -------------- |
| `cam0` | The camera is on the CAM0 connector. Leave it out for CAM1. |
| `always-on` | Keep the camera power on all the time (for debugging hardware problems). |
| `link-frequency=360000000` | Run the CSI-2 link at 360 MHz (720 Mbps per lane) instead of the default. |

Examples:

```
# camera on CAM1
dtoverlay=imx283

# camera on CAM0
dtoverlay=imx283,cam0

# camera on CAM0, link at 360 MHz
dtoverlay=imx283,cam0,link-frequency=360000000
```

### Settings to leave alone

- **`link-frequency`**: the default is 720 MHz (1440 Mbps per lane). The driver
  accepts only 720 MHz and 360 MHz (`720000000` and `360000000`); any other
  value makes the probe fail. Leave the parameter out unless you need the slower
  link. The frame rates below are for the 720 MHz default.

## /etc/modprobe.d/imx283.conf

`crop_vmax` and `experimental_modes` are module parameters, not overlay
parameters, so they do not go on the `dtoverlay=` line. Create
`/etc/modprobe.d/imx283.conf` with:

```
options imx283 crop_vmax=0 experimental_modes=0
```

- **`crop_vmax=0`**: the driver's default is 1, which gives the cropped
  aspect-ratio rows their own, lower VMAX floors (higher frame rates than the
  parent mode, for example about 35 fps for the 2.55:1 Mode 0 row against
  21.4 fps). Those floors were worked out from Sony's figures, not measured on
  a sensor, so Apertar runs with `crop_vmax=0`: every row keeps its parent
  mode's floor. The frame rates in the table below are for `crop_vmax=0`.
- **`experimental_modes=0`**: this is the default; it keeps the 16 unvalidated
  entries out of the mode list (readout modes 1S, 4, 5 and 6, and the twelve
  MODE_1C aspect-ratio rows). Leave it at 0 for normal use.

Reboot after editing. To see what the loaded module is using:

```bash
cat /sys/module/imx283/parameters/crop_vmax          # N when crop_vmax=0
cat /sys/module/imx283/parameters/experimental_modes # N by default
```

## Modes

With the defaults the driver offers **57 modes**: 42 at 12 bit and 15 at
10 bit. They come in seven families. A family is one readout mode of the
sensor (its binning and frame-rate limit) plus, in most families, a set of
centred crops of it framed for common aspect ratios, from 1:1 up to 2.55:1.
The sizes below are the *active* picture the sensor delivers, per the mode
tables in `imx283.c`.

| Family | Bits | Binning | Modes | Active picture sizes | Driver max fps |
| ------ | ---- | ------- | ----- | -------------------- | -------------- |
| Mode 0 | 12 | 1x1 | 14 | 5472x3648; crops 3648x3648, 4864x3648, 5016x3648, then 5472 wide by 3080, 2956, 2880, 2736, 2488, 2464, 2328, 2288, 2188, 2144 | 21.40 |
| Mode 2 | 12 | 2x2 | 4 | 2736x1824; crops 1824x1824, 2432x1824, 2508x1824 | 51.80 |
| Mode 2A | 12 | 2x2 | 10 | 2736 wide by 1540, 1478, 1440, 1368, 1244, 1232, 1164, 1144, 1094, 1072 (no full-frame entry; Mode 2 serves the tall ratios) | 60.27 |
| Mode 3 | 12 | 3x3 | 14 | 1824x1216; crops 1216x1216, 1620x1216, 1672x1216, 1820x1024, then 1824 wide by 984, 960, 912, 828, 820, 776, 764, and 1820x728, 1824x716 | 60.36 |
| Mode 1 | 10 | 1x1 | 4 | 5472x3648; crops 3648x3648, 4864x3648, 5016x3648 | 25.48 |
| Mode 1A | 10 | 1x1 | 10 | 5472x3076; crops 5472 wide by 2956, 2880, 2736, 2488, 2464, 2328, 2288, 2188, 2144 | 30.17 |
| Mode 1C | 10 | 1x1 | 1 | 3840x2160 (a UHD window of the array) | 60.16 |

The 12-bit families (Modes 0, 2, 2A and 3) are offered for the 12-bit Bayer
format and the 10-bit ones (Modes 1, 1A and 1C) for the 10-bit format. The
driver's maximum frame rates are the sensor-side limit at the 720 MHz link
with `crop_vmax=0` (72 MHz divided by HMAX times VMAX). libcamera and the ISP
can lower them further on a Pi, so read them as a ceiling.

Each mode is advertised at its *transport* size, which is the active picture
plus the optical-black (OB) margin the sensor sends with it: 96 columns and 16
rows at 1x1, 48 and 4 at 2x2, 32 and 4 at 3x3. So Mode 0 is listed as
5568x3664 around a 5472x3648 picture, Mode 2 as 2784x1828 around 2736x1824,
Mode 3 as 1856x1220 around 1824x1216, and Mode 1C as 3936x2176 around
3840x2160. The crop reported for each mode is the active window in the sensor's
native 5592x3710 array; the active area starts at (108, 40). The driver also
publishes the delivered picture size as the read-only "Mode Active Width" and
"Mode Active Height" controls.

The 16 experimental entries (see `experimental_modes` above) are not in the
table. The twelve MODE_1C aspect-ratio rows are in that set because they
streamed noise while MODE_1C carried Mode 0's `veff`; the `veff` is fixed now
(commit `335781d`), but no re-test of the rows is recorded. Modes 1S, 4, 5 and 6
have never been validated.

## Open questions (not yet confirmed on hardware)

These are known and have not been fixed; this page only describes them.

**MODE_1C optical black at the centred window.** MODE_1C's horizontal window
origin is the centred value, 924, since commit `cb925e5` (it was 236 before).
At 924 the mode was checked on one live ISP preview frame: 3936x2176 10-bit, a
full picture edge to edge, no short frame, no noise band. It has not been
checked on a raw 3936x2176 frame, so it is not confirmed that the 96 leading
columns are optical black and that the 16 trailing rows are too, which is what
the driver and the DNG's ActiveArea assume. At 236 a raw frame had columns
0..95 flat at the black level (median 50, standard deviation about 5). If a raw
frame at 924 shows picture in those columns instead, the ActiveArea of the DNGs
discards real picture. The vertical origin is `.top = 852`; the centred row for
the real active area would be 784, and 784 has not been tried since the `veff`
fix. `imx283.c` keeps 236 as `IMX283_MODE_1C_WINDOW_LEFT_HW_CONFIRMED` for a
fallback.

**No lens shading in the 1x1 modes.** The Raspberry Pi libcamera IPA works out
the sensor's scale as the analogue crop width divided by the output width, by
integer division (`ipa_base.cpp`). The driver's output size is the transport
size, which includes the OB columns (5568), while the crop it reports does not
(5472), so the quotient is 5472 / 5568 = 0 in every 1x1 mode (Modes 0, 1, 1A and
1C and all their crops; likewise 3840 / 3936 for Mode 1C). ALSC resamples the
calibration table by that scale, divides by zero, and ends up reading a single
calibration cell for the whole frame, so those modes get no spatial lens
shading correction. The same division gives 1 for the 2x2 modes (5472 / 2784)
and 2 for 3x3 (5472 / 1856) instead of 1.97 and 2.95, so ALSC there reads
about the left 51% and 68% of the table, and the IPA's binning factor comes out
as 1 for the 2x2 modes (which switches debinning off) and 2 for 3x3. This
comes from reading the libcamera source (`rpi-libcamera` v0.7.2+rpt20261009),
not from comparing captures. A fix would have to change what the driver reports
or what libcamera computes; neither has been done.

**Per-crop VMAX floors.** With `crop_vmax=1` the rows advertise frame rates
above their parent mode's. None of those floors has been measured, which is why
`/etc/modprobe.d/imx283.conf` above sets `crop_vmax=0`.

## Logging

The driver logs `Device found` and a few control-change lines at info level.
The per-stream lines (`Mode: Size ...`, `Analogue Crop ...`, `Using clk freq
...`) are debug messages; to see them, with dynamic debug enabled in the
kernel:

```bash
echo 'module imx283 +p' | sudo tee /sys/kernel/debug/dynamic_debug/control
```
