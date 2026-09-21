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

Sony documents additional 12-bit readout modes 4 (1824x370) and 5 (1824x190), and a 10-bit mode 6 (2736x1538). Their table entries exist but are kept out of the default mode list (see the `experimental_modes` module parameter below) because their timing/crop programming needs hardware validation before exposing them to libcamera by default.

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

Modes 1S, 4, 5 and 6 are newly exposed on this branch. Their timing values are derived from Sony's published maximum frame rates and the driver's existing 72 MHz HMAX representation. **These four modes are intentionally experimental: the HMAX/VMAX values and crop/subsampling interpretation must be validated on hardware.** Modes 4 and 5 additionally produce frames (374 and 194 output rows) shorter than CineMate's 720-line preview stream and cannot survive its launch path today, independent of the timing question.

### `experimental_modes` module parameter (WP-283-4)

Because 1S, 4, 5 and 6 are unvalidated, they are kept out of the mode list by default, so neither
`--list-cameras` nor libcamera's format enumeration sees them. Each entry in `supported_modes_12bit[]` /
`supported_modes_10bit[]` carries a `.experimental` flag, set only on IMX283_MODE_1S, _4, _5 and
_6; `get_mode_table()` omits flagged entries unless the module is loaded with
`experimental_modes=1` (e.g. `modprobe imx283 experimental_modes=1`, or a
`dtoverlay=imx283,experimental_modes=1` param line, depending on how the overlay wires module
params through). It is read-only at sysfs (`module_param(..., 0444)`): reload the module to change
it, matching that these timings are meant to be tried deliberately, not toggled live.

With the parameter off (the default), the driver behaves exactly as it did before these four modes
existed: `--list-cameras` and format negotiation only ever see the shipped/validated readouts (0,
1, 1A, 2, 2A, 3, 1C, and the Mode-0 crop family).

With it on, all four come back with no other change to the list.

The entries themselves are not deleted — they are the record of this work and the path to
validating them. What clears each one:

| Mode | What's unvalidated | Clears when |
|---|---|---|
| 1S (3000x3000, 10-bit) | HMAX/VMAX derived from Sony's published 42.96 fps, never measured | A hardware take confirms real image content, correct framing and no wrap at the derived timing (part of the G8 Pi gate) |
| 4 (1824x370, 12-bit, claimed 240 fps) | Same timing-derivation gap as 1S, plus vertical subsampling instead of binning is unverified | Hardware validation of the timing *and* a preview-stream policy that can serve a mode narrower than CineMate's 720-line requirement (a stack-side change, not just a driver one) |
| 5 (1824x190, 12-bit, claimed 452 fps) | Same as mode 4 | Same as mode 4 |
| 6 (2736x1538, 10-bit, claimed 60.01 fps) | Same timing-derivation gap as 1S | A hardware take confirms real image content, correct framing and no wrap at the derived timing |

Sony documents arbitrary horizontal and vertical cropping, so the driver continues to report the selected analog crop through the V4L2 sub-device selection API. The very-high-speed modes 4 and 5 use vertical subsampling rather than ordinary 3×3 vertical binning; their reported vbin_ratio is therefore kept at 1.

## WP-283-3: vertical-crop mechanism verified against mainline

Compared `imx283_start_streaming()` against `raspberrypi/linux` `rpi-6.12.y`
`drivers/media/i2c/imx283.c` (local copy: `reference/imx283-mainline-rpi-6.12.y.c` in the
`experimental-crop-modes` planning tree, fetched 2026-09-20). Summary: the vertical-crop
arithmetic (`y_out_size`/`write_v_size`/`v_pos`/`v_widcut` from `veff`/`vst`/`vct`) is a faithful
port of mainline's, so finding X4 (arbitrary vertical cropping is unproven ground) is a port, not
an invention. Two behavioural divergences and one data gap remain, all desk-checked here and left
for the G8 Pi gate (chart take, checked for correct centring, correct size, and no wrap) to confirm.

| | mainline | this fork | verdict |
|---|---|---|---|
| VCROP_EN (MDSEL3/MDSEL4) scope | enabled for every mode | enabled for `IMX283_MODE_0` only | **Deliberate and safe as-is.** Only Mode 0 has a validated `veff`/`vst`/`vct` and a real arbitrary-crop use (the 18 fixed crops). Widening the gate to every mode today would read `veff == 0` on `IMX283_MODE_1S`/`_4`/`_5`/`_6` and drive `VWIDCUT` negative. Revisit only alongside a `veff` audit of those four modes. No code change. |
| `HTRIMMING_END` | `crop.left + crop.width` | `crop.left + crop.width + 1` | **Unresolved, not changed.** This line runs for every mode, so a wrong guess would move the horizontal window on every readout, not just the new crops. Needs a hardware read-back or the datasheet's exact HTRIMMING start/end semantics (inclusive vs. exclusive end) before touching it; the G8 chart take is the place to catch a one-column miscentre or wrap if it matters. No code change. |
| `veff` set on every mode that is wired for the crop arithmetic (`hbin_ratio == vbin_ratio`, no subsampling) | n/a (mainline only defines modes 0/2/3: `veff` 3694/1824/1234) | `IMX283_MODE_1C` and `IMX283_MODE_2A` were missing `veff` (would silently compute `VWIDCUT` against 0) | **Fixed in this package.** `IMX283_MODE_1C` (1x1, landed via the Pi-validated 6.12.y merge, WP-283-1) gets `veff = 3694`, matching mode 0 and every one of this file's other 1x1 entries. `IMX283_MODE_2A` (2x2 binning, same `mdsel1` family as `IMX283_MODE_2`, no subsampling) gets `veff = 1824`, matching mainline's 2x2 value. `IMX283_MODE_1S`, `_4`, `_5`, `_6` are **left unset**: this file already names all four "intentionally experimental: the HMAX/VMAX values and crop/subsampling interpretation must be validated on hardware" (above), so inventing a `veff` for them here would be a guess dressed as a fix. `IMX283_MODE_1`, `_1A`, `_2` also had no `veff`/`hbin_ratio`/`vbin_ratio`. That was left alone here as a pre-existing, larger gap, and closed afterwards — the first imx283 hardware run showed `IMX283_MODE_2` advertising `binning 1x1` for a 2×2 readout, because `imx283_cfg_mode_binning` has `.min = 1` and quietly clamped the missing 0 to a plausible number. All three now declare both ratios and a `veff`, and `imx283_check_mode_table()` warns at probe time for any entry that leaves them unset, puts its crop outside the active area, or declares a crop width that is not `(active out − ob) × hbin_ratio`. |
| Per-crop VMAX floor | no crop modes exist in mainline, so no precedent | uniform full-frame `min_VMAX`/`default_VMAX` kept on every Mode-0 crop entry | **No change, and none should be made without new data.** This sensor's own binned modes (2, 2A, 3) already carry a *higher* `min_VMAX` than the uncropped 1x1 mode 0 despite far fewer output lines — VMAX counts sensor scan lines, not output lines, so "shorter crop ⇒ lower VMAX" does not hold here. A per-crop floor needs the datasheet or a measured Pi sweep, per the code comment above `supported_modes_12bit[]`. |

Desk-check method for the `veff` row: `check_veff.py` (kept with the WP-283-3 work, not part of this
repo) parses `supported_modes_12bit[]`/`supported_modes_10bit[]` and asserts that every entry with
`hbin_ratio == vbin_ratio` and no subsampling caveat carries the mainline-correspondence `veff` for
that ratio (1×1 → 3694, 2×2 → 1824, 3×3 → 1234). It failed against the pre-fix tree, naming exactly
`IMX283_MODE_1C`, `IMX283_MODE_1S`, and `IMX283_MODE_2A`; it passes after the two in-scope fixes,
with `IMX283_MODE_1S`/`_4`/`_5`/`_6` reported exempt by name (not silently skipped).
