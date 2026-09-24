// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 Support for the IMX283
 *
 * The IMX283 has BigEndian register addresses
 * and uses little-endian value.
 *
 */

#include <linux/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/*
 * Read-only mode-geometry controls (WP-283-5). Named, not identified: the
 * stack discovers "Mode Binning" / "Mode Crop Left" / "Mode Crop Top" /
 * "Mode Crop Width" / "Mode Crop Height" by their exact V4L2_CTRL_TYPE_INTEGER
 * name on whichever custom control base a given sensor driver picks, per
 * DEC-4. The ids below only need to be unique within this driver.
 */
#ifndef V4L2_CID_USER_IMX283_BASE
#define V4L2_CID_USER_IMX283_BASE (V4L2_CID_USER_BASE + 0x2100)
#endif

#define V4L2_CID_IMX283_MODE_BINNING     (V4L2_CID_USER_IMX283_BASE + 0)
#define V4L2_CID_IMX283_MODE_CROP_LEFT   (V4L2_CID_USER_IMX283_BASE + 1)
#define V4L2_CID_IMX283_MODE_CROP_TOP    (V4L2_CID_USER_IMX283_BASE + 2)
#define V4L2_CID_IMX283_MODE_CROP_WIDTH  (V4L2_CID_USER_IMX283_BASE + 3)
#define V4L2_CID_IMX283_MODE_CROP_HEIGHT (V4L2_CID_USER_IMX283_BASE + 4)
/*
 * Where the active picture starts inside the TRANSPORT FRAME -- a different
 * coordinate space from the "Mode Crop" pair above, which is native sensor
 * pixels. These two exist so a DNG writer can emit ActiveArea without
 * guessing where this sensor puts its optical black, which is not derivable
 * from the sizes: the imx283 emits its horizontal OB columns LEADING and its
 * vertical OB rows TRAILING, while the imx585's RAW16 modes split their
 * vertical padding evenly top and bottom and have no horizontal padding at
 * all. Measured on a CM5 from a 3936x2176 MODE_1C frame: columns 0..95 sit
 * at the black level and column 96 is the first picture column, rows
 * 2160..2175 are zero-filled.
 */
#define V4L2_CID_IMX283_MODE_ACTIVE_LEFT (V4L2_CID_USER_IMX283_BASE + 5)
#define V4L2_CID_IMX283_MODE_ACTIVE_TOP  (V4L2_CID_USER_IMX283_BASE + 6)

struct cci_reg_sequence {
	u32 reg;
	u64 val;
};

#define CCI_REG_ADDR_MASK		GENMASK(15, 0)
#define CCI_REG_WIDTH_SHIFT		16
#define CCI_REG_WIDTH_MASK		GENMASK(19, 16)
#define CCI_REG_LE                     BIT(20)


#define CCI_REG8(x)			((1 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG16(x)			((2 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG24(x)			((3 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG32(x)			((4 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG64(x)			((8 << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG16_LE(x)         (CCI_REG_LE | (2U << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG24_LE(x)         (CCI_REG_LE | (3U << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG32_LE(x)         (CCI_REG_LE | (4U << CCI_REG_WIDTH_SHIFT) | (x))
#define CCI_REG64_LE(x)         (CCI_REG_LE | (8U << CCI_REG_WIDTH_SHIFT) | (x))




/*
 * TODOs
 *  - Move to active state api
 *  - Add 720 MBps speed mode to link_freq
 *    - HMAX/VMAX must be calculated based on link-freq to support this.
 *  - Support arbitrary cropping (this branch exposes a broad set of
 *    fixed Mode-0 12-bit 1x1 crop variants for validation)
 * 
 *  - account for the VOB
 *  - Identify where the HOB is coming from.
 * 
 *  - Remove 'events' that are not used.
 *  - Fix/remove HFLIP/VFLIP which aren't well supported at all.
 *  - Fix exposure and blanking calculations
 */

/* Chip ID */
#define IMX283_REG_CHIP_ID		CCI_REG8(0x3000)
#define IMX283_CHIP_ID			0x0b	// Default power on state

#define IMX283_REG_STANDBY		CCI_REG8(0x3000)
#define   IMX283_ACTIVE			0
#define   IMX283_STANDBY		BIT(0)
#define   IMX283_STBLOGIC		BIT(1)
#define   IMX283_STBMIPI		BIT(2)
#define   IMX283_STBDV			BIT(3)
#define   IMX283_SLEEP			BIT(4)

#define IMX283_REG_CLAMP		CCI_REG8(0x3001)
#define   IMX283_CLPSQRST		BIT(4)

#define IMX283_REG_PLSTMG08		CCI_REG8(0x3003)
#define   IMX283_PLSTMG08_VAL		0x77

#define IMX283_REG_MDSEL1		CCI_REG8(0x3004)
#define IMX283_REG_MDSEL2		CCI_REG8(0x3005)
#define IMX283_REG_MDSEL3		CCI_REG8(0x3006)
#define IMX283_MDSEL3_VCROP_EN	BIT(5)
#define IMX283_REG_MDSEL4		CCI_REG8(0x3007)
#define IMX283_MDSEL4_VCROP_EN	(BIT(4) | BIT(6))

#define IMX283_REG_SVR			CCI_REG16_LE(0x3009)

#define IMX283_REG_HTRIMMING		CCI_REG8(0x300b)
#define   IMX283_MDVREV			BIT(0) // VFLIP
#define   IMX283_HTRIMMING_EN		BIT(4)
#define   IMX283_HTRIMMING_RESERVED	BIT(5)

#define IMX283_REG_VWINPOS		CCI_REG16_LE(0x300f)
#define IMX283_REG_VWIDCUT		CCI_REG16_LE(0x3011)

#define IMX283_REG_MDSEL7		CCI_REG16_LE(0x3013)

/* CSI Clock Configuration */
#define IMX283_REG_TCLKPOST		CCI_REG8(0x3018)
#define IMX283_REG_THSPREPARE		CCI_REG8(0x301a)
#define IMX283_REG_THSZERO		CCI_REG8(0x301c)
#define IMX283_REG_THSTRAIL		CCI_REG8(0x3020)
#define IMX283_REG_TCLKPREPARE		CCI_REG8(0x3022)
#define IMX283_REG_TCLKZERO		CCI_REG16_LE(0x3024)
#define IMX283_REG_TLPX			CCI_REG8(0x3026)
#define IMX283_REG_THSEXIT		CCI_REG8(0x3028)
#define IMX283_REG_TCLKPRE		CCI_REG8(0x302a)

#define IMX283_REG_Y_OUT_SIZE		CCI_REG16_LE(0x302f)
#define IMX283_REG_WRITE_VSIZE		CCI_REG16_LE(0x3031)
#define IMX283_REG_OB_SIZE_V		CCI_REG8(0x3033)

/* HMAX internal HBLANK*/
#define IMX283_REG_HMAX			CCI_REG16_LE(0x3036)
#define IMX283_HMAX_MAX			0xffff

/* VMAX internal VBLANK */
#define IMX283_REG_VMAX			CCI_REG24_LE(0x3038)
#define   IMX283_VMAX_MAX		0xfffff

/* SHR internal */
#define IMX283_REG_SHR			CCI_REG16_LE(0x303b)
#define   IMX283_SHR_MIN		11

/*
 * Analog gain control
 *  Gain [dB] = –20log{(2048 – value [10:0]) /2048}
 *  Range: 0dB to approximately +27dB
 */
#define IMX283_REG_ANALOG_GAIN		CCI_REG16_LE(0x3042)
#define   IMX283_ANA_GAIN_MIN		0
#define   IMX283_ANA_GAIN_MAX		1957
#define   IMX283_ANA_GAIN_STEP		1
#define   IMX283_ANA_GAIN_DEFAULT	0x0

/*
 * Digital gain control
 *  Gain [dB] = value * 6
 *  Range: 0dB to +18db
 */
#define IMX283_REG_DIGITAL_GAIN		CCI_REG8(0x3044)
#define IMX283_DGTL_GAIN_MIN		0
#define IMX283_DGTL_GAIN_MAX		3
#define IMX283_DGTL_GAIN_DEFAULT	0
#define IMX283_DGTL_GAIN_STEP		1

#define IMX283_REG_HTRIMMING_START	CCI_REG16_LE(0x3058)
#define IMX283_REG_HTRIMMING_END	CCI_REG16_LE(0x305a)

#define IMX283_REG_MDSEL18		CCI_REG16_LE(0x30f6)

/* Master Mode Operation Control */
#define IMX283_REG_XMSTA		CCI_REG8(0x3105)
#define   IMX283_XMSTA			BIT(0)

#define IMX283_REG_SYNCDRV		CCI_REG8(0x3107)
#define   IMX283_SYNCDRV_XHS_XVS	(0xa0 | 0x02)
#define   IMX283_SYNCDRV_HIZ		(0xa0 | 0x03)

/* PLL Standby */
#define IMX283_REG_STBPL		CCI_REG8(0x320b)
#define  IMX283_STBPL_NORMAL		0x00
#define  IMX283_STBPL_STANDBY		0x03

/* Input Frequency Setting */
#define IMX283_REG_PLRD1		CCI_REG8(0x36c1)
#define IMX283_REG_PLRD2		CCI_REG16_LE(0x36c2)
#define IMX283_REG_PLRD3		CCI_REG8(0x36f7)
#define IMX283_REG_PLRD4		CCI_REG8(0x36f8)

#define IMX283_REG_PLSTMG02		CCI_REG8(0x36aa)
#define   IMX283_PLSTMG02_VAL		0x00

#define IMX283_REG_EBD_X_OUT_SIZE	CCI_REG16_LE(0x3a54)

/* Test pattern generator */
#define IMX283_REG_TPG_CTRL		CCI_REG8(0x3156)
#define   IMX283_TPG_CTRL_CLKEN		BIT(0)
#define   IMX283_TPG_CTRL_PATEN		BIT(4)

#define IMX283_REG_TPG_PAT		CCI_REG8(0x3157)
#define   IMX283_TPG_PAT_ALL_000	0x00
#define   IMX283_TPG_PAT_ALL_FFF	0x01
#define   IMX283_TPG_PAT_ALL_555	0x02
#define   IMX283_TPG_PAT_ALL_AAA	0x03
#define   IMX283_TPG_PAT_H_COLOR_BARS	0x0a
#define   IMX283_TPG_PAT_V_COLOR_BARS	0x0b

#define MHZ(x)				((x) * 1000 * 1000)

/* MIPI link speed is fixed at 1.44Gbps for all the modes */
#define IMX283_DEFAULT_LINK_FREQ	MHZ(720)

/* Exposure control */
#define IMX283_EXPOSURE_MIN		52
#define IMX283_EXPOSURE_STEP		1
#define IMX283_EXPOSURE_DEFAULT		1000
#define IMX283_EXPOSURE_MAX		49865

/* Embedded metadata stream structure */
#define IMX283_EMBEDDED_LINE_WIDTH 16384
#define IMX283_NUM_EMBEDDED_LINES 1

#define IMAGE_PAD			0

/*
 * Readout modes 1S, 4, 5 and 6 (added by this branch) carry HMAX/VMAX
 * timings derived only from Sony's published maximum frame rates, not
 * from hardware validation; see EXPERIMENTAL_CROPS.md. Modes 4 and 5
 * also produce frames shorter than CineMate's 720-line preview stream
 * and cannot survive its launch path today. Keep them out of the
 * default mode list (struct imx283_mode.experimental, checked in
 * get_mode_table()) until they clear hardware validation; a module
 * owner who wants to exercise them anyway can still load the driver
 * with experimental_modes=1.
 */
static bool experimental_modes;
module_param(experimental_modes, bool, 0444);
/*
 * Apply the experimental per-crop VMAX floor by default. The driver is
 * normally autoloaded by the camera device, so requiring a manual modprobe
 * argument would make the experiment needlessly fragile. Set crop_vmax=0
 * only when explicitly disabling the experiment for a test.
 */
static bool crop_vmax = true;
module_param(crop_vmax, bool, 0444);
MODULE_PARM_DESC(crop_vmax,
		  "Use the per-crop VMAX floor on Mode-0 vertical crops (experimental, "
		  "UNMEASURED: see development/imx283-crop-fps/. Default 1 = use the experimental per-crop floor; set 0 to keep the Mode-0 floor.)");
MODULE_PARM_DESC(experimental_modes,
		  "Enable unvalidated readout modes 1S, 4, 5 and 6 (default: off, see EXPERIMENTAL_CROPS.md)");

/* imx283 native and active pixel array size. */
static const struct v4l2_rect imx283_native_area = {
	.top = 0,
	.left = 0,
	.width = 5592,
	.height = 3710,
};

static const struct v4l2_rect imx283_active_area = {
	/*
	 * Sony's 5472x3648 recommended recording area is located at
	 * (108,40) in the 5592x3710 native array.  Keep X/Y in the
	 * correct coordinate axes: left=108, top=40.
	 */
	.top = 40,
	.left = 108,
	.width = 5472,
	.height = 3648,
};

struct IMX283_reg_list {
	unsigned int num_of_regs;
	const struct cci_reg_sequence *regs;
};

/* Mode : resolution and related config&values */
struct imx283_mode {
	unsigned int mode;

	/* Bits per pixel */
	unsigned int bpp;

	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* minimum H-timing */
	u64 min_HMAX;

	/* minimum V-timing */
	u64 min_VMAX;

	/* Experimental per-crop VMAX floor; zero means use min_VMAX. */
	u64 crop_min_VMAX;

	/* default H-timing */
	u64 default_HMAX;

	/* default V-timing */
	u64 default_VMAX;

	/* minimum SHR */
	u64 min_SHR;

	/* Vertical crop calculation parameters. */
	u32 veff;
	u32 vst;
	u32 vct;

	/* Horizontal and vertical binning ratio. */
	u8 hbin_ratio;
	u8 vbin_ratio;

	/* Optical Blanking */
	u32 horizontal_ob;
	u32 vertical_ob;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/*
	 * Set on entries whose timing is unvalidated on hardware (see the
	 * experimental_modes module parameter above). get_mode_table()
	 * omits these unless experimental_modes is set.
	 */
	bool experimental;
};

/*
 * IMX283 transport geometry.
 *
 * HTRIMMING selects the active sensor window, but it does not remove the
 * sensor's leading horizontal optical-black columns from the CSI-2 packet.
 * Likewise, WRITE_VSIZE includes the trailing vertical optical-black rows.
 * Therefore mode->width/height are the actual transport dimensions seen by
 * CSI-2, while mode->crop describes the active sensor window inside them.
 *
 * This distinction is critical: advertising active-only dimensions while
 * the sensor still prepends HOB makes the CSI receiver interpret the wrong
 * line length, producing the horizontal coloured-striping artifact seen on
 * CM5.
 */
static unsigned int imx283_output_width(const struct imx283_mode *mode)
{
	return mode->width;
}

static unsigned int imx283_output_height(const struct imx283_mode *mode)
{
	return mode->height;
}

static struct v4l2_rect imx283_output_crop(const struct imx283_mode *mode)
{
	return mode->crop;
}

static const struct imx283_mode *imx283_find_nearest_mode(
	const struct imx283_mode *modes, unsigned int num_modes,
	unsigned int requested_width, unsigned int requested_height)
{
	const struct imx283_mode *best = &modes[0];
	unsigned int best_score = UINT_MAX, i;

	for (i = 0; i < num_modes; i++) {
		unsigned int w = imx283_output_width(&modes[i]);
		unsigned int h = imx283_output_height(&modes[i]);
		unsigned int dw = w > requested_width ? w - requested_width :
						 requested_width - w;
		unsigned int dh = h > requested_height ? h - requested_height :
						 requested_height - h;
		unsigned int score = dw + dh;

		if (score < best_score) {
			best = &modes[i];
			best_score = score;
		}
	}
	return best;
}

struct imx283_input_frequency {
	unsigned int mhz;
	unsigned int reg_count;
	struct cci_reg_sequence regs[4];
};

static const struct imx283_input_frequency imx283_frequencies[] = {
	{
		.mhz = MHZ(6),
		.reg_count = 4,
		.regs = {
			{ IMX283_REG_PLRD1, 0x00 },
			{ IMX283_REG_PLRD2, 0x00f0 },
			{ IMX283_REG_PLRD3, 0x00 },
			{ IMX283_REG_PLRD4, 0xc0 },
		},
	},
	{
		.mhz = MHZ(12),
		.reg_count = 4,
		.regs = {
			{ IMX283_REG_PLRD1, 0x01 },
			{ IMX283_REG_PLRD2, 0x00f0 },
			{ IMX283_REG_PLRD3, 0x01 },
			{ IMX283_REG_PLRD4, 0xc0 },
		},
	},
	{
		.mhz = MHZ(18),
		.reg_count = 4,
		.regs = {
			{ IMX283_REG_PLRD1, 0x01 },
			{ IMX283_REG_PLRD2, 0x00a0 },
			{ IMX283_REG_PLRD3, 0x01 },
			{ IMX283_REG_PLRD4, 0x80 },
		},
	},
	{
		.mhz = MHZ(24),
		.reg_count = 4,
		.regs = {
			{ IMX283_REG_PLRD1, 0x02 },
			{ IMX283_REG_PLRD2, 0x00f0 },
			{ IMX283_REG_PLRD3, 0x02 },
			{ IMX283_REG_PLRD4, 0xc0 },
		},
	},
};

enum imx283_modes {
	IMX283_MODE_0,
	IMX283_MODE_1,
	IMX283_MODE_1A,
	IMX283_MODE_1S,
	IMX283_MODE_2,
	IMX283_MODE_2A,
	IMX283_MODE_3,
	IMX283_MODE_4,
	IMX283_MODE_5,
	IMX283_MODE_6,
	IMX283_MODE_1C,	/* UHD 4K (3840x2160) 16:9 crop */
};

struct imx283_readout_mode {
	u64 mdsel1;
	u64 mdsel2;
	u64 mdsel3;
	u64 mdsel4;
};

static const struct imx283_readout_mode imx283_readout_modes[] = {
	/* All pixel scan modes */
	[IMX283_MODE_0] = { 0x04, 0x03, 0x10, 0x00 }, /* 12 bit */
	[IMX283_MODE_1] = { 0x04, 0x01, 0x00, 0x00 }, /* 10 bit */
	[IMX283_MODE_1A] = { 0x04, 0x01, 0x20, 0x50 }, /* 10 bit */
	[IMX283_MODE_1S] = { 0x04, 0x41, 0x20, 0x50 }, /* 10 bit */

	/* Horizontal / Vertical 2/2-line binning */
	[IMX283_MODE_2] = { 0x0d, 0x11, 0x50, 0x00 }, /* 12 bit */
	[IMX283_MODE_2A] = { 0x0d, 0x11, 0x70, 0x50 }, /* 12 bit */

	/* Horizontal / Vertical 3/3-line binning */
	[IMX283_MODE_3] = { 0x1e, 0x18, 0x10, 0x00 }, /* 12 bit */

	/* Veritcal 2/9 subsampling, horizontal 3 binning cropping */
	[IMX283_MODE_4] = { 0x29, 0x18, 0x30, 0x50 }, /* 12 bit */

	/* Vertical 2/19 subsampling binning, horizontal 3 binning */
	[IMX283_MODE_5] = { 0x2d, 0x18, 0x10, 0x00 }, /* 12 bit */

	/* Vertical 2 binning horizontal 2/4, subsampling 16:9 cropping */
	[IMX283_MODE_6] = { 0x18, 0x21, 0x00, 0x09 }, /* 10 bit */

	/* UHD 4K (3840x2160) 16:9 crop readout */
	[IMX283_MODE_1C] = { 0x30, 0x41, 0x00, 0x00 }, /* 10 bit */
};

static const struct cci_reg_sequence mipi_data_rate_1440Mbps[] = {
	/* The default register settings provide the 1440Mbps rate */
#if 0
	{ CCI_REG8(0x36c5), 0x00 }, /* Undocumented */
	{ CCI_REG8(0x3ac4), 0x00 }, /* Undocumented */

	{ CCI_REG8(0x320B), 0x00 }, /* STBPL */
	{ CCI_REG8(0x3018), 0xa7 }, /* TCLKPOST */
	{ CCI_REG8(0x301A), 0x6f }, /* THSPREPARE */
	{ CCI_REG8(0x301C), 0x9f }, /* THSZERO */
	{ CCI_REG8(0x301E), 0x5f }, /* THSTRAIL */
	{ CCI_REG8(0x3020), 0x5f }, /* TCLKTRAIL */
	{ CCI_REG8(0x3022), 0x6f }, /* TCLKPREPARE */
	{ CCI_REG8(0x3024), 0x7f }, /* TCLKZERO[7:0] */
	{ CCI_REG8(0x3025), 0x01 }, /* TCLKZERO[8] */
	{ CCI_REG8(0x3026), 0x4f }, /* TLPX*/
	{ CCI_REG8(0x3028), 0x47 }, /* THSEXIT */
	{ CCI_REG8(0x302A), 0x07 }, /* TCKLPRE */
	{ CCI_REG8(0x3104), 0x02 }, /* SYSMODE */

#endif
};

static const struct cci_reg_sequence mipi_data_rate_720Mbps[] = {
	/* Undocumented Arducam Additions "For 720MBps" Setting */
	{ CCI_REG8(0x36c5), 0x01 }, /* Undocumented */
	{ CCI_REG8(0x3ac4), 0x01 }, /* Undocumented */

	{ CCI_REG8(0x320B), 0x00 }, /* STBPL */
	{ CCI_REG8(0x3018), 0x77 }, /* TCLKPOST */
	{ CCI_REG8(0x301A), 0x37 }, /* THSPREPARE */
	{ CCI_REG8(0x301C), 0x67 }, /* THSZERO */
	{ CCI_REG8(0x301E), 0x37 }, /* THSTRAIL */
	{ CCI_REG8(0x3020), 0x37 }, /* TCLKTRAIL */
	{ CCI_REG8(0x3022), 0x37 }, /* TCLKPREPARE */
	{ CCI_REG8(0x3024), 0xDF }, /* TCLKZERO[7:0] */
	{ CCI_REG8(0x3025), 0x00 }, /* TCLKZERO[8] */
	{ CCI_REG8(0x3026), 0x2F }, /* TLPX*/
	{ CCI_REG8(0x3028), 0x47 }, /* THSEXIT */
	{ CCI_REG8(0x302A), 0x0F }, /* TCKLPRE */
	{ CCI_REG8(0x3104), 0x02 }, /* SYSMODE */
};

static const s64 link_frequencies[] = {
	MHZ(720), /* 1440 Mbps lane data rate */
	MHZ(360), /* 720 Mbps data lane rate */
};

static const struct IMX283_reg_list link_freq_reglist[] = {
	{ /* MHZ(720)*/
		.num_of_regs = ARRAY_SIZE(mipi_data_rate_1440Mbps),
		.regs = mipi_data_rate_1440Mbps,
	},
	{ /* MHZ(360) */
		.num_of_regs = ARRAY_SIZE(mipi_data_rate_720Mbps),
		.regs = mipi_data_rate_720Mbps,
	},
};

#define CENTERED_RECTANGLE(rect, _width, _height) \
	{ \
		.left = rect.left + ((rect.width - (_width)) / 2), \
		.top = rect.top + ((rect.height - (_height)) / 2), \
		.width = (_width), \
		.height = (_height), \
	}

/*
 * Mode configs.
 *
 * CONFIRMED: HMAX tracks emitted columns. MODE_3 emits all 5472 columns
 * and declares min_HMAX 284; its shorter HMAX is therefore not a result
 * of horizontal binning alone.
 *
 * CONFIRMED: VMAX tracks the sensor's scanned lines, which is why binning
 * cannot shorten the vertical period. A VCROP window is different from
 * binning: rows excluded by the window are not scanned. The vendor-preset
 * VCROP configurations in this file (MODE_1A, MODE_1S, MODE_2A and MODE_4)
 * all carry reduced VMAX floors, while the Mode-0 crops are driver-programmed
 * arbitrary VWIDCUT windows.
 *
 * UNKNOWN: whether an arbitrary driver-programmed Mode-0 VWIDCUT window
 * shortens the sensor scan in the same way as the vendor-preset windows.
 * That requires hardware measurement; see development/imx283-crop-fps/.
 *
 * MODE_2A is a direct counterexample to the old "binned modes carry a higher
 * floor" wording: its min_VMAX is 3300, below Mode 0's 3793. Neither 3793
 * nor any Mode-0 crop floor is treated as authoritative by this comment.
 */

/*
 * Experimental aspect-ratio crop variants.
 *
 * The crop rectangle is expressed in native sensor coordinates. The
 * reported transport dimensions include the mode's optical-black area.
 * These entries are geometry experiments only; they retain the parent
 * mode's timing floor until measured on hardware.
 */
#define IMX283_ASPECT_MODE(_mode, _bpp, _cw, _ch, _hmax, _vmax, _crop_vmax, _dhmax, _dvmax, _shr, _veff, _hb, _vb, _hob, _vob, _left, _top) \
	{ \
		.mode = (_mode), .bpp = (_bpp), \
		.width = ((_cw) / (_hb)) + (_hob), \
		.height = ((_ch) / (_vb)) + (_vob), \
		.min_HMAX = (_hmax), .min_VMAX = (_vmax), \
		.crop_min_VMAX = (_crop_vmax), \
		.default_HMAX = (_dhmax), .default_VMAX = (_dvmax), \
		.min_SHR = (_shr), .veff = (_veff), .vst = 0, .vct = 0, \
		.hbin_ratio = (_hb), .vbin_ratio = (_vb), \
		.horizontal_ob = (_hob), .vertical_ob = (_vob), \
		.crop = { .left = (_left), .top = (_top), .width = (_cw), .height = (_ch) }, \
		.experimental = false, \
	}

#define IMX283_CROPPED_1X1_MODE(_cw, _ch, _left, _top) \
	{ \
		.mode = IMX283_MODE_0, .bpp = 12, \
		.width = (_cw) + 96, .height = (_ch) + 16, \
		.min_HMAX = 887, .min_VMAX = 3793, \
		.crop_min_VMAX = (_ch) + 16 + 129, \
		.default_HMAX = 900, .default_VMAX = 4000, \
		.min_SHR = 12, .veff = 3694, .vst = 0, .vct = 0, \
		.hbin_ratio = 1, .vbin_ratio = 1, \
		.horizontal_ob = 96, .vertical_ob = 16, \
		.crop = { .left = (_left), .top = (_top), .width = (_cw), .height = (_ch) }, \
		.experimental = false, \
	}

#define IMX283_CROP_1X1(_w, _h) \
	IMX283_CROPPED_1X1_MODE((_w), (_h), (5472 - (_w)) / 2, (3648 - (_h)) / 2)

static const struct imx283_mode supported_modes_12bit[] = {
	{
		/*
		 * 5568x3664 Mode 0. min_VMAX 3793 is a timing value that back-solves
		 * from 72e6 / (887 x 3793) ~= 21.40 fps in this table; it is not a
		 * datasheet-authoritative vertical scan floor.
		 */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 3648 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
	},
	{
		/* 2784x1828 51.80fps readout mode 2 */
		.mode = IMX283_MODE_2,
		.bpp = 12,
		.width = (5472 + 96)/2,
		.height = (3648 + 8)/2,
		.min_HMAX = 362,
		.min_VMAX = 3840,
		.default_HMAX = 375,
		.default_VMAX = 3840,
		.min_SHR = 12,
		/*
		 * 2x2 binning, same readout family as IMX283_MODE_2A (mdsel1
		 * 0x0d, "Horizontal / Vertical 2/2-line binning"), which is
		 * also why the output is half the array in both axes. These
		 * five fields were absent until now and therefore zero, and
		 * a zero hbin_ratio is not harmless: it is what
		 * imx283_update_mode_metadata() feeds to the read-only "Mode
		 * Binning" control, whose .min = 1 quietly clamped it to 1, so
		 * the Pi advertised this 2x2 mode as "binning 1x1". libcamera
		 * derives its own binning from analogCrop.width /
		 * outputSize.width and the IPA uses it for black level and
		 * lens shading, so the wrong ratio is not cosmetic.
		 *
		 * veff is mainline's value for 2x2 binning (WP-283-3), the
		 * same one MODE_2A carries. It is unused today -- the
		 * arbitrary vertical-crop path in imx283_start_streaming() is
		 * Mode-0-only -- but a zero left here is a live trap for
		 * anyone who widens that gate, because VWIDCUT would go
		 * negative.
		 */
		.veff = 1824,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 2,
		.vbin_ratio = 2,
		.horizontal_ob = 96/2,
		.vertical_ob = 8/2,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
	},
	{
		/* Readout mode 2A: 2x2 binned 12-bit, 16:9 (2736x1538 active). */
		.mode = IMX283_MODE_2A,
		.bpp = 12,
		.width = 2736 + 48,
		.height = 1538 + 4,
		.min_HMAX = 362,
		.min_VMAX = 3300,
		.default_HMAX = 375,
		.default_VMAX = 3300,
		.min_SHR = 12,
		/*
		 * Same 2x2 binning family as IMX283_MODE_2 (mdsel1 0x0d,
		 * "Horizontal / Vertical 2/2-line binning", no subsampling),
		 * so it gets mainline's veff for 2x2 binning (WP-283-3).
		 */
		.veff = 1824,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 2,
		.vbin_ratio = 2,
		.horizontal_ob = 48,
		.vertical_ob = 4,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3076),
	},
	{
		/* Readout mode 3: 3x3 binned 12-bit (1824x1216 active). */
		.mode = IMX283_MODE_3,
		.bpp = 12,
		.width = 1824 + 32,
		.height = 1216 + 4,
		.min_HMAX = 284,
		.min_VMAX = 4200,
		.default_HMAX = 285,
		.default_VMAX = 4200,
		.min_SHR = 16,
		.veff = 1234,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 3,
		.vbin_ratio = 3,
		.horizontal_ob = 32,
		.vertical_ob = 4,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
	},
	{
		/*
		 * Readout mode 4: 3x horizontal binning with 2/9 vertical
		 * subsampling, 1824x370 active output, 12-bit.
		 *
		 * Sony specifies 240.21 fps. The timing below is derived from
		 * the documented maximum frame rate and must be hardware-validated.
		 */
		.mode = IMX283_MODE_4,
		.bpp = 12,
		.width = 1824 + 32,
		.height = 370 + 4,
		.min_HMAX = 284,
		.min_VMAX = 1052,
		.default_HMAX = 285,
		.default_VMAX = 1052,
		.min_SHR = 16,
		.hbin_ratio = 3,
		.vbin_ratio = 1,
		.horizontal_ob = 32,
		.vertical_ob = 4,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
		.experimental = true,
	},
	{
		/*
		 * Readout mode 5: 3x horizontal binning with 2/19 vertical
		 * subsampling, 1824x190 active output, 12-bit.
		 *
		 * Sony specifies 452.03 fps. The timing below is derived from
		 * the documented maximum frame rate and must be hardware-validated.
		 */
		.mode = IMX283_MODE_5,
		.bpp = 12,
		.width = 1824 + 32,
		.height = 190 + 4,
		.min_HMAX = 284,
		.min_VMAX = 559,
		.default_HMAX = 285,
		.default_VMAX = 559,
		.min_SHR = 16,
		.hbin_ratio = 3,
		.vbin_ratio = 1,
		.horizontal_ob = 32,
		.vertical_ob = 4,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
		.experimental = true,
	},
	/*
	 * Aspect-ratio family for Mode 0 (WP-283-6), replacing the eighteen ad-hoc
	 * pixel-count crop entries this branch used to carry here: those were sizes,
	 * not framings, and a driver already carrying the hardest table in this file
	 * should not carry two families of it. Geometry is taken verbatim from
	 * development/experimental-crop-modes/ASPECT-RATIOS.md's imx283 table, which
	 * is the authority for these numbers -- do not recompute them here.
	 *
	 * VMAX stays at the full-frame floor for every ratio (WP-283-3): there is no
	 * mainline precedent or datasheet figure for a per-crop floor on this sensor,
	 * and mainline's own binned modes carry a *higher* floor than the full frame,
	 * so a shorter crop must not be assumed faster. Every entry below therefore
	 * runs at the same ~21.40fps as full Mode 0; what a ratio buys is field of
	 * view and a smaller file (MB/frame noted per entry, at 12-bit).
	 */
	{
		/* Mode 0, 12-bit 1x1, 1:1 crop -- 20.6 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 3648 + 96,
		.height = 3648 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 3648, 3648),
	},
	{
		/* Mode 0, 12-bit 1x1, 1.33:1 crop -- 27.3 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 4864 + 96,
		.height = 3648 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 4864, 3648),
	},
	{
		/* Mode 0, 12-bit 1x1, 1.37:1 crop -- 28.1 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5016 + 96,
		.height = 3648 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5016, 3648),
	},
	{
		/* Mode 0, 12-bit 1x1, 1.78:1 crop -- 25.9 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 3096 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 3225,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3096),
	},
	{
		/* Mode 0, 12-bit 1x1, 1.85:1 crop -- 24.8 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2972 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 3101,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2972),
	},
	{
		/* Mode 0, 12-bit 1x1, 1.89:1 crop -- 24.3 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2912 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 3041,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2912),
	},
	{
		/* Mode 0, 12-bit 1x1, 1.90:1 crop -- 24.2 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2896 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 3025,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2896),
	},
	{
		/* Mode 0, 12-bit 1x1, 2.00:1 crop -- 23.0 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2752 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 2881,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2752),
	},
	{
		/* Mode 0, 12-bit 1x1, 2.20:1 crop -- 20.9 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2504 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 2633,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2504),
	},
	{
		/* Mode 0, 12-bit 1x1, 2.22:1 crop -- 20.7 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2480 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 2609,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2480),
	},
	{
		/* Mode 0, 12-bit 1x1, 2.35:1 crop -- 19.6 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2344 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 2473,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2344),
	},
	{
		/* Mode 0, 12-bit 1x1, 2.39:1 crop -- 19.2 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2304 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 2433,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2304),
	},
	{
		/* Mode 0, 12-bit 1x1, 2.50:1 crop -- 18.4 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2204 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 2333,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2204),
	},
	{
		/* Mode 0, 12-bit 1x1, 2.53:1 crop -- 18.0 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 5472 + 96,
		.height = 2160 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 2289,
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 2160),
	},
	{
		/* Experimental Mode 0, 12-bit 1x1, 16:9 UHD crop -- 12.8 MB/frame */
		.mode = IMX283_MODE_0,
		.bpp = 12,
		.width = 3840 + 96,
		.height = 2160 + 16,
		.min_HMAX = 887,
		.min_VMAX = 3793,
		.crop_min_VMAX = 2305,          /* 2176 + 129 */
		.default_HMAX = 900,
		.default_VMAX = 4000,
		.min_SHR = 12,		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 3840, 2160),
		.experimental = false,
	},
	/* IMX283_MODE_2 aspect-ratio family. */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 3648, 3648, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 952, 108), /* 1:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 4850, 3648, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 351, 108), /* 1.33:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 4996, 3648, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 278, 108), /* 1.37:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 3648, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 108), /* 1.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 3074, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 395), /* 1.78:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2956, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 454), /* 1.85:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2894, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 485), /* 1.89:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2880, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 492), /* 1.90:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2736, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 564), /* 2.00:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2486, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 689), /* 2.20:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2464, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 700), /* 2.22:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2328, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 768), /* 2.35:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2288, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 788), /* 2.39:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2188, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 838), /* 2.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2, 12, 5472, 2144, 362, 3840, 2012, 375, 3840, 12, 1824, 2, 2, 48, 4, 40, 860), /* 2.55:1 */
	/* IMX283_MODE_2A aspect-ratio family. */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 3076, 3076, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 1238, 394), /* 1:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 4090, 3076, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 731, 394), /* 1.33:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 4214, 3076, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 669, 394), /* 1.37:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 4614, 3076, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 469, 394), /* 1.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 3074, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 395), /* 1.78:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2956, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 454), /* 1.85:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2894, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 485), /* 1.89:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2880, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 492), /* 1.90:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2736, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 564), /* 2.00:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2486, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 689), /* 2.20:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2464, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 700), /* 2.22:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2328, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 768), /* 2.35:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2288, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 788), /* 2.39:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2188, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 838), /* 2.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_2A, 12, 5472, 2144, 362, 3300, 1758, 375, 3300, 12, 1824, 2, 2, 48, 4, 40, 860), /* 2.55:1 */
	/* IMX283_MODE_3 aspect-ratio family. */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 3648, 3648, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 952, 108), /* 1:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 4851, 3648, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 350, 108), /* 1.33:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 4995, 3648, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 278, 108), /* 1.37:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 3648, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 108), /* 1.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 3072, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 396), /* 1.78:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2955, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 454), /* 1.85:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2895, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 484), /* 1.89:1 */	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2880, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 492), /* 1.90:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2736, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 564), /* 2.00:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2487, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 688), /* 2.20:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2463, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 700), /* 2.22:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2328, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 768), /* 2.35:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2289, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 787), /* 2.39:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2187, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 838), /* 2.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_3, 12, 5472, 2145, 284, 4200, 2980, 285, 4200, 16, 1234, 3, 3, 32, 4, 40, 859), /* 2.55:1 */
		/*
	 * Standard cinema crop families.
	 *
	 * Keep the active crop dimensions as the requested picture dimensions;
	 * IMX283_CROPPED_1X1_MODE() adds the sensor's 96-column horizontal and
	 * 16-line vertical optical-black transport area without changing the
	 * active image dimensions.
	 *
	 * 2K 16:9 = 2048x1152 and HD 16:9 = 1920x1080. The remaining entries
	 * retain the established even-pixel framing dimensions for their
	 * corresponding aspect ratios.
	 */
	/* 1x1 cropped-in 2K family. */
	IMX283_CROP_1X1(2048, 2048), /* 1:1 */
	IMX283_CROP_1X1(2048, 1540), /* 1.33:1 */
	IMX283_CROP_1X1(2048, 1494), /* 1.37:1 */
	IMX283_CROP_1X1(2048, 1364), /* 1.5:1 */
	IMX283_CROP_1X1(2048, 1152), /* 1.78:1 */
	IMX283_CROP_1X1(2048, 1106), /* 1.85:1 */
	IMX283_CROP_1X1(2048, 1084), /* 1.89:1 */
	IMX283_CROP_1X1(2048, 1078), /* 1.9:1 */
	IMX283_CROP_1X1(2048, 1024), /* 2:1 */
	IMX283_CROP_1X1(2048, 930), /* 2.2:1 */
	IMX283_CROP_1X1(2048, 922), /* 2.22:1 */
	IMX283_CROP_1X1(2048, 870), /* 2.35:1 */
	IMX283_CROP_1X1(2048, 856), /* 2.39:1 */
	IMX283_CROP_1X1(2048, 818), /* 2.5:1 */
	IMX283_CROP_1X1(2048, 802), /* 2.55:1 */
	/* 1x1 cropped-in HD family. */
	IMX283_CROP_1X1(1920, 1920), /* 1:1 */
	IMX283_CROP_1X1(1920, 1444), /* 1.33:1 */
	IMX283_CROP_1X1(1920, 1400), /* 1.37:1 */
	IMX283_CROP_1X1(1920, 1280), /* 1.5:1 */
	IMX283_CROP_1X1(1920, 1080), /* 1.78:1 */
	IMX283_CROP_1X1(1920, 1038), /* 1.85:1 */
	IMX283_CROP_1X1(1920, 1016), /* 1.89:1 */
	IMX283_CROP_1X1(1920, 1010), /* 1.9:1 */
	IMX283_CROP_1X1(1920, 960), /* 2:1 */
	IMX283_CROP_1X1(1920, 872), /* 2.2:1 */
	IMX283_CROP_1X1(1920, 864), /* 2.22:1 */
	IMX283_CROP_1X1(1920, 816), /* 2.35:1 */
	IMX283_CROP_1X1(1920, 802), /* 2.39:1 */
	IMX283_CROP_1X1(1920, 768), /* 2.5:1 */
	IMX283_CROP_1X1(1920, 752), /* 2.55:1 */

	/* IMX283_MODE_0 aspect-ratio family (1x1 UHD-window zoom family). */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 2160, 2160, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 852), /* 1:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 2879, 2160, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 852), /* 1.33:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 2959, 2160, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 852), /* 1.37:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3240, 2160, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 852), /* 1.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 2160, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 852), /* 1.78:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 2076, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 894), /* 1.85:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 2032, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 916), /* 1.89:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 2020, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 921), /* 1.90:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 1920, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 972), /* 2.00:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 1745, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 1059), /* 2.20:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 1729, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 1067), /* 2.22:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 1634, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 1115), /* 2.35:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 1607, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 1128), /* 2.39:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 1536, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 1164), /* 2.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_0, 12, 3840, 1506, 887, 3793, 2305, 900, 4000, 12, 3694, 1, 1, 96, 16, 856, 1179), /* 2.55:1 */
};
static const struct imx283_mode supported_modes_10bit[] = {
	{
		/* 5568x3664 25.48fps readout mode 1 */
		.mode = IMX283_MODE_1,
		.bpp = 10,
		.width = 5472 + 96,
		.height = 3648 + 16,
		.min_HMAX = 745,
		.min_VMAX = 3793,
		.default_HMAX = 750,
		.default_VMAX = 3840,
		.min_SHR = 12,
		/*
		 * All-pixel 1x1 readout (mdsel1 0x04), so the binning ratios
		 * are 1. They were absent until now, which happened to report
		 * correctly only because imx283_cfg_mode_binning has .min = 1
		 * and the control clamped the zero back up; nobody should have
		 * to rely on that. veff is mainline's 1x1 value (WP-283-3),
		 * spelled out for the same reason as on MODE_2: unused while
		 * the vertical-crop path stays Mode-0-only, a negative VWIDCUT
		 * waiting to happen if that gate is ever widened.
		 */
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3648),
	},
	{
		/* 5568x3094 30.17fps readout mode 1A */
		.mode = IMX283_MODE_1A,
		.bpp = 10,
		.width = 5472 + 96,
		.height = 3078 + 16,
		.min_HMAX = 745,
		.min_VMAX = 3203,
		.default_HMAX = 750,
		.default_VMAX = 3840,
		.min_SHR = 12,
		/*
		 * CONFIRMED: this is the vendor-preset 16:9 VCROP path
		 * (mdsel3/mdsel4 carry VCROP_EN). It therefore establishes that
		 * a windowed readout can have a shorter VMAX.
		 *
		 * The shipped min_VMAX 3203 back-solves from the 29.97 fps target
		 * in originating commit e6fc463:
		 *     72e6 / (750 x 3203) ~= 29.97.
		 * It is therefore not a datasheet-authoritative floor.
		 *
		 * UNKNOWN: that does not prove that an arbitrary driver-programmed
		 * Mode-0 VWIDCUT window can use the same rule; that remains a hardware
		 * measurement question.
		 */
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3078),
	},
	{
		/*
		 * Readout mode 1S: 3000x3000 square 10-bit readout.
		 * PROBABLE/UNSOURCED: min_VMAX 2235 cannot be reconciled with the
		 * advertised 3016-line transport frame; it would make the VBLANK
		 * floor negative. The value could not be traced to a source commit
		 * or datasheet figure, so it must not be used as evidence for the
		 * HMAX/VMAX transport model until measured or otherwise sourced.
		 */
		.mode = IMX283_MODE_1S,
		.bpp = 10,
		.width = 3000 + 96,
		.height = 3000 + 16,
		.min_HMAX = 745,
		.min_VMAX = 2235,
		.default_HMAX = 750,
		.default_VMAX = 2235,
		.min_SHR = 12,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		/*
		 * 3000x3000, not 5472x3000. This entry outputs .width =
		 * 3000 + 96 at hbin_ratio 1, so the window it scans is 3000
		 * columns wide; claiming 5472 made libcamera describe a
		 * 3000-column picture as a crop of the full array width. The
		 * same class of defect as MODE_1C's .top above and MODE_2's
		 * missing ratios below, and the horizontal check in
		 * imx283_check_mode_table() now catches all three.
		 *
		 * Unlike MODE_1C this also moves HTRIMMING_START (40 ->
		 * 1276), which is safe to do here and not there: MODE_1S is
		 * .experimental, gated off behind the experimental_modes
		 * module param, and has never been enabled on hardware -- so
		 * there is no shipping framing to preserve, and the state it
		 * was in could not have been right either way.
		 */
		.crop = CENTERED_RECTANGLE(imx283_active_area, 3000, 3000),
		.experimental = true,
	},
	{
		/*
		 * Readout mode 6: 2x2 binned 16:9 10-bit readout.
		 * Sony specifies 2736x1538 at 60.01 fps. Timing is derived
		 * from the documented maximum frame rate and must be validated.
		 */
		.mode = IMX283_MODE_6,
		.bpp = 10,
		.width = 2736 + 48,
		.height = 1538 + 4,
		.min_HMAX = 745,
		.min_VMAX = 1600,
		.default_HMAX = 750,
		.default_VMAX = 1600,
		.min_SHR = 12,
		.hbin_ratio = 2,
		.vbin_ratio = 2,
		.horizontal_ob = 48,
		.vertical_ob = 4,
		.crop = CENTERED_RECTANGLE(imx283_active_area, 5472, 3076),
		.experimental = true,
	},
	{
		/* 3936x2176 (3840x2160 active) 60.16fps readout mode 1C - UHD 4K 16:9 crop */
		.mode = IMX283_MODE_1C,
		.bpp = 10,
		.width = 3840 + 96,
		.height = 2160 + 16,
		.min_HMAX = 544,
		.min_VMAX = 2200,
		.default_HMAX = 576,
		.default_VMAX = 2500,
		.min_SHR = 12,
		/*
		 * 1x1, no binning; every other 1x1 entry in this file (and
		 * mainline's mode 0) uses veff 3694 (WP-283-3). Landed via
		 * the Pi-validated 6.12.y merge (WP-283-1), unlike this
		 * fork's own unvalidated 1S/4/5/6 additions.
		 */
		.veff = 3694,
		.vst = 0,
		.vct = 0,
		.hbin_ratio = 1,
		.vbin_ratio = 1,
		.horizontal_ob = 96,
		.vertical_ob = 16,
		/*
		 * Centred on the active area like every other entry in this
		 * file, replacing the private `imx283_UHD_area` rectangle
		 * this entry used to be centred on -- grep that name and you
		 * land here.
		 *
		 * That rectangle was {.left = 236, .top = 0}. Both numbers
		 * arrived underived in "add UHD mode" (7751c32) and the
		 * comment that later justified them ("the 0x30 readout drive
		 * mode addresses the array differently from the all-pixel
		 * modes", 95183c8) cites nothing -- no datasheet window, no
		 * register read-back. .top = 0 was provably wrong whatever
		 * 0x30 does: this field is a *native* pixel-array coordinate
		 * (imx283_active_area itself starts at .top = 108), so an
		 * active-pixel rectangle can never start at line 0. libcamera
		 * builds analogCrop by subtracting the active-area origin and
		 * duly reported a negative origin on the Pi:
		 *
		 *   3936x2176 [60.16 fps - (196, -108)/3840x2160 crop;
		 *              binning 1x1; mode-crop (236,0)/3840x2160]
		 *
		 * against, for a correct mode, mode-crop and analogCrop
		 * differing by exactly the active origin (40,108). Reading
		 * .top = 0 as "the window starts at the top of the array" is
		 * the most likely origin of the bug: right idea, wrong
		 * coordinate space.
		 *
		 * What the registers here can and cannot settle:
		 *  - 0x30 is a 1x1 *window*, not a subsampling mode: mdsel2
		 *    0x41 is the all-pixel 10-bit 0x01 plus the same bit 6
		 *    that IMX283_MODE_1S (the 3000x3000 crop) sets, and no
		 *    integer decimation can produce this frame -- 3840 active
		 *    columns would need 7680 of a 5472-column array at 2x, and
		 *    2160 lines would need 4320 of 3648; 5472/3840 = 1.425 is
		 *    not a ratio the sensor offers. So .width/.height and
		 *    hbin/vbin 1x1 above are right. (Do not argue this from
		 *    the timing floors: min_HMAX and min_VMAX track the OUTPUT
		 *    frame, not the scanned window. IMX283_MODE_3 bins 3x
		 *    horizontally -- it scans all 5472 columns -- and still
		 *    declares min_HMAX 284 against mode 0's 887.)
		 *  - .crop.top is metadata only for this entry: the arbitrary
		 *    vertical-crop path in imx283_start_streaming() is
		 *    Mode-0-only, and 0x30's mdsel3/mdsel4 do not even set the
		 *    VCROP_EN bits, so VWINPOS/VWIDCUT are never written and
		 *    the vertical window is whatever drive mode 0x30 hardwires.
		 *    Nothing in this driver can tell us where that is.
		 *  - .crop.left is NOT metadata: it is written as
		 *    HTRIMMING_START for every mode. Centring therefore moves
		 *    the real horizontal window from native column 236 to 856,
		 *    i.e. 620 px to the right -- a visible reframing of this
		 *    mode, and the fix for it if 236 was ever honoured.
		 *
		 * .left = 236 is HARDWARE-CONFIRMED. Do not centre it.
		 *
		 * The two coordinates are not the same kind of thing:
		 *
		 *  - .top is metadata for this entry. The arbitrary
		 *    vertical-crop path in imx283_start_streaming() is
		 *    Mode-0-only, and 0x30's mdsel3/mdsel4 do not set the
		 *    VCROP_EN bits, so VWINPOS/VWIDCUT are never written here.
		 *    852 removes the negative analogCrop libcamera used to
		 *    report and changes nothing the sensor does.
		 *  - .left is NOT metadata: it is written as HTRIMMING_START
		 *    for every mode.
		 *
		 * It was briefly changed to 856 to centre the window, on the
		 * argument that every other entry is centred and that 236 had
		 * arrived underived in 7751c32 with a later comment (95183c8)
		 * that cited nothing. The sensor settled it immediately: at
		 * 856 this mode streams a frame whose left portion is a grey
		 * ramp and whose right two thirds are vertical colour noise --
		 * a window running off the end of what drive mode 0x30
		 * actually reads. At 236 it is a clean picture.
		 *
		 * So 95183c8's claim was right even though it showed no
		 * working: 0x30 DOES address the array differently from the
		 * all-pixel modes, and 236 is where its window starts. The
		 * defect was only ever .top, which named a row inside the
		 * optical black.
		 *
		 * The consequence is that this mode's crop is genuinely
		 * off-centre horizontally, and the settings page draws its
		 * crop box left of centre because that is the truth. Changing
		 * the picture to match the diagram is not available: the
		 * hardware decides where this window is.
		 */
		.crop = {
			.left   = 236,
			.top    = 852,
			.width  = 3840,
			.height = 2160,
		},
	},
	/* IMX283_MODE_1 aspect-ratio family. */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 3648, 3648, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 952, 108), /* 1:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 4851, 3648, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 350, 108), /* 1.33:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 4997, 3648, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 277, 108), /* 1.37:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 3648, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 108), /* 1.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 3074, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 395), /* 1.78:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2957, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 453), /* 1.85:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2895, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 484), /* 1.89:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2880, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 492), /* 1.90:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2736, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 564), /* 2.00:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2487, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 688), /* 2.20:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2464, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 700), /* 2.22:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2328, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 768), /* 2.35:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2289, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 787), /* 2.39:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2188, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 838), /* 2.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1, 10, 5472, 2145, 745, 3793, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 859), /* 2.55:1 */
	/* IMX283_MODE_1A aspect-ratio family. */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 3078, 3078, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 1237, 393), /* 1:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 4093, 3078, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 729, 393), /* 1.33:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 4216, 3078, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 668, 393), /* 1.37:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 4617, 3078, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 467, 393), /* 1.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 3074, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 395), /* 1.78:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2957, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 453), /* 1.85:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2895, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 484), /* 1.89:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2880, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 492), /* 1.90:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2736, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 564), /* 2.00:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2487, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 688), /* 2.20:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2464, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 700), /* 2.22:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2328, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 768), /* 2.35:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2289, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 787), /* 2.39:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2188, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 838), /* 2.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1A, 10, 5472, 2145, 745, 3203, 0, 750, 3840, 12, 3694, 1, 1, 96, 16, 40, 859), /* 2.55:1 */
	/* IMX283_MODE_1C aspect-ratio family (10-bit UHD-window family). */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 2160, 2160, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 1076, 852), /* 1:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 2879, 2160, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 720, 852), /* 1.33:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 2959, 2160, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 676, 852), /* 1.37:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3240, 2160, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 536, 852), /* 1.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 2160, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 852), /* 1.78:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 2076, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 894), /* 1.85:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 2032, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 916), /* 1.89:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 2020, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 922), /* 1.90:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 1920, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 972), /* 2.00:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 1745, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 1059), /* 2.20:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 1729, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 1067), /* 2.22:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 1634, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 1115), /* 2.35:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 1607, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 1128), /* 2.39:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 1536, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 1164), /* 2.50:1 */
	IMX283_ASPECT_MODE(IMX283_MODE_1C, 10, 3840, 1506, 544, 2200, 0, 576, 2500, 12, 3694, 1, 1, 96, 16, 236, 1179), /* 2.55:1 */
};

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* regulator supplies */
static const char * const imx283_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.1V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define imx283_NUM_SUPPLIES ARRAY_SIZE(imx283_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define imx283_XCLR_MIN_DELAY_US	100000
#define imx283_XCLR_DELAY_RANGE_US	1000

struct imx283 {
	struct device *dev;

	const struct imx283_input_frequency *freq;

	/* Selected link_frequency */
	unsigned int link_freq_idx;

	struct v4l2_subdev sd;
	struct media_pad pad;

	unsigned int fmt_code;

	struct clk *xclk;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx283_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/*
	 * Read-only mode-geometry metadata (WP-283-5), updated whenever the
	 * active mode changes. Values are the driver's own hbin_ratio and
	 * the mode->crop rectangle, both already in native sensor-pixel
	 * coordinates -- no domain conversion is needed here, unlike a
	 * sensor whose crop is stored in the output domain.
	 */
	struct v4l2_ctrl *mode_binning_ctrl;
	struct v4l2_ctrl *mode_crop_left_ctrl;
	struct v4l2_ctrl *mode_crop_top_ctrl;
	struct v4l2_ctrl *mode_crop_width_ctrl;
	struct v4l2_ctrl *mode_crop_height_ctrl;
	struct v4l2_ctrl *mode_active_left_ctrl;
	struct v4l2_ctrl *mode_active_top_ctrl;

	/* Current mode */
	const struct imx283_mode *mode;

	u16 hmax;
	u32 vmax;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};



int cci_read(struct imx283 *imx283, u32 reg, u64 *val, int *err) {
    if (err && *err)
        return *err;

    struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
    u32 reg_addr = reg & CCI_REG_ADDR_MASK;
    u32 width = (reg & CCI_REG_WIDTH_MASK) >> CCI_REG_WIDTH_SHIFT;
    u8 addr_buf[2] = { reg_addr >> 8, reg_addr & 0xff };
    u8 data_buf[8] = { 0 };  // Max 8 bytes for 64-bit data
    struct i2c_msg msgs[2];
    int ret;

    if (width == 0 || width > 8) {
        if (err) *err = -EINVAL;
        return -EINVAL;
    }

    // Setup I2C message to write the register address
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = sizeof(addr_buf);
    msgs[0].buf = addr_buf;

    // Setup I2C message to read data from the register
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = width;
    msgs[1].buf = data_buf;

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret != 2) {
        if (err) *err = -EIO;
        return -EIO;
    }

    // Assuming big-endian register format
    *val = 0;
    for (int i = 0; i < width; i++) {
        *val = (*val << 8) | data_buf[i];
    }

    return 0;
}


int cci_write(struct imx283 *imx283, u32 reg, u64 val, int *err) {

    struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
    u32 reg_addr = reg & CCI_REG_ADDR_MASK;
    u32 width = (reg & CCI_REG_WIDTH_MASK) >> CCI_REG_WIDTH_SHIFT;
    bool is_le = reg & CCI_REG_LE;
    u8 buf[10]; // Maximum size needed: 2 bytes for address + 8 bytes for data
    int ret, i;

    // Set the register address (big-endian)
    buf[0] = (reg_addr >> 8) & 0xff;
    buf[1] = reg_addr & 0xff;

    // Set the data
    for (i = 0; i < width; i++) {
        if (is_le) {
            // Little-endian: lower address bytes have lower value bytes
            buf[2 + i] = (val >> (8 * i)) & 0xff;
        } else {
            // Big-endian: lower address bytes have higher value bytes
            buf[2 + width - 1 - i] = (val >> (8 * i)) & 0xff;
        }
    }

    ret = i2c_master_send(client, buf, 2 + width);
    if (ret < 0) {
        if (err) *err = ret;
        return ret;
    }

    return 0;
}


int cci_multi_reg_write(struct imx283 *imx283, const struct cci_reg_sequence *regs, unsigned int num_regs, int *err) {

    for (unsigned int i = 0; i < num_regs; i++) {
        *err = cci_write(imx283, regs[i].reg, regs[i].val, err);
        if (*err)
            return *err;
    }

    return 0;
}



static inline struct imx283 *to_imx283(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx283, sd);
}

/*
 * Copy the entries of `src` into `dst` (sized by the caller to hold all of
 * `src_count` of them), skipping `.experimental` entries unless
 * experimental_modes is set. Returns the number of entries copied.
 *
 * Filtering here, once, keeps every other mode's position and count
 * unchanged in the returned list; nothing but the four experimental
 * entries (IMX283_MODE_1S/_4/_5/_6, see the module parameter above) is
 * affected.
 */
static unsigned int build_filtered_mode_table(const struct imx283_mode *src,
					       unsigned int src_count,
					       struct imx283_mode *dst)
{
	unsigned int i, n = 0;

	for (i = 0; i < src_count; i++) {
		if (src[i].experimental && !experimental_modes)
			continue;
		dst[n++] = src[i];
	}

	return n;
}

/*
 * Probe-time self-check on the mode tables above.
 *
 * Both defects this exists for shipped as plain omissions in static data
 * that nothing in the driver ever looked at, and both only surfaced when
 * a human read `--list-cameras` on a Pi:
 *
 *  - IMX283_MODE_1C advertised a crop with .top = 0, outside the active
 *    area, so libcamera's analogCrop origin came out negative;
 *  - IMX283_MODE_2 omitted its binning ratios, so a 2x2-binned mode
 *    advertised "binning 1x1" (the control's .min = 1 hid the zero).
 *
 * Both are one comparison each, and the comparison is cheap enough to
 * run over every entry on every probe, experimental ones included --
 * those are exactly the entries nobody has looked at.
 *
 * Deliberately a warning, not a probe failure: the tables are
 * compile-time data, so nothing can be repaired at runtime, and a camera
 * that still streams with one wrong metadata field is worth more to the
 * operator than one that refuses to bind. dmesg is where the next reader
 * of this driver is already looking.
 */
static void imx283_check_mode_table(struct device *dev, const char *name,
				    const struct imx283_mode *modes,
				    unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		const struct v4l2_rect *crop = &modes[i].crop;
		s32 right = crop->left + (s32)crop->width;
		s32 bottom = crop->top + (s32)crop->height;

		if (crop->left < imx283_active_area.left ||
		    crop->top < imx283_active_area.top ||
		    right > imx283_active_area.left + (s32)imx283_active_area.width ||
		    bottom > imx283_active_area.top + (s32)imx283_active_area.height)
			dev_warn(dev,
				 "%s[%u] (%ux%u, readout-mode enum %u): crop (%d,%d)/%ux%u is not inside the active area (%d,%d)/%ux%u\n",
				 name, i, modes[i].width, modes[i].height,
				 modes[i].mode,
				 crop->left, crop->top, crop->width, crop->height,
				 imx283_active_area.left, imx283_active_area.top,
				 imx283_active_area.width,
				 imx283_active_area.height);

		if (!modes[i].hbin_ratio || !modes[i].vbin_ratio)
			dev_warn(dev,
				 "%s[%u] (%ux%u, readout-mode enum %u): binning ratio unset (h=%u v=%u), advertised binning will be wrong\n",
				 name, i, modes[i].width, modes[i].height,
				 modes[i].mode,
				 modes[i].hbin_ratio, modes[i].vbin_ratio);

		/*
		 * The crop must be the window the transport frame came out
		 * of: active output columns times the horizontal binning
		 * ratio. Containment above is not enough -- it passes any
		 * rectangle that happens to sit inside the array, which is
		 * how IMX283_MODE_1S shipped a 5472-column crop for a
		 * 3000-column readout. This is also the check that catches a
		 * binning ratio that is merely WRONG rather than zero, which
		 * the test above cannot see: imx283_cfg_mode_binning clamps
		 * to 1..3, so a bad ratio is advertised as a plausible one.
		 *
		 * Horizontal only. The vertical analogue does not hold for
		 * IMX283_MODE_4 and _5, which thin the frame by reading fewer
		 * lines rather than by binning them, so their vbin_ratio is 1
		 * while crop.height is the full 3648. There is no field in
		 * this struct that distinguishes subsampling from binning, so
		 * there is nothing to check them against; add one before
		 * adding the vertical test, or it will fire on two entries
		 * that are correct.
		 */
		if (modes[i].hbin_ratio) {
			struct v4l2_rect output_crop = imx283_output_crop(&modes[i]);
			unsigned int output_width = imx283_output_width(&modes[i]);
			if (output_width * modes[i].hbin_ratio != output_crop.width)
				dev_warn(dev,
					 "%s[%u] (%ux%u, readout-mode enum %u): active crop width %u does not match %u output x %u binning = %u\n",
					 name, i, modes[i].width, modes[i].height, modes[i].mode,
					 output_crop.width, output_width, modes[i].hbin_ratio,
					 output_width * modes[i].hbin_ratio);
		}
	}
}

static inline void get_mode_table(unsigned int code,
				  const struct imx283_mode **mode_list,
				  unsigned int *num_modes)
{
	static struct imx283_mode filtered_12bit[ARRAY_SIZE(supported_modes_12bit)];
	static struct imx283_mode filtered_10bit[ARRAY_SIZE(supported_modes_10bit)];
	static unsigned int num_filtered_12bit;
	static unsigned int num_filtered_10bit;
	static bool filtered_tables_built;

	/*
	 * experimental_modes is a boot-time-only parameter (module_param
	 * ... 0444, no runtime write), so the filtered tables never need
	 * to be rebuilt once populated.
	 */
	if (!filtered_tables_built) {
		num_filtered_12bit = build_filtered_mode_table(supported_modes_12bit,
								ARRAY_SIZE(supported_modes_12bit),
								filtered_12bit);
		num_filtered_10bit = build_filtered_mode_table(supported_modes_10bit,
								ARRAY_SIZE(supported_modes_10bit),
								filtered_10bit);
		filtered_tables_built = true;
	}

	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = filtered_12bit;
		*num_modes = num_filtered_12bit;
		break;
	/* 10-bit */
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		*mode_list = filtered_10bit;
		*num_modes = num_filtered_10bit;
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Get bayer order based on flip setting. */
static u32 imx283_get_format_code(struct imx283 *imx283, u32 code)
{
	unsigned int i;
	lockdep_assert_held(&imx283->mutex);
	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	return codes[i];
}

static void imx283_set_default_format(struct imx283 *imx283)
{
	/* Set default mode to max resolution */
	imx283->mode = &supported_modes_12bit[0];
	imx283->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
}

// Move this to .init_cfg
static int imx283_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx283 *imx283 = to_imx283(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);

	struct v4l2_rect *try_crop;

	mutex_lock(&imx283->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = imx283_output_width(&supported_modes_12bit[0]);
	try_fmt_img->height = imx283_output_height(&supported_modes_12bit[0]);
	try_fmt_img->code = imx283_get_format_code(imx283,
						   MEDIA_BUS_FMT_SRGGB12_1X12);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_crop to the selected default mode's active area. */
	try_crop = v4l2_subdev_state_get_crop(fh->state, IMAGE_PAD);
	*try_crop = imx283_output_crop(imx283->mode);

	mutex_unlock(&imx283->mutex);

	return 0;
}

static u64 calculate_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 shr, u64 svr, u64 offset) {
    u64 numerator;
    numerator = (vmax * (svr + 1) - shr) * hmax + offset;

    do_div(numerator, hmax);
    numerator = clamp_t(uint32_t, numerator, 0, 0xFFFFFFFF);
    return numerator;
}

static void calculate_min_max_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 min_shr, u64 svr, u64 offset, u64 *min_exposure, u64 *max_exposure) {
    u64 max_shr = (svr + 1) * vmax - 4;
    max_shr = min_t(uint64_t, max_shr, 0xFFFF);

    *min_exposure = calculate_v4l2_cid_exposure(hmax, vmax, max_shr, svr, offset);
    *max_exposure = calculate_v4l2_cid_exposure(hmax, vmax, min_shr, svr, offset);
}


/*
Integration Time [s] = [{VMAX × (SVR + 1) – (SHR)}
 × HMAX + offset] / (72 × 10^6)

Integration Time [s] = exposure * HMAX / (72 × 10^6)
*/

static uint32_t calculate_shr(uint32_t exposure, uint32_t hmax, uint64_t vmax, uint32_t svr, uint32_t offset) {
    uint64_t temp;
    uint32_t shr;

    temp = ((uint64_t)exposure * hmax - offset);
    do_div(temp, hmax);
    shr = (uint32_t)(vmax * (svr + 1) - temp);

    return shr;
}

static const char * const imx283_tpg_menu[] = {
	"Disabled",
	"All 000h",
	"All FFFh",
	"All 555h",
	"All AAAh",
	"Horizontal color bars",
	"Vertical color bars",
};

static const int imx283_tpg_val[] = {
	IMX283_TPG_PAT_ALL_000,
	IMX283_TPG_PAT_ALL_000,
	IMX283_TPG_PAT_ALL_FFF,
	IMX283_TPG_PAT_ALL_555,
	IMX283_TPG_PAT_ALL_AAA,
	IMX283_TPG_PAT_H_COLOR_BARS,
	IMX283_TPG_PAT_V_COLOR_BARS,
};

static int imx283_update_test_pattern(struct imx283 *imx283, u32 pattern_index)
{
	int ret;

	if (pattern_index >= ARRAY_SIZE(imx283_tpg_val))
		return -EINVAL;

	if (pattern_index) {
		ret = cci_write(imx283, IMX283_REG_TPG_PAT,
				imx283_tpg_val[pattern_index], NULL);
		if (ret)
			return ret;

		ret = cci_write(imx283, IMX283_REG_TPG_CTRL,
				IMX283_TPG_CTRL_CLKEN | IMX283_TPG_CTRL_PATEN, NULL);
	} else {
		ret = cci_write(imx283, IMX283_REG_TPG_CTRL, 0x00, NULL);
	}

	return ret;
}

static int imx283_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx283 *imx283 =
		container_of(ctrl->handler, struct imx283, ctrl_handler);
	const struct imx283_mode *mode = imx283->mode;
	u64 shr, pixel_rate, hmax = 0;
	int ret = 0;

	//state = v4l2_subdev_get_locked_active_state(&imx283->sd);
	//format = v4l2_subdev_get_pad_format(&imx283->sd, state, 0);

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK){
		/* Honour the VBLANK limits when setting exposure. */
		u64 current_exposure, max_exposure, min_exposure, vmax;
		vmax = ((u64)imx283_output_height(mode) + ctrl->val) ;
		imx283->vmax = vmax;

		calculate_min_max_v4l2_cid_exposure(imx283->hmax, imx283->vmax,
						    (u64)mode->min_SHR, 0, 209,
						    &min_exposure, &max_exposure);

		current_exposure = clamp_t(uint32_t, current_exposure, min_exposure, max_exposure);

		dev_info(imx283->dev,"exposure_max:%lld, exposure_min:%lld, current_exposure:%lld\n",max_exposure, min_exposure, current_exposure);
		dev_info(imx283->dev, "\tVMAX:%d, HMAX:%d\n", imx283->vmax, imx283->hmax);
		__v4l2_ctrl_modify_range(imx283->exposure, min_exposure,max_exposure, 1,current_exposure);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(imx283->dev) == 0)
		return 0;

	
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		{
		dev_info(imx283->dev,"V4L2_CID_EXPOSURE : %d\n",ctrl->val);
		dev_info(imx283->dev,"\tvblank:%d, hblank:%d\n",imx283->vblank->val, imx283->hblank->val);
		dev_info(imx283->dev, "\tVMAX:%d, HMAX:%d\n", imx283->vmax, imx283->hmax);
		shr = calculate_shr(ctrl->val, imx283->hmax, imx283->vmax, 0, 209);
		dev_info(imx283->dev,"\tSHR:%lld\n",shr);
		ret = cci_write(imx283, IMX283_REG_SHR, shr, NULL);

		}
		break;

	case V4L2_CID_HBLANK:
		{
		dev_info(imx283->dev, "V4L2_CID_HBLANK : %d\n", ctrl->val);
		//int hmax = (IMX283_NATIVE_WIDTH + ctrl->val) * 72000000; / IMX283_PIXEL_RATE;
		pixel_rate = (u64)imx283_output_width(mode) * 72000000;
		do_div(pixel_rate, mode->min_HMAX);
		hmax = (u64)(imx283_output_width(mode) + ctrl->val) * 72000000;
		do_div(hmax, pixel_rate);
		imx283->hmax = hmax;
		dev_info(imx283->dev, "\tHMAX : %d\n", imx283->hmax);
		ret = cci_write(imx283, IMX283_REG_HMAX, hmax, NULL);
		}
		break;

	case V4L2_CID_VBLANK:
		{
		dev_info(imx283->dev,"V4L2_CID_VBLANK : %d\n",ctrl->val);
		imx283->vmax = ((u64)imx283_output_height(mode) + ctrl->val);
		dev_info(imx283->dev, "\tVMAX : %d\n", imx283->vmax);
		ret = cci_write(imx283, IMX283_REG_VMAX, imx283->vmax, NULL);
		}
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		dev_info(imx283->dev, "V4L2_CID_ANALOGUE_GAIN : %d\n", ctrl->val);
		ret = cci_write(imx283, IMX283_REG_ANALOG_GAIN, ctrl->val, NULL);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		dev_info(imx283->dev, "V4L2_CID_DIGITAL_GAIN : %d\n", ctrl->val);
		ret = cci_write(imx283, IMX283_REG_DIGITAL_GAIN, ctrl->val, NULL);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		//dev_info(imx283->dev,"V4L2_CID_HFLIP : %d\n",imx283->hflip->val);
		//dev_info(imx283->dev,"V4L2_CID_VFLIP : %d\n",imx283->vflip->val);
		//ret = imx283_write_reg_1byte(imx283, IMX283_REG_VFLIP, imx283->vflip->val);
		break;

	case V4L2_CID_TEST_PATTERN:
		ret = imx283_update_test_pattern(imx283, ctrl->val);
		break;

	default:
		dev_info(imx283->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		//ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx283->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx283_ctrl_ops = {
	.s_ctrl = imx283_set_ctrl,
};

/*
 * Read-only mode-geometry controls (WP-283-5). .max on the crop controls
 * mirrors imx283_native_area's width/height (5592 x 3710): a designated
 * initializer cannot reference another static const struct's field here,
 * so the native array size is repeated as a literal.
 */
static const struct v4l2_ctrl_config imx283_cfg_mode_binning = {
	.ops = &imx283_ctrl_ops, .id = V4L2_CID_IMX283_MODE_BINNING,
	.name = "Mode Binning", .type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1, .max = 3, .step = 1, .def = 1,
};
static const struct v4l2_ctrl_config imx283_cfg_mode_crop_left = {
	.ops = &imx283_ctrl_ops, .id = V4L2_CID_IMX283_MODE_CROP_LEFT,
	.name = "Mode Crop Left", .type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0, .max = 5592, .step = 1, .def = 0,
};
static const struct v4l2_ctrl_config imx283_cfg_mode_crop_top = {
	.ops = &imx283_ctrl_ops, .id = V4L2_CID_IMX283_MODE_CROP_TOP,
	.name = "Mode Crop Top", .type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0, .max = 3710, .step = 1, .def = 0,
};
static const struct v4l2_ctrl_config imx283_cfg_mode_crop_width = {
	.ops = &imx283_ctrl_ops, .id = V4L2_CID_IMX283_MODE_CROP_WIDTH,
	.name = "Mode Crop Width", .type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0, .max = 5592, .step = 1, .def = 5592,
};
static const struct v4l2_ctrl_config imx283_cfg_mode_crop_height = {
	.ops = &imx283_ctrl_ops, .id = V4L2_CID_IMX283_MODE_CROP_HEIGHT,
	.name = "Mode Crop Height", .type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0, .max = 3710, .step = 1, .def = 3710,
};

/*
 * Where the picture starts inside the frame the sensor actually sends, in
 * that frame's own pixels -- NOT native sensor coordinates like the four
 * above. A DNG writer needs exactly this to emit ActiveArea; without it the
 * optical black ships as part of the image and every renderer draws it.
 *
 * Measured on a CM5, both depths and both binnings (see the commit that
 * added these):
 *   MODE_1C  3936x2176 10-bit: cols 0..95 at black level, col 96 is picture;
 *                              rows 2160..2175 zero.
 *   MODE_2   2784x1828 12-bit: cols 0..47 at black level, col 48 is picture;
 *                              rows 1824..1827 zero.
 * i.e. horizontal_ob columns LEADING, vertical_ob rows TRAILING, in the
 * mode's own (post-binning) pixels. Hence left = horizontal_ob and top = 0.
 *
 * The max is the widest frame in either table (5568) rather than the native
 * array width: this is a frame coordinate, so it can never exceed the frame.
 */
static const struct v4l2_ctrl_config imx283_cfg_mode_active_left = {
	.ops = &imx283_ctrl_ops, .id = V4L2_CID_IMX283_MODE_ACTIVE_LEFT,
	.name = "Mode Active Left", .type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0, .max = 5568, .step = 1, .def = 0,
};
static const struct v4l2_ctrl_config imx283_cfg_mode_active_top = {
	.ops = &imx283_ctrl_ops, .id = V4L2_CID_IMX283_MODE_ACTIVE_TOP,
	.name = "Mode Active Top", .type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0, .max = 3664, .step = 1, .def = 0,
};

static int imx283_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx283 *imx283 = to_imx283(sd);

	if (code->index >= (ARRAY_SIZE(codes) / 4))
		return -EINVAL;

	code->code = imx283_get_format_code(imx283, codes[code->index * 4]);

	return 0;
}

static int imx283_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx283 *imx283 = to_imx283(sd);

	const struct imx283_mode *mode_list;
	unsigned int num_modes;

	get_mode_table(fse->code, &mode_list, &num_modes);

	if (fse->index >= num_modes)
		return -EINVAL;

	if (fse->code != imx283_get_format_code(imx283, fse->code))
		return -EINVAL;

	fse->min_width = imx283_output_width(&mode_list[fse->index]);
	fse->max_width = fse->min_width;
	fse->min_height = imx283_output_height(&mode_list[fse->index]);
	fse->max_height = fse->min_height;

	return 0;
}
static void imx283_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx283_update_image_pad_format(struct imx283 *imx283,
					   const struct imx283_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = imx283_output_width(mode);
	fmt->format.height = imx283_output_height(mode);
	fmt->format.field = V4L2_FIELD_NONE;
	imx283_reset_colorspace(&fmt->format);
}

static int imx283_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx283 *imx283 = to_imx283(sd);

	mutex_lock(&imx283->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = imx283_get_format_code(imx283, try_fmt->code);
		fmt->format = *try_fmt;
	} else {
		imx283_update_image_pad_format(imx283, imx283->mode, fmt);
		fmt->format.code = imx283_get_format_code(imx283, imx283->fmt_code);
	}

	mutex_unlock(&imx283->mutex);
	return 0;
}

/* TODO */
/*
 * Report the active mode's real binning and sensor crop through the
 * read-only "Mode Binning" / "Mode Crop *" controls (WP-283-5, DEC-4).
 * mode->crop is already in native sensor-pixel coordinates (see the
 * imx283_mode field comment and CENTERED_RECTANGLE), so it is reported * as-is. hbin_ratio and vbin_ratio differ on a few entries (the 3x1
 * horizontal-only-binning modes); DEC-4 says report the horizontal ratio
 * in that case, which is what "Mode Binning" always does here.
 */
static void imx283_update_mode_metadata(struct imx283 *imx283,
					 const struct imx283_mode *mode)
{
	__v4l2_ctrl_s_ctrl(imx283->mode_binning_ctrl, mode->hbin_ratio);
	{
		struct v4l2_rect output_crop = imx283_output_crop(mode);
		__v4l2_ctrl_s_ctrl(imx283->mode_crop_left_ctrl, output_crop.left);
		__v4l2_ctrl_s_ctrl(imx283->mode_crop_top_ctrl, output_crop.top);
		__v4l2_ctrl_s_ctrl(imx283->mode_crop_width_ctrl, output_crop.width);
		__v4l2_ctrl_s_ctrl(imx283->mode_crop_height_ctrl, output_crop.height);
	}
	/*
	 * Frame coordinates, not sensor coordinates. horizontal_ob/vertical_ob
	 * are expressed in the mode's transport pixels, so they need no scaling.
	 * VOB is trailing rather than a top offset, hence active-top is zero.
	 */
	__v4l2_ctrl_s_ctrl(imx283->mode_active_left_ctrl,
					mode->horizontal_ob);
	__v4l2_ctrl_s_ctrl(imx283->mode_active_top_ctrl, 0);
}

static u64 imx283_min_vmax(const struct imx283_mode *mode)
{
	if (crop_vmax && mode->crop_min_VMAX)
		return mode->crop_min_VMAX;
	return mode->min_VMAX;
}

static void imx283_set_framing_limits(struct imx283 *imx283)
{
	const struct imx283_mode *mode = imx283->mode;
	u64 def_hblank;
	u64 pixel_rate;

	imx283_update_mode_metadata(imx283, mode);

	imx283->vmax = mode->default_VMAX;
	imx283->hmax = mode->default_HMAX;

	pixel_rate = (u64)imx283_output_width(mode) * 72000000;
	do_div(pixel_rate,mode->min_HMAX);
	dev_info(imx283->dev,"Pixel Rate : %lld\n",pixel_rate);


	//int def_hblank = mode->default_HMAX * IMX283_PIXEL_RATE / 72000000 - IMX283_NATIVE_WIDTH;
	def_hblank = mode->default_HMAX * pixel_rate;
	do_div(def_hblank, 72000000);
	def_hblank = def_hblank - imx283_output_width(mode);
	__v4l2_ctrl_modify_range(imx283->hblank, 0,
				 IMX283_HMAX_MAX, 1, def_hblank);
	__v4l2_ctrl_s_ctrl(imx283->hblank, def_hblank);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx283->vblank,
				 imx283_min_vmax(mode) - imx283_output_height(mode),
				 IMX283_VMAX_MAX - imx283_output_height(mode),
				 1, mode->default_VMAX - imx283_output_height(mode));
	__v4l2_ctrl_s_ctrl(imx283->vblank,
			   mode->default_VMAX - imx283_output_height(mode));

	/* Setting this will adjust the exposure limits as well. */

	__v4l2_ctrl_modify_range(imx283->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

	dev_info(imx283->dev,"Setting default HBLANK : %lld, VBLANK : %lld with PixelRate: %lld\n",def_hblank,mode->default_VMAX - imx283_output_height(mode), pixel_rate);

}
/* TODO */
static int imx283_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx283_mode *mode;
	struct imx283 *imx283 = to_imx283(sd);
	const struct imx283_mode *mode_list;
	unsigned int num_modes;

	mutex_lock(&imx283->mutex);

	/* Bayer order varies with flips */
	fmt->format.code = imx283_get_format_code(imx283,
							fmt->format.code);

	get_mode_table(fmt->format.code, &mode_list, &num_modes);

	mode = imx283_find_nearest_mode(mode_list, num_modes,
				       fmt->format.width,
				       fmt->format.height);
	imx283_update_image_pad_format(imx283, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state,
							fmt->pad);
		*framefmt = fmt->format;

		/*
		 * Keep the TRY crop in step with the TRY format, so a caller
		 * that probes a mode and then reads its crop back sees the
		 * pair that belongs together. The two deliberately differ in
		 * size: the static mode table retains the optical-black transport margins;
		 * the driver now emits an active-only frame, so the TRY crop is the
		 * effective sensor window actually delivered to V4L2.
		 *
		 * Only the TRY state is touched. This driver still uses the
		 * legacy subdev state model -- internal_ops.open seeds the
		 * per-file TRY state and v4l2_subdev_init_finalize() is never
		 * called -- so sd->active_state is NULL, and an ACTIVE S_FMT
		 * reaches this op with sd_state == NULL. Writing the crop
		 * unconditionally therefore dereferenced NULL and oopsed the
		 * kernel the moment libcamera configured a mode (found on a
		 * CM5 with kernel 6.12.93, Comm: cinepi-raw). The ACTIVE crop
		 * needs no store: imx283_get_selection() answers it straight
		 * from imx283->mode->crop, see __imx283_get_pad_crop().
		 */
		*v4l2_subdev_state_get_crop(sd_state, fmt->pad) = imx283_output_crop(mode);
	} else if (imx283->mode != mode) {
		imx283->mode = mode;
		imx283->fmt_code = fmt->format.code;
		imx283_set_framing_limits(imx283);
	}

	mutex_unlock(&imx283->mutex);

	return 0;
}
/* TODO */
static const struct v4l2_rect *
__imx283_get_pad_crop(struct imx283 *imx283,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE: {
		static struct v4l2_rect output_crop;
		output_crop = imx283_output_crop(imx283->mode);
		return &output_crop;
	}
	}

	return NULL;
}

static int imx283_standby_cancel(struct imx283 *imx283)
{
	int ret = 0;

	cci_write(imx283, IMX283_REG_STANDBY,
		  IMX283_STBLOGIC | IMX283_STBDV, &ret);

	/* Configure PLL clocks based on the xclk */
	cci_multi_reg_write(imx283, imx283->freq->regs,
			    imx283->freq->reg_count, &ret);

	dev_err(imx283->dev, "Using clk freq %d MHz", imx283->freq->mhz / MHZ(1));

	/* Initialise communication */
	cci_write(imx283, IMX283_REG_PLSTMG08, IMX283_PLSTMG08_VAL, &ret);
	cci_write(imx283, IMX283_REG_PLSTMG02, IMX283_PLSTMG02_VAL, &ret);

	/* Enable PLL */
	cci_write(imx283, IMX283_REG_STBPL, IMX283_STBPL_NORMAL, &ret);

	/* Configure the MIPI link speed */
	cci_multi_reg_write(imx283,
			    link_freq_reglist[imx283->link_freq_idx].regs,
			    link_freq_reglist[imx283->link_freq_idx].num_of_regs,
			    &ret);

	usleep_range(1000, 2000); /* 1st Stabilisation period of 1 ms or more */

	/* Activate */
	cci_write(imx283, IMX283_REG_STANDBY, IMX283_ACTIVE, &ret);
	usleep_range(19000, 20000); /* 2nd Stabilisation period of 19ms or more */

	cci_write(imx283, IMX283_REG_CLAMP, IMX283_CLPSQRST, &ret);
	cci_write(imx283, IMX283_REG_XMSTA, 0, &ret);
	cci_write(imx283, IMX283_REG_SYNCDRV, IMX283_SYNCDRV_XHS_XVS, &ret);

	return ret;
}

/* Start streaming */
static int imx283_start_streaming(struct imx283 *imx283)
{
	const struct imx283_readout_mode *readout;
	const struct imx283_mode *mode = imx283->mode;
	int ret;

	ret = imx283_standby_cancel(imx283);
	if (ret) {
		dev_err(imx283->dev, "failed to cancel standby\n");
		return ret;
	}

	/* Set the readout mode registers */
	readout = &imx283_readout_modes[imx283->mode->mode];
	cci_write(imx283, IMX283_REG_MDSEL1, readout->mdsel1, &ret);
	cci_write(imx283, IMX283_REG_MDSEL2, readout->mdsel2, &ret);
	cci_write(imx283, IMX283_REG_MDSEL3, readout->mdsel3, &ret);
	cci_write(imx283, IMX283_REG_MDSEL4, readout->mdsel4, &ret);

	/* Mode 1S specific entries from the Readout Drive Mode Tables */
	if (mode->mode == IMX283_MODE_1S) {
		cci_write(imx283, IMX283_REG_MDSEL7, 0x01, &ret);
		cci_write(imx283, IMX283_REG_MDSEL18, 0x1098, &ret);
	}

	if (ret) {
		dev_err(imx283->dev, "%s failed to set readout\n", __func__);
		return ret;
	}

	/* Initialise SVR. Unsupported for now - Always 0 */
	cci_write(imx283, IMX283_REG_SVR, 0x00, &ret);

	dev_err(imx283->dev, "Mode: Size %d x %d\n", mode->width, mode->height);

	dev_err(imx283->dev, "Analogue Crop (in the mode) %d,%d %dx%d\n",
		mode->crop.left,
		mode->crop.top,
		mode->crop.width,
		mode->crop.height);

	if (mode->veff && mode->vbin_ratio) {
		/*
		 * Apply the sensor VCROP mechanism to modes with validated
		 * vertical-crop geometry. Experimental modes without veff remain
		 * on the legacy path.
		 */
		cci_write(imx283, IMX283_REG_MDSEL3,
			  readout->mdsel3 | IMX283_MDSEL3_VCROP_EN, &ret);
		cci_write(imx283, IMX283_REG_MDSEL4,
			  readout->mdsel4 | IMX283_MDSEL4_VCROP_EN, &ret);
		{
			u32 y_out_size = mode->crop.height / mode->vbin_ratio;
			u32 write_v_size = y_out_size + mode->vertical_ob;
			u32 v_widcut = ((mode->veff - y_out_size) / 2) + mode->vct;
			s32 v_pos;

			if (imx283->vflip->val)
				v_pos = ((-(s32)mode->crop.top / mode->vbin_ratio) / 2) + mode->vst;
			else
				v_pos = ((s32)mode->crop.top / mode->vbin_ratio / 2) + mode->vst;

			cci_write(imx283, IMX283_REG_Y_OUT_SIZE, y_out_size, &ret);
			cci_write(imx283, IMX283_REG_WRITE_VSIZE, write_v_size, &ret);
			cci_write(imx283, IMX283_REG_VWIDCUT, v_widcut, &ret);
			cci_write(imx283, IMX283_REG_VWINPOS, v_pos, &ret);
		}
		cci_write(imx283, IMX283_REG_OB_SIZE_V, mode->vertical_ob, &ret);
	} else {
		/* Preserve the existing timing/crop programming for other modes. */
		cci_write(imx283, IMX283_REG_Y_OUT_SIZE,
			  mode->crop.height / mode->vbin_ratio, &ret);
		cci_write(imx283, IMX283_REG_WRITE_VSIZE,
			  mode->crop.height / mode->vbin_ratio + mode->vertical_ob, &ret);
		cci_write(imx283, IMX283_REG_OB_SIZE_V, mode->vertical_ob, &ret);
	}

	/*
	 * Configure horizontal cropping.
	 *
	 * WP-283-3: mainline writes HTRIMMING_END = crop.left + crop.width;
	 * use the exclusive end coordinate used by the upstream IMX283
	 * driver. The previous +1 extended the horizontal trimming window by
	 * one sensor column and could expose a spurious right-edge column.
	 */
	cci_write(imx283, IMX283_REG_HTRIMMING,
		  IMX283_HTRIMMING_EN | IMX283_HTRIMMING_RESERVED, &ret);
	{
		struct v4l2_rect output_crop = imx283_output_crop(mode);
		cci_write(imx283, IMX283_REG_HTRIMMING_START,
			  output_crop.left, &ret);
		cci_write(imx283, IMX283_REG_HTRIMMING_END,
			  output_crop.left + output_crop.width, &ret);
	}

	/* Todo: These must be calculated based on the link-freq and mode */
	cci_write(imx283, IMX283_REG_HMAX, mode->default_HMAX, &ret);
	cci_write(imx283, IMX283_REG_VMAX, mode->default_VMAX, &ret);
	cci_write(imx283, IMX283_REG_SHR, mode->min_SHR, &ret);

	/* Disable embedded data */
	cci_write(imx283, IMX283_REG_EBD_X_OUT_SIZE, 0, &ret);

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx283->sd.ctrl_handler);

	return ret;
}

/* Stop streaming */
static void imx283_stop_streaming(struct imx283 *imx283)
{
	int ret;

	ret = cci_write(imx283, IMX283_REG_STANDBY, IMX283_STBLOGIC, NULL);
	if (ret)
		dev_err(imx283->dev, "%s failed to set stream\n", __func__);
}

static int imx283_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx283 *imx283 = to_imx283(sd);
	int ret = 0;

	mutex_lock(&imx283->mutex);
	if (imx283->streaming == enable) {
		mutex_unlock(&imx283->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(imx283->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(imx283->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx283_start_streaming(imx283);
		if (ret)
			goto err_rpm_put;
	} else {
		imx283_stop_streaming(imx283);
		pm_runtime_put(imx283->dev);
	}

	imx283->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx283->vflip, enable);
	__v4l2_ctrl_grab(imx283->hflip, enable);

	mutex_unlock(&imx283->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(imx283->dev);
err_unlock:
	mutex_unlock(&imx283->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx283_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);
	int ret;

	ret = regulator_bulk_enable(imx283_NUM_SUPPLIES,
				    imx283->supplies);
	if (ret) {
		dev_err(imx283->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx283->xclk);
	if (ret) {
		dev_err(imx283->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx283->reset_gpio, 1);
	usleep_range(imx283_XCLR_MIN_DELAY_US,
		     imx283_XCLR_MIN_DELAY_US + imx283_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(imx283_NUM_SUPPLIES, imx283->supplies);
	return ret;
}

static int imx283_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);

	gpiod_set_value_cansleep(imx283->reset_gpio, 0);
	regulator_bulk_disable(imx283_NUM_SUPPLIES, imx283->supplies);
	clk_disable_unprepare(imx283->xclk);

	return 0;
}

static int __maybe_unused imx283_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);

	if (imx283->streaming)
		imx283_stop_streaming(imx283);

	return 0;
}

static int __maybe_unused imx283_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);
	int ret;

	if (imx283->streaming) {
		ret = imx283_start_streaming(imx283);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx283_stop_streaming(imx283);
	imx283->streaming = 0;
	return ret;
}

static int imx283_get_regulators(struct imx283 *imx283)
{
	unsigned int i;

	for (i = 0; i < imx283_NUM_SUPPLIES; i++)
		imx283->supplies[i].supply = imx283_supply_name[i];

	return devm_regulator_bulk_get(imx283->dev,
				       imx283_NUM_SUPPLIES,
				       imx283->supplies);
}

/* Verify chip ID */
static int imx283_identify_module(struct imx283 *imx283)
{
	int ret;
	u64 val;

	ret = cci_read(imx283, IMX283_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(imx283->dev, "failed to read chip id %x, with error %d\n",
			IMX283_CHIP_ID, ret);
		return ret;
	}

	if (val != IMX283_CHIP_ID) {
		dev_err(imx283->dev, "chip id mismatch: %x!=%llx\n",
			IMX283_CHIP_ID, val);
		return -EIO;
	}

	dev_info(imx283->dev, "Device found\n");

	return 0;
}

static int imx283_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx283 *imx283 = to_imx283(sd);

		mutex_lock(&imx283->mutex);
		sel->r = *__imx283_get_pad_crop(imx283, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx283->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = imx283_native_area;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT: {
		struct imx283 *imx283 = to_imx283(sd);
		sel->r = imx283_output_crop(imx283->mode);
		return 0;
	}
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = imx283_active_area;

		return 0;
	}

	return -EINVAL;
}


static const struct v4l2_subdev_core_ops imx283_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx283_video_ops = {
	.s_stream = imx283_set_stream,
};

static const struct v4l2_subdev_pad_ops imx283_pad_ops = {
	.enum_mbus_code = imx283_enum_mbus_code,
	.get_fmt = imx283_get_pad_format,
	.set_fmt = imx283_set_pad_format,
	.get_selection = imx283_get_selection,
	.enum_frame_size = imx283_enum_frame_size,
};

static const struct v4l2_subdev_ops imx283_subdev_ops = {
	.core = &imx283_core_ops,
	.video = &imx283_video_ops,
	.pad = &imx283_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx283_internal_ops = {
	.open = imx283_open,
};

/* Initialize control handlers */
static int imx283_init_controls(struct imx283 *imx283)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx283->sd);
	struct v4l2_fwnode_device_properties props;
	const struct imx283_mode *mode = imx283->mode;
	int ret;

	ctrl_hdlr = &imx283->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 21);
	if (ret)
		return ret;

	mutex_init(&imx283->mutex);
	ctrl_hdlr->lock = &imx283->mutex;


	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx283_set_framing_limits() call below.
	 */
	/* By default, PIXEL_RATE is read only */
	imx283->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0xffff,
					       0xffff, 1,
					       0xffff);

	imx283->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						   &imx283_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_frequencies) - 1,
						   0, link_frequencies);
	if (imx283->link_freq)
		imx283->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Read-only mode-geometry controls (WP-283-5). Flagged read-only
	 * after creation and written from inside the driver with
	 * __v4l2_ctrl_s_ctrl, which bypasses the read-only refusal that
	 * only applies to the user-space ioctl path -- the same shape the
	 * imx585 driver uses for its own "Mode Binning" / "Mode Crop *"
	 * controls. Actual values are set below by imx283_set_framing_limits()
	 * via imx283_update_mode_metadata(), which also runs on every later
	 * mode change.
	 */
	imx283->mode_binning_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							 &imx283_cfg_mode_binning,
							 NULL);
	imx283->mode_crop_left_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							   &imx283_cfg_mode_crop_left,
							   NULL);
	imx283->mode_crop_top_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							  &imx283_cfg_mode_crop_top,
							  NULL);
	imx283->mode_crop_width_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							    &imx283_cfg_mode_crop_width,
							    NULL);
	imx283->mode_crop_height_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							     &imx283_cfg_mode_crop_height,
							     NULL);
	if (imx283->mode_binning_ctrl)
		imx283->mode_binning_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (imx283->mode_crop_left_ctrl)
		imx283->mode_crop_left_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (imx283->mode_crop_top_ctrl)
		imx283->mode_crop_top_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (imx283->mode_crop_width_ctrl)
		imx283->mode_crop_width_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	imx283->mode_active_left_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							     &imx283_cfg_mode_active_left,
							     NULL);
	imx283->mode_active_top_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							    &imx283_cfg_mode_active_top,
							    NULL);
	if (imx283->mode_crop_height_ctrl)
		imx283->mode_crop_height_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (imx283->mode_active_left_ctrl)
		imx283->mode_active_left_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (imx283->mode_active_top_ctrl)
		imx283->mode_active_top_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Initial vblank/hblank/exposure based on the current mode. */
	imx283->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					   V4L2_CID_VBLANK,
					   imx283_min_vmax(mode) - imx283_output_height(mode),
					   IMX283_VMAX_MAX, 1,
					   mode->default_VMAX - imx283_output_height(mode));

	imx283->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx283->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX283_EXPOSURE_MIN,
					     IMX283_EXPOSURE_MAX,
					     IMX283_EXPOSURE_STEP,
					     IMX283_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX283_ANA_GAIN_MIN, IMX283_ANA_GAIN_MAX,
			  IMX283_ANA_GAIN_STEP, IMX283_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX283_DGTL_GAIN_MIN, IMX283_DGTL_GAIN_MAX,
			  IMX283_DGTL_GAIN_STEP, IMX283_DGTL_GAIN_DEFAULT);

	imx283->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx283->hflip)
		imx283->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx283->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx283_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx283->vflip)
		imx283->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx283_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx283_tpg_menu) - 1,
				     0, 0, imx283_tpg_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx283_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx283->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx283_set_framing_limits(imx283);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx283->mutex);

	return ret;
}

static void imx283_free_controls(struct imx283 *imx283)
{
	v4l2_ctrl_handler_free(imx283->sd.ctrl_handler);
	mutex_destroy(&imx283->mutex);
}

static const struct of_device_id imx283_dt_ids[] = {
	{ .compatible = "sony,imx283", },
	{ /* sentinel */ }
};

static int imx283_parse_endpoint(struct imx283 *imx283)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx283->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	int ret;
	int i, j;

	if (!fwnode)
		return -ENXIO;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep) {
		dev_err(imx283->dev, "Failed to get next endpoint\n");
		return -ENXIO;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(imx283->dev,
			"number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(imx283->dev, "no link frequencies defined\n");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		for (j = 0; j < ARRAY_SIZE(link_frequencies); j++) {
			if (bus_cfg.link_frequencies[i] == link_frequencies[j]) {
				imx283->link_freq_idx = j;
				break;
			}
		}

		if (j == ARRAY_SIZE(link_frequencies)) {
			ret = dev_err_probe(imx283->dev, -EINVAL,
					    "no supported link freq found\n");
			goto done_endpoint_free;
		}
	}

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
};

static int imx283_probe(struct i2c_client *client)
{
	struct imx283 *imx283;
	int ret;
	unsigned int i;
	unsigned int xclk_freq;

	imx283 = devm_kzalloc(&client->dev, sizeof(*imx283), GFP_KERNEL);
	if (!imx283)
		return -ENOMEM;

	imx283->dev = &client->dev;

	struct device *dev = &client->dev;

	/* Static-data self-check; warns only, see imx283_check_mode_table(). */
	imx283_check_mode_table(dev, "supported_modes_12bit",
				supported_modes_12bit,
				ARRAY_SIZE(supported_modes_12bit));
	imx283_check_mode_table(dev, "supported_modes_10bit",
				supported_modes_10bit,
				ARRAY_SIZE(supported_modes_10bit));

	v4l2_i2c_subdev_init(&imx283->sd, client, &imx283_subdev_ops);

	/*
	imx283 = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx283)) {
		ret = PTR_ERR(imx283);
		dev_err(imx283->dev, "failed to initialize CCI: %d\n", ret);
		return ret;
	}
	*/

	/* Get system clock (xclk) */
	imx283->xclk = devm_clk_get(imx283->dev, NULL);
	if (IS_ERR(imx283->xclk)) {
		dev_err(imx283->dev, "failed to get xclk\n");
		return PTR_ERR(imx283->xclk);
	}

	xclk_freq = clk_get_rate(imx283->xclk);
	for (i = 0; i < ARRAY_SIZE(imx283_frequencies); i++) {
		if (xclk_freq == imx283_frequencies[i].mhz) {
			imx283->freq = &imx283_frequencies[i];
			break;
		}
	}
	if (!imx283->freq) {
		dev_err(imx283->dev, "xclk frequency unsupported: %d Hz\n", xclk_freq);
		return -EINVAL;
	}

	ret = imx283_get_regulators(imx283);
	if (ret) {
		dev_err(imx283->dev, "failed to get regulators\n");
		return ret;
	}

	ret = imx283_parse_endpoint(imx283);
	if (ret) {
		dev_err(imx283->dev, "failed to parse endpoint configuration\n");
		return ret;
	}

	/* Request optional enable pin */
	imx283->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx283_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx283_power_on(dev);
	if (ret)
		return ret;

	ret = imx283_identify_module(imx283);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx283_set_default_format(imx283);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx283_init_controls(imx283);
	if (ret)
		goto error_pm;

	/* Initialize subdev */
	imx283->sd.internal_ops = &imx283_internal_ops;
	imx283->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx283->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx283->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx283->sd.entity, 1, &imx283->pad);
	if (ret) {
		dev_err(imx283->dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx283->sd);
	if (ret < 0) {
		dev_err(imx283->dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx283->sd.entity);

error_handler_free:
	imx283_free_controls(imx283);

error_pm:
	pm_runtime_disable(imx283->dev);
	pm_runtime_set_suspended(imx283->dev);
error_power_off:
	imx283_power_off(imx283->dev);

	return ret;
}

static void imx283_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx283 *imx283 = to_imx283(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx283_free_controls(imx283);

	pm_runtime_disable(imx283->dev);
	if (!pm_runtime_status_suspended(imx283->dev))
		imx283_power_off(imx283->dev);
	pm_runtime_set_suspended(imx283->dev);

}

MODULE_DEVICE_TABLE(of, imx283_dt_ids);

static const struct dev_pm_ops imx283_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx283_suspend, imx283_resume)
	SET_RUNTIME_PM_OPS(imx283_power_off, imx283_power_on, NULL)
};

static struct i2c_driver imx283_i2c_driver = {
	.driver = {
		.name = "imx283",
		.of_match_table	= imx283_dt_ids,
		.pm = &imx283_pm_ops,
	},
	.probe = imx283_probe,
	.remove = imx283_remove,
};

module_i2c_driver(imx283_i2c_driver);

MODULE_DESCRIPTION("Sony IMX283 Sensor Driver");
MODULE_LICENSE("GPL v2");