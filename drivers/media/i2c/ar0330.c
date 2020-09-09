/*
 * Driver for AR0330 CMOS Image Sensor from Aptina
 *
 * Copyright (C) 2011, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (C) 2011, Javier Martin <javier.martin@vista-silicon.com>
 * Copyright (C) 2011, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * Based on the MT9V032 driver and Bastian Hecht's code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "smiapp-pll.h"

#define AR0330_PIXEL_ARRAY_WIDTH		2304
#define AR0330_PIXEL_ARRAY_HEIGHT		1536

#define AR0330_WINDOW_WIDTH_MIN			32
#define AR0330_WINDOW_WIDTH_DEF			2304
#define AR0330_WINDOW_WIDTH_MAX			AR0330_PIXEL_ARRAY_WIDTH
#define AR0330_WINDOW_HEIGHT_MIN		32
#define AR0330_WINDOW_HEIGHT_DEF		1296
#define AR0330_WINDOW_HEIGHT_MAX		AR0330_PIXEL_ARRAY_HEIGHT

#define AR0330_CHIP_VERSION				0x3000
#define		AR0330_CHIP_VERSION_VALUE		0x2604
#define AR0330_Y_ADDR_START				0x3002
#define AR0330_X_ADDR_START				0x3004
#define AR0330_Y_ADDR_END				0x3006
#define AR0330_X_ADDR_END				0x3008
#define AR0330_FRAME_LENGTH_LINES			0x300a
#define AR0330_LINE_LENGTH_PCK				0x300c
#define AR0330_CHIP_REVISION				0x300e
#define		AR0330_CHIP_REVISION_1x			0x10
#define		AR0330_CHIP_REVISION_2x			0x20
#define AR0330_LOCK_CONTROL				0x3010
#define		AR0330_LOCK_CONTROL_UNLOCK		0xbeef
#define AR0330_COARSE_INTEGRATION_TIME			0x3012
#define		AR0330_COARSE_INTEGRATION_TIME_MIN	1
#define		AR0330_COARSE_INTEGRATION_TIME_DEF	16
#define		AR0330_COARSE_INTEGRATION_TIME_MAX	65535
#define AR0330_RESET					0x301a
#define		AR0330_RESET_SMIA_DIS			(1 << 12)
#define		AR0330_RESET_FORCE_PLL_ON		(1 << 11)
#define		AR0330_RESET_RESTART_BAD		(1 << 10)
#define		AR0330_RESET_MASK_BAD			(1 << 9)
#define		AR0330_RESET_GPI_EN			(1 << 8)
#define		AR0330_RESET_PARALLEL_EN		(1 << 7)
#define		AR0330_RESET_DRIVE_PINS			(1 << 6)
#define		AR0330_RESET_LOCK_REG			(1 << 3)
#define		AR0330_RESET_STREAM			(1 << 2)
#define		AR0330_RESET_RESTART			(1 << 1)
#define		AR0330_RESET_RESET			(1 << 0)
/* AR03303_MODE_SELECT is an alias for AR0330_RESET_STREAM */
#define AR0330_MODE_SELECT				0x301c
#define		AR0330_MODE_SELECT_STREAM		(1 << 0)
#define AR0330_VT_PIX_CLK_DIV				0x302a
#define AR0330_VT_SYS_CLK_DIV				0x302c
#define AR0330_PRE_PLL_CLK_DIV				0x302e
#define AR0330_PLL_MULTIPLIER				0x3030
#define AR0330_OP_PIX_CLK_DIV				0x3036
#define AR0330_OP_SYS_CLK_DIV				0x3038
#define AR0330_FRAME_COUNT				0x303a
#define AR0330_READ_MODE				0x3040
#define		AR0330_READ_MODE_VERT_FLIP		(1 << 15)
#define		AR0330_READ_MODE_HORIZ_MIRROR		(1 << 14)
#define		AR0330_READ_MODE_COL_BIN		(1 << 13)
#define		AR0330_READ_MODE_ROW_BIN		(1 << 12)
#define		AR0330_READ_MODE_COL_SF_BIN		(1 << 9)
#define		AR0330_READ_MODE_COL_SUM		(1 << 5)
#define AR0330_GREEN1_GAIN				0x3056
#define AR0330_BLUE_GAIN				0x3058
#define AR0330_RED_GAIN					0x305a
#define AR0330_GREEN2_GAIN				0x305c
#define AR0330_GLOBAL_GAIN				0x305e
#define		AR0330_GLOBAL_GAIN_MIN			128
#define		AR0330_GLOBAL_GAIN_DEF			128
#define		AR0330_GLOBAL_GAIN_MAX			2047
#define AR0330_ANALOG_GAIN				0x3060
#define AR0330_DATAPATH_SELECT				0x306e
#define		AR0330_DATAPATH_SLEW_DOUT_MASK		(7 << 13)
#define		AR0330_DATAPATH_SLEW_DOUT_SHIFT		13
#define		AR0330_DATAPATH_SLEW_PCLK_MASK		(7 << 10)
#define		AR0330_DATAPATH_SLEW_PCLK_SHIFT		10
#define		AR0330_DATAPATH_HIGH_VCM		(1 << 9)
#define AR0330_TEST_PATTERN_MODE			0x3070
#define AR0330_TEST_DATA_RED				0x3072
#define AR0330_TEST_DATA_GREENR				0x3074
#define AR0330_TEST_DATA_BLUE				0x3076
#define AR0330_TEST_DATA_GREENB				0x3078
#define AR0330_SEQ_DATA_PORT				0x3086
#define AR0330_SEQ_CTRL_PORT				0x3088
#define AR0330_X_ODD_INC				0x30a2
#define AR0330_Y_ODD_INC				0x30a6
#define AR0330_X_ODD_INC				0x30a2
#define AR0330_DATA_FORMAT_BITS				0x31ac
#define AR0330_SERIAL_FORMAT				0x31ae
#define AR0330_FRAME_PREAMBLE				0x31b0
#define AR0330_LINE_PREAMBLE				0x31b2
#define AR0330_MIPI_TIMING_0				0x31b4
#define AR0330_MIPI_TIMING_1				0x31b6
#define AR0330_MIPI_TIMING_2				0x31b8
#define AR0330_MIPI_TIMING_3				0x31ba
#define AR0330_MIPI_TIMING_4				0x31bc
#define AR0330_MIPI_CONFIG_STATUS			0x31be
#define AR0330_COMPRESSION				0x31d0
#define		AR0330_COMPRESSION_ENABLE		(1 << 0)
#define AR0330_POLY_SC					0x3780
#define		AR0330_POLY_SC_ENABLE			(1 << 15)

struct ar0330 {
	struct i2c_client *client;
	struct device *dev;

	struct smiapp_pll pll;
	unsigned int version;

	struct clk *clock;
	struct gpio_desc *reset;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_rect crop;  /* Sensor window */
	struct v4l2_mbus_framefmt format;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *flip[2];
	struct v4l2_ctrl *pixel_rate;

	struct mutex power_lock; /* lock to protect power_count */
	int power_count;

	/* Registers cache */
	u16 read_mode;
};

static struct ar0330 *to_ar0330(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0330, subdev);
}

/* -----------------------------------------------------------------------------
 * Register access
 */

static int __ar0330_read(struct ar0330 *ar0330, u16 reg, size_t size)
{
	struct i2c_client *client = ar0330->client;
	u8 data[2];
	struct i2c_msg msg[2] = {
		{ client->addr, 0, 2, data },
		{ client->addr, I2C_M_RD, size, data },
	};
	int ret;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(ar0330->dev, "%s(0x%04x) failed (%d)\n", __func__,
			reg, ret);
		return ret;
	}

	if (size == 2)
		return (data[0] << 8) | data[1];
	else
		return data[0];
}

static int __ar0330_write(struct ar0330 *ar0330, u16 reg, u16 value,
			  size_t size)
{
	struct i2c_client *client = ar0330->client;
	u8 data[4];
	struct i2c_msg msg = { client->addr, 0, 2 + size, data };
	int ret;

	dev_dbg(ar0330->dev, "writing 0x%04x to 0x%04x\n", value, reg);

	data[0] = reg >> 8;
	data[1] = reg & 0xff;
	if (size == 2) {
		data[2] = value >> 8;
		data[3] = value & 0xff;
	} else {
		data[2] = value & 0xff;
	}

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(ar0330->dev, "%s(0x%04x) failed (%d)\n", __func__,
			reg, ret);
		return ret;
	}

	return 0;
}

static inline int ar0330_read8(struct ar0330 *ar0330, u16 reg)
{
	return __ar0330_read(ar0330, reg, 1);
}

static inline int ar0330_write8(struct ar0330 *ar0330, u16 reg, u8 value)
{
	return __ar0330_write(ar0330, reg, value, 1);
}

static inline int ar0330_read16(struct ar0330 *ar0330, u16 reg)
{
	return __ar0330_read(ar0330, reg, 2);
}

static inline int ar0330_write16(struct ar0330 *ar0330, u16 reg, u16 value)
{
	return __ar0330_write(ar0330, reg, value, 2);
}

static int ar0330_set_read_mode(struct ar0330 *ar0330, u16 clear, u16 set)
{
	u16 value = (ar0330->read_mode & ~clear) | set;
	int ret;

	ret = ar0330_write16(ar0330, AR0330_READ_MODE, value);
	if (ret < 0)
		return ret;

	ar0330->read_mode = value;
	return 0;
}

static int ar0330_pll_configure(struct ar0330 *ar0330)
{
	int ret;

	ret = ar0330_write16(ar0330, AR0330_VT_PIX_CLK_DIV,
			     ar0330->pll.vt_pix_clk_div);
	if (ret < 0)
		return ret;

	ret = ar0330_write16(ar0330, AR0330_VT_SYS_CLK_DIV,
			     ar0330->pll.vt_sys_clk_div);
	if (ret < 0)
		return ret;

	ret = ar0330_write16(ar0330, AR0330_PRE_PLL_CLK_DIV,
			     ar0330->pll.pre_pll_clk_div);
	if (ret < 0)
		return ret;

	ret = ar0330_write16(ar0330, AR0330_PLL_MULTIPLIER,
			     ar0330->pll.pll_multiplier);
	if (ret < 0)
		return ret;

	ret = ar0330_write16(ar0330, AR0330_OP_PIX_CLK_DIV,
			     ar0330->pll.op_pix_clk_div);
	if (ret < 0)
		return ret;

	ret = ar0330_write16(ar0330, AR0330_OP_SYS_CLK_DIV,
			     ar0330->pll.op_sys_clk_div);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	return 0;
}

static int ar0330_pll_init(struct ar0330 *ar0330)
{
	struct smiapp_pll_limits limits;
	unsigned long rate = 24000000;
	int ret;

	rate = clk_round_rate(ar0330->clock, rate);
	if (clk_set_rate(ar0330->clock, rate))
		return -EINVAL;

	dev_dbg(ar0330->dev, "clock rate set to %lu\n", rate);

	limits.min_ext_clk_freq_hz = 6000000;
	limits.max_ext_clk_freq_hz = 64000000;
	/*
	 * HACK: The real minimum pre_pll_clk_div value is 1. The SMIA++ PLL
	 * computation code currently hardcodes pre_pll_clk_div to its minimum
	 * value, so set it to 8 as that's what the AR0330 requires with the
	 * panda board.
	 */
	limits.min_pre_pll_clk_div = 8;
	limits.max_pre_pll_clk_div = 64;
	limits.min_pll_ip_freq_hz = 1000000;
	limits.max_pll_ip_freq_hz = 24000000;
	limits.min_pll_multiplier = 32;
	limits.max_pll_multiplier = 384;
	limits.min_pll_op_freq_hz = 384000000;
	limits.max_pll_op_freq_hz = 768000000;

	limits.min_vt_sys_clk_div = 1;
	limits.max_vt_sys_clk_div = 16;
	limits.min_vt_sys_clk_freq_hz = 24000000;
	limits.max_vt_sys_clk_freq_hz = 768000000;
	limits.min_vt_pix_clk_div = 4;
	limits.max_vt_pix_clk_div = 16;
	limits.min_vt_pix_clk_freq_hz = 1;
	limits.max_vt_pix_clk_freq_hz = 98000000;

	limits.min_op_sys_clk_div = 1;
	limits.max_op_sys_clk_div = 1;
	limits.min_op_sys_clk_freq_hz = 384000000;
	limits.max_op_sys_clk_freq_hz = 768000000;
	limits.min_op_pix_clk_div = 8;
	limits.max_op_pix_clk_div = 12;
	limits.min_op_pix_clk_freq_hz = 1;
	limits.max_op_pix_clk_freq_hz = 98000000;

	ar0330->pll.lanes = 2;
	ar0330->pll.binning_horizontal = 1;
	ar0330->pll.binning_vertical = 1;
	ar0330->pll.scale_m = 16;
	ar0330->pll.scale_n = 16;
	ar0330->pll.bits_per_pixel = 12;
	ar0330->pll.flags = SMIAPP_PLL_FLAG_OP_PIX_CLOCK_PER_LANE
			  | SMIAPP_PLL_FLAG_DUAL_READOUT;
	ar0330->pll.ext_clk_freq_hz = rate;
	ar0330->pll.link_freq = 294000000;

	ret = smiapp_pll_calculate(ar0330->dev, &limits, &ar0330->pll);
	if (ret < 0)
		return ret;

	dev_dbg(ar0330->dev, "ext_clk_freq_hz %u\n", ar0330->pll.ext_clk_freq_hz);
	dev_dbg(ar0330->dev, "pre_pll_clk_div %u\n", ar0330->pll.pre_pll_clk_div);
	dev_dbg(ar0330->dev, "pll_multiplier %u\n", ar0330->pll.pll_multiplier);
	dev_dbg(ar0330->dev, "vt_pix_clk_div %u\n", ar0330->pll.vt_pix_clk_div);
	dev_dbg(ar0330->dev, "vt_sys_clk_div %u\n", ar0330->pll.vt_sys_clk_div);
	dev_dbg(ar0330->dev, "vt_pix_clk_freq_hz %u\n", ar0330->pll.vt_pix_clk_freq_hz);
	dev_dbg(ar0330->dev, "op_pix_clk_div %u\n", ar0330->pll.op_pix_clk_div);
	dev_dbg(ar0330->dev, "op_sys_clk_div %u\n", ar0330->pll.op_sys_clk_div);

	/*
	 * The sensor has dual pixel readout paths, the pixel rate is equal to
	 * twice the VT pixel clock frequency.
	 */
	ar0330->pixel_rate->cur.val64 = ar0330->pll.vt_pix_clk_freq_hz * 2;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Power handling
 */

struct ar0330_register {
	u16 addr;
	u16 value;
	bool wide;
}
;
struct ar0330_patch {
	const struct ar0330_register *reg_data;
	unsigned int reg_size;

	u16 seq_addr;
	const u16 *seq_data;
	unsigned int seq_size;
};

static const u16 ar0330_sequencer_v1[] = {
	0x4540, 0x6134, 0x4a31, 0x4342, 0x4560, 0x2714, 0x3dff, 0x3dff,
	0x3dea, 0x2704, 0x3d10, 0x2705, 0x3d10, 0x2715, 0x3527, 0x053d,
	0x1045, 0x4027, 0x4027, 0x143d, 0xff3d, 0xff3d, 0xea62, 0x2728,
	0x3627, 0x083d, 0x6444, 0x2c2c, 0x2c2c, 0x4b01, 0x432d, 0x4643,
	0x1647, 0x435f, 0x4f50, 0x2604, 0x2684, 0x2027, 0xfc53, 0x0d5c,
	0x0d60, 0x5754, 0x1709, 0x5556, 0x4917, 0x145c, 0x0945, 0x0045,
	0x8026, 0xa627, 0xf917, 0x0227, 0xfa5c, 0x0b5f, 0x5307, 0x5302,
	0x4d28, 0x6c4c, 0x0928, 0x2c28, 0x294e, 0x1718, 0x26a2, 0x5c03,
	0x1744, 0x2809, 0x27f2, 0x1714, 0x2808, 0x164d, 0x1a26, 0x8317,
	0x0145, 0xa017, 0x0727, 0xf317, 0x2945, 0x8017, 0x0827, 0xf217,
	0x285d, 0x27fa, 0x170e, 0x2681, 0x5300, 0x17e6, 0x5302, 0x1710,
	0x2683, 0x2682, 0x4827, 0xf24d, 0x4e28, 0x094c, 0x0b17, 0x6d28,
	0x0817, 0x014d, 0x1a17, 0x0126, 0x035c, 0x0045, 0x4027, 0x9017,
	0x2a4a, 0x0a43, 0x160b, 0x4327, 0x9445, 0x6017, 0x0727, 0x9517,
	0x2545, 0x4017, 0x0827, 0x905d, 0x2808, 0x530d, 0x2645, 0x5c01,
	0x2798, 0x4b12, 0x4452, 0x5117, 0x0260, 0x184a, 0x0343, 0x1604,
	0x4316, 0x5843, 0x1659, 0x4316, 0x5a43, 0x165b, 0x4327, 0x9c45,
	0x6017, 0x0727, 0x9d17, 0x2545, 0x4017, 0x1027, 0x9817, 0x2022,
	0x4b12, 0x442c, 0x2c2c, 0x2c00,
};

static const struct ar0330_register ar0330_register_v1[] = {
	{ 0x30ba, 0x2c, 0 },
	{ 0x30fe, 0x0080, 1 },
	{ 0x31e0, 0x0000, 1 },
	{ 0x3ece, 0xff, 0 },
	{ 0x3ed0, 0xe4f6, 1 },
	{ 0x3ed2, 0x0106, 1 },
	{ 0x3ed4, 0x8f6c, 1 },
	{ 0x3ed6, 0x6666, 1 },
	{ 0x3ed8, 0x6642, 1 },
	{ 0x3eda, 0x8822, 1 },
	{ 0x3edc, 0x2222, 1 },
	{ 0x3ede, 0x22c0, 1 },
	{ 0x3ee0, 0x1500, 1 },
	{ 0x3ee6, 0x0080, 1 },
	{ 0x3ee8, 0x2027, 1 },
	{ 0x3eea, 0x001d, 1 },
	{ 0x3f06, 0x046a, 1 },
};

static const u16 ar0330_sequencer_v2[] = {
	0x4a03, 0x4316, 0x0443, 0x1645, 0x4045, 0x6017, 0x2045, 0x404b,
	0x1244, 0x6134, 0x4a31, 0x4342, 0x4560, 0x2714, 0x3dff, 0x3dff,
	0x3dea, 0x2704, 0x3d10, 0x2705, 0x3d10, 0x2715, 0x3527, 0x053d,
	0x1045, 0x4027, 0x0427, 0x143d, 0xff3d, 0xff3d, 0xea62, 0x2728,
	0x3627, 0x083d, 0x6444, 0x2c2c, 0x2c2c, 0x4b01, 0x432d, 0x4643,
	0x1647, 0x435f, 0x4f50, 0x2604, 0x2684, 0x2027, 0xfc53, 0x0d5c,
	0x0d57, 0x5417, 0x0955, 0x5649, 0x5307, 0x5302, 0x4d28, 0x6c4c,
	0x0928, 0x2c28, 0x294e, 0x5c09, 0x6045, 0x0045, 0x8026, 0xa627,
	0xf817, 0x0227, 0xfa5c, 0x0b17, 0x1826, 0xa25c, 0x0317, 0x4427,
	0xf25f, 0x2809, 0x1714, 0x2808, 0x1701, 0x4d1a, 0x2683, 0x1701,
	0x27fa, 0x45a0, 0x1707, 0x27fb, 0x1729, 0x4580, 0x1708, 0x27fa,
	0x1728, 0x5d17, 0x0e26, 0x8153, 0x0117, 0xe653, 0x0217, 0x1026,
	0x8326, 0x8248, 0x4d4e, 0x2809, 0x4c0b, 0x6017, 0x2027, 0xf217,
	0x535f, 0x2808, 0x164d, 0x1a17, 0x0127, 0xfa26, 0x035c, 0x0145,
	0x4027, 0x9817, 0x2a4a, 0x0a43, 0x160b, 0x4327, 0x9c45, 0x6017,
	0x0727, 0x9d17, 0x2545, 0x4017, 0x0827, 0x985d, 0x2645, 0x4b17,
	0x0a28, 0x0853, 0x0d52, 0x5112, 0x4460, 0x184a, 0x0343, 0x1604,
	0x4316, 0x5843, 0x1659, 0x4316, 0x5a43, 0x165b, 0x4327, 0x9c45,
	0x6017, 0x0727, 0x9d17, 0x2545, 0x4017, 0x1027, 0x9817, 0x2022,
	0x4b12, 0x442c, 0x2c2c, 0x2c00,
};

static const struct ar0330_register ar0330_register_v2[] = {
	{ 0x30ba, 0x2c, 0 },
	{ 0x30fe, 0x0080, 1 },
	{ 0x31e0, 0x0000, 1 },
	{ 0x3ece, 0xff, 0 },
	{ 0x3ed0, 0xe4f6, 1 },
	{ 0x3ed2, 0x0146, 1 },
	{ 0x3ed4, 0x8f6c, 1 },
	{ 0x3ed6, 0x6666, 1 },
	{ 0x3ed8, 0x8642, 1 },
	{ 0x3eda, 0x889b, 1 },
	{ 0x3edc, 0x8863, 1 },
	{ 0x3ede, 0xaa04, 1 },
	{ 0x3ee0, 0x15f0, 1 },
	{ 0x3ee6, 0x008c, 1 },
	{ 0x3ee8, 0x2024, 1 },
	{ 0x3eea, 0xff1f, 1 },
	{ 0x3f06, 0x046a, 1 },
};

static const u16 ar0330_sequencer_v3[] = {
	0x2045,
};

static const struct ar0330_register ar0330_register_v3[] = {
	{ 0x3ed4, 0x8f6c, 1 },
	{ 0x3ed6, 0x6666, 1 }
};

struct ar0330_patch ar0330_patches[] = {
	[0] = {
		.reg_data = ar0330_register_v1,
		.reg_size = ARRAY_SIZE(ar0330_register_v1),
		.seq_addr = 0x8000,
		.seq_data = ar0330_sequencer_v1,
		.seq_size = ARRAY_SIZE(ar0330_sequencer_v1),
	},
	[1] = {
		.reg_data = ar0330_register_v2,
		.reg_size = ARRAY_SIZE(ar0330_register_v2),
		.seq_addr = 0x8000,
		.seq_data = ar0330_sequencer_v2,
		.seq_size = ARRAY_SIZE(ar0330_sequencer_v2),
	},
	[2] = {
		.reg_data = ar0330_register_v3,
		.reg_size = ARRAY_SIZE(ar0330_register_v3),
		.seq_addr = 0x800c,
		.seq_data = ar0330_sequencer_v3,
		.seq_size = ARRAY_SIZE(ar0330_sequencer_v3),
	},
	[3] = {
		.reg_size = 0,
		.seq_size = 0,
	},
};

static int ar0330_otpm_patch(struct ar0330 *ar0330)
{
	const struct ar0330_patch *patch;
	unsigned int i;
	int ret;

	if (ar0330->version == 0)
		return 0;

	patch = &ar0330_patches[ar0330->version - 1];

	for (i = 0; i < patch->reg_size; ++i) {
		if (!patch->reg_data[i].wide)
			ret = ar0330_write8(ar0330, patch->reg_data[i].addr,
					    patch->reg_data[i].value);
		else
			ret = ar0330_write16(ar0330, patch->reg_data[i].addr,
					     patch->reg_data[i].value);

		if (ret < 0)
			return ret;
	}

	ret = ar0330_write16(ar0330, AR0330_SEQ_CTRL_PORT, patch->seq_addr);
	if (ret < 0)
		return ret;

	for (i = 0; i < patch->seq_size; ++i) {
		ret = ar0330_write16(ar0330, AR0330_SEQ_DATA_PORT,
				     patch->seq_data[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ar0330_power_on(struct ar0330 *ar0330)
{
	/* Enable clock */
	clk_enable(ar0330->clock);

	/* Assert reset for 1ms */
	if (ar0330->reset) {
		gpiod_set_value(ar0330->reset, 1);
		usleep_range(1000, 2000);
		gpiod_set_value(ar0330->reset, 0);
		usleep_range(10000, 11000);
	}

	return 0;
}

static void ar0330_power_off(struct ar0330 *ar0330)
{
	/* Disable clock */
	clk_disable(ar0330->clock);
}

static int __ar0330_set_power(struct ar0330 *ar0330, bool on)
{
	int ret;

	if (!on) {
		ar0330_power_off(ar0330);
		return 0;
	}

	ret = ar0330_power_on(ar0330);
	if (ret < 0)
		return ret;

	ret = ar0330_otpm_patch(ar0330);
	if (ret < 0)
		return ret;

	return v4l2_ctrl_handler_setup(&ar0330->ctrls);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

static int ar0330_set_params(struct ar0330 *ar0330)
{
	struct v4l2_mbus_framefmt *format = &ar0330->format;
	const struct v4l2_rect *crop = &ar0330->crop;
	unsigned int xskip;
	unsigned int yskip;
	int ret;

	/* Serial interface. */
	ret = ar0330_write16(ar0330, AR0330_RESET, 0);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_DATA_FORMAT_BITS, 0x0c0c);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_SERIAL_FORMAT, 0x0202);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_FRAME_PREAMBLE, 36);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_LINE_PREAMBLE, 12);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_MIPI_TIMING_0, 0x2643);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_MIPI_TIMING_1, 0x114e);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_MIPI_TIMING_2, 0x2048);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_MIPI_TIMING_3, 0x0186);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_MIPI_TIMING_4, 0x8005);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_MIPI_CONFIG_STATUS, 0x2003);
	if (ret < 0)
		return ret;

	/*
	 * Windows position and size.
	 *
	 * TODO: Make sure the start coordinates and window size match the
	 * skipping and mirroring requirements.
	 */
	ret = ar0330_write16(ar0330, AR0330_X_ADDR_START, crop->left);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_Y_ADDR_START, crop->top);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_X_ADDR_END,
			     crop->left + crop->width);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_Y_ADDR_END,
			     crop->top + crop->height);
	if (ret < 0)
		return ret;

	/* Row and column binning. */
	xskip = DIV_ROUND_CLOSEST(crop->width, format->width) * 2 - 1;
	yskip = DIV_ROUND_CLOSEST(crop->height, format->height) * 2 - 1;

	ret = ar0330_write16(ar0330, AR0330_X_ODD_INC, xskip);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_Y_ODD_INC, yskip);
	if (ret < 0)
		return ret;

	/* Blanking - use default values. */
	ret = ar0330_write16(ar0330, AR0330_LINE_LENGTH_PCK,
			     (crop->width + 192) / 2);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(ar0330, AR0330_FRAME_LENGTH_LINES,
			     crop->height + 60);
	if (ret < 0)
		return ret;

	return ret;
}

static int ar0330_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);
	int ret;

	dev_dbg(ar0330->dev, "%s: frame count is %d\n", __func__,
		ar0330_read16(ar0330, AR0330_FRAME_COUNT));

	if (!enable)
		return ar0330_write8(ar0330, AR0330_MODE_SELECT, 0);

	ret = ar0330_pll_configure(ar0330);
	if (ret < 0)
		return ret;

	ret = ar0330_set_params(ar0330);
	if (ret < 0)
		return ret;

	return ar0330_write8(ar0330, AR0330_MODE_SELECT,
			     AR0330_MODE_SELECT_STREAM);
}

static int ar0330_enum_mbus_code(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);

	if (code->pad || code->index)
		return -EINVAL;

	code->code = ar0330->format.code;
	return 0;
}

static int ar0330_enum_frame_size(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);

	if (fse->index >= 3 || fse->code != ar0330->format.code)
		return -EINVAL;

	fse->min_width = AR0330_WINDOW_WIDTH_DEF / (fse->index + 1);
	fse->max_width = fse->min_width;
	fse->min_height = AR0330_WINDOW_HEIGHT_DEF / (fse->index + 1);
	fse->max_height = fse->min_height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ar0330_get_pad_format(struct ar0330 *ar0330, struct v4l2_subdev_fh *fh,
			 unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0330->format;
	default:
		return NULL;
	}
}

static struct v4l2_rect *
__ar0330_get_pad_crop(struct ar0330 *ar0330, struct v4l2_subdev_fh *fh,
		     unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0330->crop;
	default:
		return NULL;
	}
}

static int ar0330_get_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *fmt)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);

	fmt->format = *__ar0330_get_pad_format(ar0330, fh, fmt->pad,
						fmt->which);
	return 0;
}

static int ar0330_set_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	unsigned int width;
	unsigned int height;
	unsigned int hratio;
	unsigned int vratio;

	__crop = __ar0330_get_pad_crop(ar0330, fh, format->pad,
					format->which);

	/* Clamp the width and height to avoid dividing by zero. */
	width = clamp_t(unsigned int, ALIGN(format->format.width, 2),
			max(__crop->width / 3, AR0330_WINDOW_WIDTH_MIN),
			__crop->width);
	height = clamp_t(unsigned int, ALIGN(format->format.height, 2),
			max(__crop->height / 3, AR0330_WINDOW_HEIGHT_MIN),
			__crop->height);

	hratio = DIV_ROUND_CLOSEST(__crop->width, width);
	vratio = DIV_ROUND_CLOSEST(__crop->height, height);

	__format = __ar0330_get_pad_format(ar0330, fh, format->pad,
					    format->which);
	__format->width = __crop->width / hratio;
	__format->height = __crop->height / vratio;

	format->format = *__format;

	return 0;
}

static int ar0330_get_crop(struct v4l2_subdev *subdev,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_crop *crop)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);

	crop->rect = *__ar0330_get_pad_crop(ar0330, fh, crop->pad,
					     crop->which);
	return 0;
}

static int ar0330_set_crop(struct v4l2_subdev *subdev,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_crop *crop)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	struct v4l2_rect rect;

	/*
	 * Clamp the crop rectangle boundaries and align them to a multiple of 2
	 * pixels to ensure a GRBG Bayer pattern.
	 */
	rect.left = clamp(ALIGN(crop->rect.left, 2), 0,
			  AR0330_WINDOW_WIDTH_MAX - AR0330_WINDOW_WIDTH_MIN);
	rect.top = clamp(ALIGN(crop->rect.top, 2), 0,
			 AR0330_WINDOW_HEIGHT_MAX - AR0330_WINDOW_HEIGHT_MIN);
	rect.width = clamp(ALIGN(crop->rect.width, 2),
			   AR0330_WINDOW_WIDTH_MIN,
			   AR0330_WINDOW_WIDTH_MAX);
	rect.height = clamp(ALIGN(crop->rect.height, 2),
			    AR0330_WINDOW_HEIGHT_MIN,
			    AR0330_WINDOW_HEIGHT_MAX);

	rect.width = min(rect.width, AR0330_WINDOW_WIDTH_MAX - rect.left);
	rect.height = min(rect.height, AR0330_WINDOW_HEIGHT_MAX - rect.top);

	__crop = __ar0330_get_pad_crop(ar0330, fh, crop->pad, crop->which);

	if (rect.width != __crop->width || rect.height != __crop->height) {
		/*
		 * Reset the output image size if the crop rectangle size has
		 * been modified.
		 */
		__format = __ar0330_get_pad_format(ar0330, fh, crop->pad,
						    crop->which);
		__format->width = rect.width;
		__format->height = rect.height;
	}

	*__crop = rect;
	crop->rect = rect;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */

#define V4L2_CID_TEST_PATTERN		(V4L2_CID_USER_BASE | 0x1001)

static int ar0330_s_ctrl(struct v4l2_ctrl *ctrl)
{
	static const u16 test_pattern[] = { 0, 1, 2, 3, 256, };
	struct ar0330 *ar0330 =
			container_of(ctrl->handler, struct ar0330, ctrls);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		return ar0330_write16(ar0330, AR0330_COARSE_INTEGRATION_TIME,
				      ctrl->val);

	case V4L2_CID_GAIN:
		return ar0330_write16(ar0330, AR0330_GLOBAL_GAIN, ctrl->val);

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP: {
		u16 clr = 0;
		u16 set = 0;

		ret = ar0330_write16(ar0330, AR0330_LOCK_CONTROL,
				     AR0330_LOCK_CONTROL_UNLOCK);
		if (ret < 0)
			return ret;

		if (ar0330->flip[0]->val)
			set |= AR0330_READ_MODE_HORIZ_MIRROR;
		else
			clr |= AR0330_READ_MODE_HORIZ_MIRROR;

		if (ar0330->flip[1]->val)
			set |= AR0330_READ_MODE_VERT_FLIP;
		else
			clr |= AR0330_READ_MODE_VERT_FLIP;

		ret = ar0330_set_read_mode(ar0330, clr, set);

		ar0330_write16(ar0330, AR0330_LOCK_CONTROL, 0);
		return ret;
	}

	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == 1) {
			ret = ar0330_write16(ar0330, AR0330_TEST_DATA_RED,
					     0x0eb0);
			if (ret < 0)
				return ret;
			ret = ar0330_write16(ar0330, AR0330_TEST_DATA_GREENR,
					     0x0690);
			if (ret < 0)
				return ret;
			ret = ar0330_write16(ar0330, AR0330_TEST_DATA_BLUE,
					     0x00b0);
			if (ret < 0)
				return ret;
			ret = ar0330_write16(ar0330, AR0330_TEST_DATA_GREENB,
					     0x0690);
			if (ret < 0)
				return ret;
		}

		return ar0330_write16(ar0330, AR0330_TEST_PATTERN_MODE,
				      test_pattern[ctrl->val]);
	}

	return 0;
}

static struct v4l2_ctrl_ops ar0330_ctrl_ops = {
	.s_ctrl = ar0330_s_ctrl,
};

static const char * const ar0330_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"Full Color Bar",
	"Fade-to-gray Color Bar",
	"Walking 1s",
};

static const struct v4l2_ctrl_config ar0330_ctrls[] = {
	{
		.ops		= &ar0330_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Test Pattern",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0330_test_pattern_menu) - 1,
		.step		= 0,
		.def		= 0,
		.flags		= 0,
		.menu_skip_mask	= 0,
		.qmenu		= ar0330_test_pattern_menu,
	}
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

static int ar0330_set_power(struct v4l2_subdev *subdev, int on)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);
	int ret = 0;

	mutex_lock(&ar0330->power_lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (ar0330->power_count == !on) {
		ret = __ar0330_set_power(ar0330, !!on);
		if (ret < 0)
			goto out;
	}

	/* Update the power count. */
	ar0330->power_count += on ? 1 : -1;
	WARN_ON(ar0330->power_count < 0);

out:
	mutex_unlock(&ar0330->power_lock);
	return ret;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static int ar0330_registered(struct v4l2_subdev *subdev)
{
	struct ar0330 *ar0330 = to_ar0330(subdev);
	s32 revision;
	s32 data;
	int ret;

	ret = ar0330_power_on(ar0330);
	if (ret < 0) {
		dev_err(ar0330->dev, "AR0330 power up failed\n");
		return ret;
	}

	/* Read out the chip version register */
	data = ar0330_read16(ar0330, AR0330_CHIP_VERSION);
	if (data != AR0330_CHIP_VERSION_VALUE) {
		dev_err(ar0330->dev, "AR0330 not detected, wrong version "
			"0x%04x\n", data);
		return -ENODEV;
	}

	revision = ar0330_read8(ar0330, AR0330_CHIP_REVISION);
	if (revision < 0) {
		dev_err(ar0330->dev, "%s: unable to read chip revision (%d)\n",
			__func__, revision);
		return -ENODEV;
	}

	data = ar0330_read16(ar0330, AR0330_TEST_DATA_RED);
	if (data < 0) {
		dev_err(ar0330->dev, "%s: unable to read chip version (%d)\n",
			__func__, data);
		return -ENODEV;
	}

	if (revision == 0x10)
		ar0330->version = 1;
	else if (revision != 0x20)
		ar0330->version = 2;
	else if (data == 0x0006)
		ar0330->version = 3;
	else if (data == 0x0007)
		ar0330->version = 4;
	else
		ar0330->version = 0;

	ar0330_power_off(ar0330);

	dev_info(ar0330->dev, "AR0330 rev. %02x ver. %u detected at address "
		 "0x%02x\n", revision, ar0330->version, ar0330->client->addr);

	return ret;
}

static int ar0330_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	crop = v4l2_subdev_get_try_crop(fh, 0);
	crop->left = (AR0330_WINDOW_WIDTH_MAX - AR0330_WINDOW_WIDTH_DEF) / 2;
	crop->top = (AR0330_WINDOW_HEIGHT_MAX - AR0330_WINDOW_HEIGHT_DEF) / 2;
	crop->width = AR0330_WINDOW_WIDTH_DEF;
	crop->height = AR0330_WINDOW_HEIGHT_DEF;

	format = v4l2_subdev_get_try_format(fh, 0);

	format->code = V4L2_MBUS_FMT_SGRBG12_1X12;
	format->width = AR0330_WINDOW_WIDTH_DEF;
	format->height = AR0330_WINDOW_HEIGHT_DEF;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	return ar0330_set_power(subdev, 1);
}

static int ar0330_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	return ar0330_set_power(subdev, 0);
}

static struct v4l2_subdev_core_ops ar0330_subdev_core_ops = {
	.s_power        = ar0330_set_power,
};

static struct v4l2_subdev_video_ops ar0330_subdev_video_ops = {
	.s_stream       = ar0330_s_stream,
};

static struct v4l2_subdev_pad_ops ar0330_subdev_pad_ops = {
	.enum_mbus_code = ar0330_enum_mbus_code,
	.enum_frame_size = ar0330_enum_frame_size,
	.get_fmt = ar0330_get_format,
	.set_fmt = ar0330_set_format,
	.get_crop = ar0330_get_crop,
	.set_crop = ar0330_set_crop,
};

static struct v4l2_subdev_ops ar0330_subdev_ops = {
	.core   = &ar0330_subdev_core_ops,
	.video  = &ar0330_subdev_video_ops,
	.pad    = &ar0330_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0330_subdev_internal_ops = {
	.registered = ar0330_registered,
	.open = ar0330_open,
	.close = ar0330_close,
};

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */

static int ar0330_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ar0330 *ar0330;
	unsigned int i;
	int ret;

	ar0330 = kzalloc(sizeof(*ar0330), GFP_KERNEL);
	if (ar0330 == NULL)
		return -ENOMEM;

	ar0330->client = client;
	ar0330->dev = &client->dev;
	ar0330->read_mode = 0;

	ar0330->clock = devm_clk_get(ar0330->dev, NULL);
	if (IS_ERR(ar0330->clock)) {
		ret = PTR_ERR(ar0330->clock);
		if (ret != -EPROBE_DEFER)
			dev_err(ar0330->dev, "Failed to get clock: %d\n",
				ret);
		goto done;
	}

	ar0330->reset = devm_gpiod_get_optional(ar0330->dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(ar0330->reset)) {
		ret = PTR_ERR(ar0330->reset);
		if (ret != -EPROBE_DEFER)
			dev_err(ar0330->dev, "Failed to get reset GPIO: %d\n",
				ret);
		goto done;
	}

	v4l2_ctrl_handler_init(&ar0330->ctrls, ARRAY_SIZE(ar0330_ctrls) + 5);

	v4l2_ctrl_new_std(&ar0330->ctrls, &ar0330_ctrl_ops,
			  V4L2_CID_EXPOSURE, AR0330_COARSE_INTEGRATION_TIME_MIN,
			  AR0330_COARSE_INTEGRATION_TIME_MAX, 1,
			  AR0330_COARSE_INTEGRATION_TIME_DEF);
	v4l2_ctrl_new_std(&ar0330->ctrls, &ar0330_ctrl_ops,
			  V4L2_CID_GAIN, AR0330_GLOBAL_GAIN_MIN,
			  AR0330_GLOBAL_GAIN_MAX, 1, AR0330_GLOBAL_GAIN_DEF);
	ar0330->flip[0] = v4l2_ctrl_new_std(&ar0330->ctrls, &ar0330_ctrl_ops,
					    V4L2_CID_HFLIP, 0, 1, 1, 0);
	ar0330->flip[1] = v4l2_ctrl_new_std(&ar0330->ctrls, &ar0330_ctrl_ops,
					    V4L2_CID_VFLIP, 0, 1, 1, 0);
	ar0330->pixel_rate = v4l2_ctrl_new_std(&ar0330->ctrls, &ar0330_ctrl_ops,
					       V4L2_CID_IMAGE_PROC_PIXEL_RATE,
					       0, 0, 1, 0);

	for (i = 0; i < ARRAY_SIZE(ar0330_ctrls); ++i)
		v4l2_ctrl_new_custom(&ar0330->ctrls, &ar0330_ctrls[i], NULL);

	v4l2_ctrl_cluster(ARRAY_SIZE(ar0330->flip), ar0330->flip);

	if (ar0330->ctrls.error)
		dev_err(ar0330->dev, "%s: control initialization error %d\n",
			__func__, ar0330->ctrls.error);

	ar0330->subdev.ctrl_handler = &ar0330->ctrls;

	mutex_init(&ar0330->power_lock);
	v4l2_i2c_subdev_init(&ar0330->subdev, client, &ar0330_subdev_ops);
	ar0330->subdev.internal_ops = &ar0330_subdev_internal_ops;

	ar0330->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&ar0330->subdev.entity, 1, &ar0330->pad, 0);
	if (ret < 0)
		goto done;

	ar0330->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ar0330->crop.width = AR0330_WINDOW_WIDTH_DEF;
	ar0330->crop.height = AR0330_WINDOW_HEIGHT_DEF;
	ar0330->crop.left = (AR0330_WINDOW_WIDTH_MAX - AR0330_WINDOW_WIDTH_DEF)
			  / 2;
	ar0330->crop.top = (AR0330_WINDOW_HEIGHT_MAX - AR0330_WINDOW_HEIGHT_DEF)
			 / 2;

	ar0330->format.code = V4L2_MBUS_FMT_SGRBG12_1X12;
	ar0330->format.width = AR0330_WINDOW_WIDTH_DEF;
	ar0330->format.height = AR0330_WINDOW_HEIGHT_DEF;
	ar0330->format.field = V4L2_FIELD_NONE;
	ar0330->format.colorspace = V4L2_COLORSPACE_SRGB;

	ret = ar0330_pll_init(ar0330);

done:
	if (ret < 0) {
		v4l2_ctrl_handler_free(&ar0330->ctrls);
		media_entity_cleanup(&ar0330->subdev.entity);
		kfree(ar0330);
	}

	return ret;
}

static int ar0330_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ar0330 *ar0330 = to_ar0330(subdev);

	v4l2_ctrl_handler_free(&ar0330->ctrls);
	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	kfree(ar0330);

	return 0;
}

static const struct i2c_device_id ar0330_id[] = {
	{ "ar0330", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ar0330_id);

static struct i2c_driver ar0330_i2c_driver = {
	.driver = {
		.name = "ar0330",
	},
	.probe          = ar0330_probe,
	.remove         = ar0330_remove,
	.id_table       = ar0330_id,
};

static int __init ar0330_mod_init(void)
{
	return i2c_add_driver(&ar0330_i2c_driver);
}

static void __exit ar0330_mod_exit(void)
{
	i2c_del_driver(&ar0330_i2c_driver);
}

module_init(ar0330_mod_init);
module_exit(ar0330_mod_exit);

MODULE_DESCRIPTION("Aptina AR0330 Camera driver");
MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_LICENSE("GPL v2");
