// SPDX-License-Identifier: GPL-2.0

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#include "mtk_seninf_reg.h"

#define SENINF_TIMESTAMP_STEP		0x67
#define SENINF_SETTLE_DELAY		0x15
#define SENINF_HS_TRAIL_PARAMETER	0x8

#define SENINF_MAX_NUM_INPUTS		4
#define SENINF_MAX_NUM_OUTPUTS		4
#define SENINF_MAX_NUM_PADS		(SENINF_MAX_NUM_INPUTS + \
					 SENINF_MAX_NUM_OUTPUTS)

#define SENINF_DEFAULT_WIDTH		1920
#define SENINF_DEFAULT_HEIGHT		1080

#define SENINF_PAD_10BIT		0

#define SENINF_TEST_MODEL          	0
#define SENINF_NORMAL_MODEL        	1
#define SENINF_ALL_ERR_IRQ_EN		0x7f
#define SENINF_IRQ_CLR_SEL		0x80000000

#define SENINF_MIPI_SENSOR		0x8

enum TEST_MODE {
	TEST_PATTERN_DISABLED = 0x0,
	TEST_PATTERN_ENABLED
};

/* Port number in the device tree. */
enum mtk_seninf_port {
	CSI_PORT_0 = 0,	/* 4D1C or 2D1C */
	CSI_PORT_1,	/* 4D1C */
	CSI_PORT_2,	/* 4D1C */
	CSI_PORT_0B,	/* 2D1C */
};

enum PIXEL_MODE {
	ONE_PIXEL_MODE  = 0x0,
	TWO_PIXEL_MODE  = 0x1,
	FOUR_PIXEL_MODE = 0x2,
};

enum mtk_seninf_id {
	SENINF_1 = 0,
	SENINF_2 = 1,
	SENINF_3 = 2,
	SENINF_5 = 4,
};

enum mtk_seninf_phy {
	SENINF_PHY_CSI0 = 0,
	SENINF_PHY_CSI1 = 1,
	SENINF_PHY_CSI2 = 2,
	SENINF_PHY_CSI0A = 3,
	SENINF_PHY_CSI0B = 4,
};

enum mtk_seninf_phy_mode {
	SENINF_PHY_MODE_NONE,
	SENINF_PHY_MODE_4D1C,
	SENINF_PHY_MODE_2D1C,
};

enum mtk_seninf_format_flag {
	MTK_SENINF_FORMAT_BAYER = BIT(0),
	MTK_SENINF_FORMAT_DPCM = BIT(1),
	MTK_SENINF_FORMAT_JPEG = BIT(2),
	MTK_SENINF_FORMAT_INPUT_ONLY = BIT(3),
};

enum mtk_seninf_version {
	SENINF_20,
	SENINF_50,
};

enum mtk_seninf_csi2_rx_type {
	MTK_SENINF_CSI2_RX_NCSI2,
	MTK_SENINF_CSI2_RX_CSI2,
};

struct mtk_seninf_conf {
	enum mtk_seninf_version seninf_version;
	char *model;
	enum mtk_seninf_csi2_rx_type csi2_rx_type;
	u8 nb_inputs;
	u8 nb_outputs;
	u8 nb_phy;
};

static const struct mtk_seninf_conf seninf_8183_conf = {
	.seninf_version = SENINF_50,
	.model = "mtk-camsys-5.0",
	.csi2_rx_type = MTK_SENINF_CSI2_RX_CSI2,
	.nb_inputs = 4,
	.nb_outputs = 4,
	.nb_phy = 5,
};

struct mtk_seninf_format_info {
	u32 code;
	u32 flags;
};

struct mtk_seninf_input {
	enum mtk_seninf_port pad;
	enum mtk_seninf_id seninf_id;
	void __iomem *base;
	struct mtk_seninf *seninf;

	struct phy *phy;
	enum mtk_seninf_phy_mode phy_mode;

	struct v4l2_fwnode_bus_mipi_csi2 bus;

	struct v4l2_subdev *subdev;
	struct v4l2_mbus_framefmt format;

	unsigned int source_pad;
};

struct mtk_seninf {
	struct device *dev;
	struct phy *phy[5];
	unsigned int num_clks;
	struct clk_bulk_data *clks;
	void __iomem *base;

	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev subdev;
	struct media_pad pads[SENINF_MAX_NUM_PADS];
	struct v4l2_async_notifier notifier;
	struct v4l2_ctrl_handler ctrl_handler;

	struct v4l2_mbus_framefmt source_format;

	struct mtk_seninf_input inputs[SENINF_MAX_NUM_INPUTS];
	struct mtk_seninf_input *active_input;

	const struct mtk_seninf_conf *conf;

	bool is_testmode;
};

inline struct mtk_seninf *sd_to_mtk_seninf(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_seninf, subdev);
}

/* -----------------------------------------------------------------------------
 * Formats
 */

static const struct mtk_seninf_format_info mtk_seninf_formats[] = {
	{
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR14_1X14,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG14_1X14,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG14_1X14,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR16_1X16,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG16_1X16,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG16_1X16,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB16_1X16,
		.flags = MTK_SENINF_FORMAT_BAYER,
	}, {
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
	}, {
		.code = MEDIA_BUS_FMT_VYUY8_1X16,
	}, {
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.code = MEDIA_BUS_FMT_YVYU8_1X16,
	}, {
		.code = MEDIA_BUS_FMT_JPEG_1X8,
		.flags = MTK_SENINF_FORMAT_JPEG,
	}, {
		.code = MEDIA_BUS_FMT_S5C_UYVY_JPEG_1X8,
		.flags = MTK_SENINF_FORMAT_JPEG,
	},
	/* Keep the input-only formats last. */
	{
		.code = MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8,
		.flags = MTK_SENINF_FORMAT_DPCM | MTK_SENINF_FORMAT_INPUT_ONLY,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8,
		.flags = MTK_SENINF_FORMAT_DPCM | MTK_SENINF_FORMAT_INPUT_ONLY,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8,
		.flags = MTK_SENINF_FORMAT_DPCM | MTK_SENINF_FORMAT_INPUT_ONLY,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8,
		.flags = MTK_SENINF_FORMAT_DPCM | MTK_SENINF_FORMAT_INPUT_ONLY,
	}
};

static const struct mtk_seninf_format_info *mtk_seninf_format_info(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_seninf_formats); ++i) {
		if (mtk_seninf_formats[i].code == code)
			return &mtk_seninf_formats[i];
	}

	return NULL;
}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

/*
 * In order to minimize code difference between ISP2.0 and ISP5.0,
 * we use an array to convert addresses coming from ISP5.0 to match
 * ISP2.0 register topology.
 */
static const u16 mtk_seninf_20_translation_table[] = {
	0x0000, /* 0x0: top_ctrl */
	0xffff, /* 0x1: NA */
	0x0100, /* 0x2: ctrl */
	0x0300, /* 0x3: NCSI2 */
	0xffff, /* 0x4: NA */
	0xffff, /* 0x5: NA */
	0x0020, /* 0x6: TG */
	0xffff, /* 0x7: NA */
	0xffff, /* 0x8: NA */
	0xffff, /* 0x9: NA */
	0xffff, /* 0xa: NA */
	0xffff, /* 0xb: NA */
	0xffff, /* 0xc: NA */
	0x0120, /* 0xd: top_mux_ctrl */
	0xffff, /* 0xe: NA */
	0xffff, /* 0xf: NA */
};

/*
 * To compute ISP2.0 address we keep the 16 lower bits of register
 * and apply a translation thanks to the mtk_isp20_translation_table.
 */
static u32 mtk_seninf_20_address_translation(u32 reg)
{
	return mtk_seninf_20_translation_table[(reg >> 8) & 0xf] + (reg & 0xff);
}

static u32 mtk_seninf_read(struct mtk_seninf *priv, u32 reg)
{
	if (priv->conf->seninf_version == SENINF_20)
		reg = mtk_seninf_20_address_translation(reg);

	return readl(priv->base + reg);
}

static void mtk_seninf_write(struct mtk_seninf *priv, u32 reg, u32 value)
{
	if (priv->conf->seninf_version == SENINF_20)
		reg = mtk_seninf_20_address_translation(reg);

	writel(value, priv->base + reg);
}

static void __mtk_seninf_update(struct mtk_seninf *priv, u32 reg,
				u32 mask, u32 shift, u32 value)
{
	u32 val = mtk_seninf_read(priv, reg);

	mtk_seninf_write(priv, reg, (val & ~mask) | (value << shift));
}

#define mtk_seninf_update(priv, reg, field, val)			\
	__mtk_seninf_update(priv, reg, reg##_##field##_MASK,		\
			    reg##_##field##_SHIFT, val)

static u32 mtk_seninf_input_read(struct mtk_seninf_input *input, u32 reg)
{
	if (input->seninf->conf->seninf_version == SENINF_20)
		reg = mtk_seninf_20_address_translation(reg);

	return readl(input->base + reg);
}

static void mtk_seninf_input_write(struct mtk_seninf_input *input, u32 reg,
				   u32 value)
{
	if (input->seninf->conf->seninf_version == SENINF_20)
		reg = mtk_seninf_20_address_translation(reg);

	writel(value, input->base + reg);
}

static void __mtk_seninf_input_update(struct mtk_seninf_input *input, u32 reg,
				      u32 mask, u32 shift, u32 value)
{
	u32 val = mtk_seninf_input_read(input, reg);

	mtk_seninf_input_write(input, reg, (val & ~mask) | (value << shift));
}

#define mtk_seninf_input_update(input, reg, field, val)			\
	__mtk_seninf_input_update(input, reg, reg##_##field##_MASK,	\
				  reg##_##field##_SHIFT, val)

static void mtk_seninf_set_mux(struct mtk_seninf *priv,
			       struct mtk_seninf_input *input)
{
	const struct mtk_seninf_format_info *fmtinfo;
	unsigned int val, pos;
	const struct mtk_seninf_conf *conf = priv->conf;
	unsigned int pix_sel_ext;
	unsigned int pix_sel;
	unsigned int hs_pol = 0;
	unsigned int vs_pol = 0;
	unsigned int pixel_mode = TWO_PIXEL_MODE;

	fmtinfo = mtk_seninf_format_info(input->format.code);

	/* Enable mux */
	mtk_seninf_input_update(input, SENINF_MUX_CTRL, SENINF_MUX_EN, 1);
	mtk_seninf_input_update(input, SENINF_MUX_CTRL, SENINF_SRC_SEL,
				SENINF_MIPI_SENSOR);
	if (conf->seninf_version == SENINF_50)
		mtk_seninf_input_update(input, SENINF_MUX_CTRL_EXT,
			    SENINF_SRC_SEL_EXT, SENINF_NORMAL_MODEL);

	switch (pixel_mode) {
	case 1: /* 2 Pixel */
		pix_sel_ext = 0;
		pix_sel = 1;
		break;
	case 2: /* 4 Pixel */
		pix_sel_ext = 1;
		pix_sel = 0;
		break;
	default: /* 1 Pixel */
		pix_sel_ext = 0;
		pix_sel = 0;
		break;
	}

	if (conf->seninf_version == SENINF_50)
		mtk_seninf_input_update(input, SENINF_MUX_CTRL_EXT,
			    SENINF_PIX_SEL_EXT, pix_sel_ext);
	mtk_seninf_input_update(input, SENINF_MUX_CTRL, SENINF_PIX_SEL, pix_sel);

	if (!(fmtinfo->flags & MTK_SENINF_FORMAT_JPEG)) {
		mtk_seninf_input_update(input, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 2);
		mtk_seninf_input_update(input, SENINF_MUX_CTRL, FIFO_FLUSH_EN, 0x1b);
		mtk_seninf_input_update(input, SENINF_MUX_CTRL, FIFO_PUSH_EN, 0x1f);
	} else {
		mtk_seninf_input_update(input, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 0);
		mtk_seninf_input_update(input, SENINF_MUX_CTRL, FIFO_FLUSH_EN, 0x18);
		mtk_seninf_input_update(input, SENINF_MUX_CTRL, FIFO_PUSH_EN, 0x1e);
	}

	mtk_seninf_input_update(input, SENINF_MUX_CTRL, SENINF_HSYNC_POL, hs_pol);
	mtk_seninf_input_update(input, SENINF_MUX_CTRL, SENINF_VSYNC_POL, vs_pol);

	val = mtk_seninf_input_read(input, SENINF_MUX_CTRL);
	mtk_seninf_input_write(input, SENINF_MUX_CTRL, val | 0x00000003);
	mtk_seninf_input_write(input, SENINF_MUX_CTRL, val & 0xfffffffc);

	mtk_seninf_write(priv, SENINF_TOP_MUX_CTRL, 0x00043210);

	/* HQ */
	mtk_seninf_input_write(input, SENINF_MUX_SPARE, 0xc2000);

	/*
	 * Hardcode the top mux (from SENINF input to async FIFO) with a direct
	 * mapping, and use the top cam mux to configure routing from the async
	 * FIFOs to the outputs (CAM and CAMSV).
	 */
	if (conf->seninf_version == SENINF_50) {
		pos = input->source_pad - conf->nb_inputs + 2;
		val = (mtk_seninf_read(priv, SENINF_TOP_CAM_MUX_CTRL)
		       & ~(0xF << (pos * 4))) |
		       ((input->seninf_id & 0xF) << (pos * 4));
		mtk_seninf_write(priv, SENINF_TOP_CAM_MUX_CTRL, val);
	}
}

static void mtk_seninf_csi2_setup_phy(struct mtk_seninf *priv)
{
	/* CSI0(A) and CSI0B */
	if (priv->inputs[CSI_PORT_0].phy_mode ||
	    priv->inputs[CSI_PORT_0B].phy_mode) {
		struct mtk_seninf_input *input_a = &priv->inputs[CSI_PORT_0];
		struct mtk_seninf_input *input_b = &priv->inputs[CSI_PORT_0B];
		unsigned int csi0b_clock;
		unsigned int dphy_mode;

		/*
		 * If CSI0B is enabled, use its clock lane. Otherwise set it to
		 * CSI0B lane 2 to ensure it won't conflict with any lane used
		 * by CSI0(A).
		 */
		csi0b_clock = input_b->phy_mode ? input_b->bus.clock_lane : 2;

		/*
		 * If CSI0A operates in 4D1C then the whole port operates in
		 * 4D1C, otherwise we have either a single or a dual 2D1C
		 * configuration.
		 */
		dphy_mode = input_a->phy_mode == SENINF_PHY_MODE_4D1C ? 0 : 1;

		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI0,
				  DPHY_MODE, dphy_mode);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI0,
				  CK_SEL_1, input_a->bus.clock_lane);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI0,
				  CK_SEL_2, csi0b_clock);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI0,
				  PHY_SENINF_LANE_MUX_CSI0_EN, 1);
	}

	/* CSI1 */
	if (priv->inputs[CSI_PORT_1].phy_mode) {
		struct mtk_seninf_input *input = &priv->inputs[CSI_PORT_1];

		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI1,
				  DPHY_MODE, 0 /* 4D1C */);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI1,
				  CK_SEL_1, input->bus.clock_lane);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI1,
				  PHY_SENINF_LANE_MUX_CSI1_EN, 1);
	}

	/* CSI2 */
	if (priv->inputs[CSI_PORT_2].phy_mode) {
		struct mtk_seninf_input *input = &priv->inputs[CSI_PORT_2];

		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI2,
				  DPHY_MODE, 0 /* 4D1C */);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI2,
				  CK_SEL_1, input->bus.clock_lane);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI2,
				  PHY_SENINF_LANE_MUX_CSI2_EN, 1);
	}
}

static void mtk_seninf_csi2_rx_config(struct mtk_seninf *priv,
				 struct mtk_seninf_input *input)
{
	unsigned int lanes[4] = { };
	unsigned int i;

	/*
	 * Configure data lane muxing. In 2D1C mode, lanes 0 to 2 correspond to
	 * CSIx[AB]_L{0,1,2}, and in 4D1C lanes 0 to 5 correspond to
	 * CSIxA_L{0,1,2}, CSIxB_L{0,1,2}.
	 *
	 * The clock lane must be skipped when calculating the index of the
	 * physical data lane. For instance, in 4D1C mode, the sensor clock
	 * lane is typically connected to lane 2 (CSIxA_L2), and the sensor
	 * data lanes 0-3 to lanes 1 (CSIxA_L1), 3 (CSIxB_L0), 0 (CSIxA_L0) and
	 * 4 (CSIxB_L1). The when skipping the clock lane, the data lane
	 * indices become 1, 2, 0 and 3.
	 */
	for (i = 0; i < input->bus.num_data_lanes; ++i) {
		lanes[i] = input->bus.data_lanes[i];
		if (lanes[i] > input->bus.clock_lane)
			lanes[i]--;
	}

	mtk_seninf_input_update(input, MIPI_RX_CON24_CSI0,
				CSI0_BIST_LN0_MUX, lanes[0]);
	mtk_seninf_input_update(input, MIPI_RX_CON24_CSI0,
				CSI0_BIST_LN1_MUX, lanes[1]);
	mtk_seninf_input_update(input, MIPI_RX_CON24_CSI0,
				CSI0_BIST_LN2_MUX, lanes[2]);
	mtk_seninf_input_update(input, MIPI_RX_CON24_CSI0,
				CSI0_BIST_LN3_MUX, lanes[3]);
}

static void mtk_seninf_csi2_set_mipi(struct mtk_seninf *priv,
				     struct mtk_seninf_input *input)
{
	const struct mtk_seninf_format_info *fmtinfo;
	unsigned int dpcm;
	unsigned int data_lane_num = input->bus.num_data_lanes;
	unsigned int data_header_order = 1;
	unsigned int val = 0;

	fmtinfo = mtk_seninf_format_info(input->format.code);

	/* Configure timestamp */
	mtk_seninf_input_write(input, SENINF_TG1_TM_STP, SENINF_TIMESTAMP_STEP);

	/* HQ */
	mtk_seninf_input_write(input, SENINF_TG1_PH_CNT, 0x0);
	mtk_seninf_input_write(input, SENINF_TG1_SEN_CK, 0x10001);

	/* First Enable Sensor interface and select pad (0x1a04_0200) */
	mtk_seninf_input_update(input, SENINF_CTRL, SENINF_EN, 1);
	mtk_seninf_input_update(input, SENINF_CTRL, PAD2CAM_DATA_SEL, SENINF_PAD_10BIT);
	mtk_seninf_input_update(input, SENINF_CTRL, SENINF_SRC_SEL, 0);
	mtk_seninf_input_update(input, SENINF_CTRL_EXT, SENINF_CSI2_IP_EN, 1);
	mtk_seninf_input_update(input, SENINF_CTRL_EXT, SENINF_NCSI2_IP_EN, 0);

	/* DPCM Enable */
	dpcm = fmtinfo->flags & MTK_SENINF_FORMAT_DPCM ? 0x2a : 0;
	val = 1 << ((dpcm == 0x2a) ? 15 : ((dpcm & 0xF) + 7));
	mtk_seninf_input_write(input, SENINF_CSI2_DPCM, val);

	/* Settle delay */
	mtk_seninf_input_update(input, SENINF_CSI2_LNRD_TIMING,
				DATA_SETTLE_PARAMETER, SENINF_SETTLE_DELAY);

	/* HQ */
	mtk_seninf_input_write(input, SENINF_CSI2_LNRC_FSM, 0x10);

	/* CSI2 control */
	val = mtk_seninf_input_read(input, SENINF_CSI2_CTL)
	    | (data_header_order << 16) | 0x10 | ((1 << data_lane_num) - 1);
	mtk_seninf_input_write(input, SENINF_CSI2_CTL, val);

	mtk_seninf_input_update(input, SENINF_CSI2_RESYNC_MERGE_CTL,
				BYPASS_LANE_RESYNC, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_RESYNC_MERGE_CTL, CDPHY_SEL, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_RESYNC_MERGE_CTL,
				CPHY_LANE_RESYNC_CNT, 3);
	mtk_seninf_input_update(input, SENINF_CSI2_MODE, CSR_CSI2_MODE, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_MODE, CSR_CSI2_HEADER_LEN, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_DPHY_SYNC, SYNC_SEQ_MASK_0, 0xff00);
	mtk_seninf_input_update(input, SENINF_CSI2_DPHY_SYNC, SYNC_SEQ_PAT_0, 0x001d);

	mtk_seninf_input_update(input, SENINF_CSI2_CTL, CLOCK_HS_OPTION, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_CTL, HSRX_DET_EN, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_CTL, HS_TRAIL_EN, 1);
	mtk_seninf_input_update(input, SENINF_CSI2_HS_TRAIL, HS_TRAIL_PARAMETER,
				SENINF_HS_TRAIL_PARAMETER);

	/* Set debug port to output packet number */
	mtk_seninf_input_update(input, SENINF_CSI2_DGB_SEL, DEBUG_EN, 1);
	mtk_seninf_input_update(input, SENINF_CSI2_DGB_SEL, DEBUG_SEL, 0x1a);

	/* HQ */
	mtk_seninf_input_write(input, SENINF_CSI2_SPARE0, 0xfffffffe);

	/* Enable CSI2 IRQ mask */
	/* Turn on all interrupt */
	mtk_seninf_input_write(input, SENINF_CSI2_INT_EN, 0xffffffff);
	/* Write clear CSI2 IRQ */
	mtk_seninf_input_write(input, SENINF_CSI2_INT_STATUS, 0xffffffff);
	/* Enable CSI2 Extend IRQ mask */
	/* Turn on all interrupt */
	mtk_seninf_input_update(input, SENINF_CTRL, CSI2_SW_RST, 1);
	udelay(1);
	mtk_seninf_input_update(input, SENINF_CTRL, CSI2_SW_RST, 0);
}

static void mtk_seninf_ncsi2_set_mipi(struct mtk_seninf *priv,
				      struct mtk_seninf_input *input)
{
	const struct mtk_seninf_format_info *fmtinfo;
	unsigned int val;

	fmtinfo = mtk_seninf_format_info(input->format.code);

	/* HQ */
	mtk_seninf_input_write(input, SENINF_TG1_PH_CNT, 0x0);
	mtk_seninf_input_write(input, SENINF_TG1_SEN_CK, 0x10001);

	/* First Enable Sensor interface and select pad (0x1a04_0200) */
	mtk_seninf_input_update(input, SENINF_CTRL, SENINF_EN, 1);
	mtk_seninf_input_update(input, SENINF_CTRL, PAD2CAM_DATA_SEL,
		    SENINF_PAD_10BIT);
	mtk_seninf_input_update(input, SENINF_CTRL, SENINF_SRC_SEL, 8);

	mtk_seninf_input_write(input, SENINF_NCSI2_CAL_38, 1U);
	mtk_seninf_input_write(input, SENINF_NCSI2_CAL_3C, 0x00051545U);
	mtk_seninf_input_write(input, SENINF_NCSI2_CAL_38, 5U);
	mdelay(1);
	mtk_seninf_input_write(input, SENINF_NCSI2_CAL_38, 4U);
	mtk_seninf_input_write(input, SENINF_NCSI2_CAL_3C, 0U);
	mtk_seninf_input_write(input, SENINF_NCSI2_DBG_SEL, 0x11U);
	mtk_seninf_input_update(input, SENINF_NCSI2_CTL, ED_SEL, 1);
	mtk_seninf_input_update(input, SENINF_NCSI2_CTL, CLOCK_LANE, 1);
	mtk_seninf_input_update(input, SENINF_NCSI2_CTL, DATA_LANE3, 1);
	mtk_seninf_input_update(input, SENINF_NCSI2_CTL, DATA_LANE2, 1);
	mtk_seninf_input_update(input, SENINF_NCSI2_CTL, DATA_LANE1, 1);
	mtk_seninf_input_update(input, SENINF_NCSI2_CTL, DATA_LANE0, 1);
	mtk_seninf_input_update(input, SENINF_NCSI2_CTL, CLOCK_HS_OPTION, 0);
	mtk_seninf_input_update(input, SENINF_NCSI2_CTL, CLOCK_HS_OPTION, 1);
	mtk_seninf_input_write(input, SENINF_NCSI2_LNRD_TIMING, 0x2800U);
	mtk_seninf_input_write(input, SENINF_NCSI2_INT_STATUS,
			       SENINF_NCSI2_INT_STATUS_ALL);
	mtk_seninf_input_write(input, SENINF_NCSI2_INT_EN,
			       SENINF_NCSI2_INT_EN_ALL);
	mtk_seninf_input_write(input, SENINF_NCSI2_CAL_24, 0xE4000000U);
	val = 0xFFFFFF00U & mtk_seninf_input_read(input, SENINF_NCSI2_DBG_SEL);
	mtk_seninf_input_write(input, SENINF_NCSI2_DBG_SEL, val);
	val = 0xFFFFFF45U | mtk_seninf_input_read(input, SENINF_NCSI2_DBG_SEL);
	mtk_seninf_input_write(input, SENINF_NCSI2_DBG_SEL, val);
	val = 0xFFFFFFEFU & mtk_seninf_input_read(input, SENINF_NCSI2_HSRX_DBG);
	mtk_seninf_input_write(input, SENINF_NCSI2_HSRX_DBG, val);
	mtk_seninf_input_write(input, SENINF_NCSI2_DI_CTRL, 0x01010101U);
	mtk_seninf_input_write(input, SENINF_NCSI2_DI, 0x03020100U);
	mtk_seninf_input_write(input, SENINF_NCSI2_DBG_SEL, 0x10);
}

static int seninf_enable_test_pattern(struct mtk_seninf *priv)
{
	const struct mtk_seninf_format_info *fmtinfo;
	const struct mtk_seninf_conf *conf = priv->conf;
	unsigned int val;
	unsigned int pixel_mode = TWO_PIXEL_MODE;
	unsigned int pix_sel_ext;
	unsigned int pix_sel;
	unsigned int hs_pol = 0;
	unsigned int vs_pol = 0;
	unsigned int seninf = 0;
	unsigned int mux = 0;
	int ret;

	fmtinfo = mtk_seninf_format_info(priv->source_format.code);

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to pm_runtime_get_sync: %d\n", ret);
		pm_runtime_put_noidle(priv->dev);
		return ret;
	}

	mtk_seninf_update(priv, SENINF_TOP_CTRL, MUX_LP_MODE, 0);

	mtk_seninf_update(priv, SENINF_TOP_CTRL, SENINF_PCLK_EN, 1);
	mtk_seninf_update(priv, SENINF_TOP_CTRL, SENINF2_PCLK_EN, 1);

	mtk_seninf_update(priv, SENINF_CTRL, SENINF_EN, 1);
	mtk_seninf_update(priv, SENINF_CTRL, SENINF_SRC_SEL, 1);
	if (conf->seninf_version == SENINF_50)
		mtk_seninf_update(priv, SENINF_CTRL_EXT,
			    SENINF_TESTMDL_IP_EN, 1);

	mtk_seninf_update(priv, SENINF_TG1_TM_CTL, TM_EN, 1);
	mtk_seninf_update(priv, SENINF_TG1_TM_CTL, TM_PAT, 0xc);
	mtk_seninf_update(priv, SENINF_TG1_TM_CTL, TM_VSYNC, 4);
	mtk_seninf_update(priv, SENINF_TG1_TM_CTL, TM_DUMMYPXL, 0x28);

	if (fmtinfo->flags & MTK_SENINF_FORMAT_BAYER)
		mtk_seninf_update(priv, SENINF_TG1_TM_CTL, TM_FMT, 0x0);
	else
		mtk_seninf_update(priv, SENINF_TG1_TM_CTL, TM_FMT, 0x1);

	switch (priv->source_format.code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
		mtk_seninf_write(priv, SENINF_TG1_TM_SIZE,
				 (priv->source_format.height + 8) << 16 |
				 priv->source_format.width * 2);
		break;
	default:
		mtk_seninf_write(priv, SENINF_TG1_TM_SIZE,
				 (priv->source_format.height + 8) << 16 |
				 priv->source_format.width);
		break;
	}

	mtk_seninf_write(priv, SENINF_TG1_TM_CLK, 0x8);
	if (conf->seninf_version == SENINF_50)
		mtk_seninf_write(priv, SENINF_TG1_TM_STP, 0x1);

	/* Set top mux */
	val = (mtk_seninf_read(priv, SENINF_TOP_MUX_CTRL) & (~(0xf << (mux * 4)))) |
	      ((seninf & 0xf) << (mux * 4));
	mtk_seninf_write(priv, SENINF_TOP_MUX_CTRL, val);

	/* TODO : if mux != 0 => use pseninf + 0x1000 * mux */
	mtk_seninf_update(priv, SENINF_MUX_CTRL, SENINF_MUX_EN, 1);
	if (conf->seninf_version == SENINF_50)
		mtk_seninf_update(priv, SENINF_MUX_CTRL_EXT,
			    SENINF_SRC_SEL_EXT, SENINF_TEST_MODEL);
	mtk_seninf_update(priv, SENINF_MUX_CTRL, SENINF_SRC_SEL, 1);

	switch (pixel_mode) {
	case 1:
		pix_sel_ext = 0;
		pix_sel = 1;
		break;
	case 2:
		pix_sel_ext = 1;
		pix_sel = 0;
		break;
	default:
		pix_sel_ext = 0;
		pix_sel = 0;
		break;
	}

	if (conf->seninf_version == SENINF_50)
		mtk_seninf_update(priv, SENINF_MUX_CTRL_EXT,
			    SENINF_PIX_SEL_EXT, pix_sel_ext);

	mtk_seninf_update(priv, SENINF_MUX_CTRL, SENINF_PIX_SEL, pix_sel);

	mtk_seninf_update(priv, SENINF_MUX_CTRL, FIFO_PUSH_EN,
		    0x1f);
	mtk_seninf_update(priv, SENINF_MUX_CTRL, FIFO_FLUSH_EN,
		    0x1b);
	mtk_seninf_update(priv, SENINF_MUX_CTRL, FIFO_FULL_WR_EN,
		    2);

	mtk_seninf_update(priv, SENINF_MUX_CTRL, SENINF_HSYNC_POL, hs_pol);
	mtk_seninf_update(priv, SENINF_MUX_CTRL, SENINF_VSYNC_POL, vs_pol);
	mtk_seninf_update(priv, SENINF_MUX_CTRL, SENINF_HSYNC_MASK, 1);

	mtk_seninf_write(priv, SENINF_MUX_INTEN,
			 SENINF_IRQ_CLR_SEL | SENINF_ALL_ERR_IRQ_EN);

	mtk_seninf_write(priv, SENINF_MUX_CTRL,
			 mtk_seninf_read(priv, SENINF_MUX_CTRL) | 0x3);
	udelay(1);
	mtk_seninf_write(priv, SENINF_MUX_CTRL,
			 mtk_seninf_read(priv, SENINF_MUX_CTRL) & ~0x3);

	if (conf->seninf_version == SENINF_50)
		mtk_seninf_write(priv, SENINF_TOP_CAM_MUX_CTRL, 0x76543010);

	dev_dbg(priv->dev, "%s: OK\n", __func__);
	return 0;
}

static int mtk_seninf_power_on(struct mtk_seninf *priv)
{
	struct mtk_seninf_input *input = priv->active_input;
	const struct mtk_seninf_conf *conf = priv->conf;
	int ret;

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to pm_runtime_get_sync: %d\n", ret);
		pm_runtime_put_noidle(priv->dev);
		return ret;
	}

	if (conf->csi2_rx_type == MTK_SENINF_CSI2_RX_CSI2)
		mtk_seninf_csi2_setup_phy(priv);

	phy_power_on(input->phy);

	if (conf->csi2_rx_type == MTK_SENINF_CSI2_RX_CSI2) {
		mtk_seninf_csi2_rx_config(priv, input);
		mtk_seninf_csi2_set_mipi(priv, input);
	} else if (conf->csi2_rx_type == MTK_SENINF_CSI2_RX_NCSI2) {
		mtk_seninf_ncsi2_set_mipi(priv, input);
	}

	mtk_seninf_set_mux(priv, input);

	return 0;
}

static void mtk_seninf_power_off(struct mtk_seninf *priv)
{
	const struct mtk_seninf_conf *conf = priv->conf;
	unsigned int val;

	if (priv->active_input) {
		struct mtk_seninf_input *input = priv->active_input;

		if (conf->csi2_rx_type == MTK_SENINF_CSI2_RX_CSI2) {
			/* Disable CSI2(2.5G) first */
			val = mtk_seninf_input_read(input, SENINF_CSI2_CTL);
			mtk_seninf_input_write(
				input, SENINF_CSI2_CTL, val & 0xffffffe0);
		} else if (conf->csi2_rx_type == MTK_SENINF_CSI2_RX_NCSI2) {
			val = mtk_seninf_input_read(input, SENINF_NCSI2_CTL);
			mtk_seninf_input_write(
				input, SENINF_NCSI2_CTL, val & 0xffffffe0);
		}

		if (!priv->is_testmode)
			phy_power_off(input->phy);
	}

	pm_runtime_put(priv->dev);
}

/* -----------------------------------------------------------------------------
 * V4L2 Controls
 */

static int seninf_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_seninf *priv = container_of(ctrl->handler,
					     struct mtk_seninf, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == TEST_PATTERN_ENABLED)
			priv->is_testmode = true;
		else if (ctrl->val == TEST_PATTERN_DISABLED)
			priv->is_testmode = false;
		else
			return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops seninf_ctrl_ops = {
	.s_ctrl = seninf_set_ctrl,
};

static const char *const seninf_test_pattern_menu[] = {
	"No test pattern",
	"Static horizontal color bars",
};

static int seninf_initialize_controls(struct mtk_seninf *priv)
{
	struct v4l2_ctrl_handler *handler;
	int ret;

	handler = &priv->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		return ret;

	v4l2_ctrl_new_std_menu_items(handler, &seninf_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(seninf_test_pattern_menu) - 1,
				     0, 0, seninf_test_pattern_menu);

	priv->is_testmode = false;

	if (handler->error) {
		ret = handler->error;
		dev_err(priv->dev,
			"Failed to init controls(%d)\n", ret);
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	priv->subdev.ctrl_handler = handler;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static const struct v4l2_mbus_framefmt mtk_seninf_default_fmt = {
	.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	.width = SENINF_DEFAULT_WIDTH,
	.height = SENINF_DEFAULT_HEIGHT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
};

static struct v4l2_mbus_framefmt *
seninf_get_pad_format(struct mtk_seninf *priv, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, u32 which)
{
	const struct mtk_seninf_conf *conf = priv->conf;

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&priv->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		if (pad < conf->nb_inputs)
			return &priv->inputs[pad].format;
		else
			return &priv->source_format;
	default:
		return NULL;
	}
}

static int seninf_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	u32 which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		format = seninf_get_pad_format(priv, cfg, i, which);
		*format = mtk_seninf_default_fmt;
	}

	return 0;
}

static int seninf_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	const struct mtk_seninf_format_info *fmtinfo;
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	const struct mtk_seninf_conf *conf = priv->conf;

	if (code->index >= ARRAY_SIZE(mtk_seninf_formats))
		return -EINVAL;

	fmtinfo = &mtk_seninf_formats[code->index];
	if (fmtinfo->flags & MTK_SENINF_FORMAT_INPUT_ONLY &&
	    code->pad >= conf->nb_inputs)
		return -EINVAL;

	code->code = fmtinfo->code;

	return 0;
}

static int seninf_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);

	fmt->format = *seninf_get_pad_format(priv, cfg, fmt->pad, fmt->which);

	return 0;
}

static int seninf_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	const struct mtk_seninf_format_info *fmtinfo;
	struct v4l2_mbus_framefmt *format;

	fmtinfo = mtk_seninf_format_info(fmt->format.code);
	if (!fmtinfo) {
		fmtinfo = &mtk_seninf_formats[0];
		fmt->format.code = fmtinfo->code;
	}

	format = seninf_get_pad_format(priv, cfg, fmt->pad, fmt->which);

	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = fmt->format.code;

	fmt->format = *format;

	/*
	 * Propagate the format to the corresponding source pad.
	 *
	 * TODO (?): We should disallow setting formats on the source pad
	 * completely, as the SENINF can't perform any processing. This would
	 * however break usage of the test pattern generator, as there would be
	 * no way to configure formats at all when no active input is selected.
	 */
	if (priv->inputs[fmt->pad].source_pad) {
		format = seninf_get_pad_format(priv, cfg, priv->inputs[fmt->pad].source_pad,
					       fmt->which);
		*format = fmt->format;
	}

	return 0;
}

static int seninf_get_routing(struct v4l2_subdev *sd,
			     struct v4l2_subdev_krouting *routing)
{
	struct mtk_seninf *priv = v4l2_get_subdevdata(sd);
	const struct mtk_seninf_conf *conf = priv->conf;
	struct v4l2_subdev_route *route = routing->routes;
	unsigned int sink, source;
	unsigned int num_routes = routing->num_routes;

	routing->num_routes = conf->nb_inputs * conf->nb_outputs;
	if (num_routes < routing->num_routes)
		return -ENOSPC;

	for (sink = 0; sink < conf->nb_inputs; ++sink) {
		for (source = 0; source < conf->nb_outputs; ++source) {
			route->sink_pad = sink;
			route->sink_stream = 0;
			route->source_pad = source + conf->nb_inputs;
			route->source_stream = 0;

			if (priv->inputs[sink].source_pad == route->source_pad)
				route->flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;

			route++;
		}
	}

	return 0;
}

static int seninf_set_routing(struct v4l2_subdev *sd,
			     struct v4l2_subdev_krouting *routing)
{
	struct mtk_seninf *priv = v4l2_get_subdevdata(sd);
	const struct mtk_seninf_conf *conf = priv->conf;
	struct v4l2_subdev_route *route = routing->routes;
	unsigned int i, k;
	int pad;

	for (k = 0; k < routing->num_routes; ++k) {
		struct mtk_seninf_input *input = &priv->inputs[route->sink_pad];

		if (route->sink_stream != 0 || route->source_stream != 0)
			return -EINVAL;

		pad = -1;
		for (i = 0; i < conf->nb_inputs; ++i) {
			if (priv->inputs[i].subdev == NULL)
				continue;
			if (priv->inputs[i].source_pad == route->source_pad) {
				pad = i;
				break;
			}
		}

		if (route->flags == V4L2_SUBDEV_ROUTE_FL_ACTIVE) {
			if ((input->source_pad != 0) || ((pad != -1) && (pad != route->sink_pad)))
				return -EMLINK;
			input->source_pad = route->source_pad;
		} else {
			if (input->source_pad == route->source_pad)
				input->source_pad = 0;
		}
	}

	return 0;
}

static int seninf_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct v4l2_subdev *source;
	int ret;

	if (!on) {
		if (priv->active_input && !priv->is_testmode) {
			source = priv->active_input->subdev;
			ret = v4l2_subdev_call(source, video, s_stream, 0);
			if (ret)
				dev_err(priv->dev,
					"failed to stop source %s: %d\n",
					source->entity.name, ret);
		}

		mtk_seninf_power_off(priv);
		return 0;
	}

	/*
	 * If no input is selected, or test mode is enabled, just enable the
	 * test pattern generator.
	 */
	if (!priv->active_input || priv->is_testmode)
		return seninf_enable_test_pattern(priv);

	/* Start the SENINF first and then the source. */
	ret = mtk_seninf_power_on(priv);
	if (ret < 0)
		return ret;

	source = priv->active_input->subdev;
	ret = v4l2_subdev_call(source, video, s_stream, 1);
	if (ret) {
		dev_err(priv->dev, "failed to start source %s: %d\n",
			source->entity.name, ret);
		mtk_seninf_power_off(priv);
		return ret;
	}

	return 0;
};

static const struct v4l2_subdev_pad_ops seninf_subdev_pad_ops = {
	.link_validate = v4l2_subdev_link_validate_default,
	.init_cfg = seninf_init_cfg,
	.set_fmt = seninf_set_fmt,
	.get_fmt = seninf_get_fmt,
	.get_routing = seninf_get_routing,
	.set_routing = seninf_set_routing,
	.enum_mbus_code = seninf_enum_mbus_code,
};

static const struct v4l2_subdev_video_ops seninf_subdev_video_ops = {
	.s_stream = seninf_s_stream,
};

static struct v4l2_subdev_core_ops seninf_subdev_core_ops = {
	.subscribe_event    = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
};

static struct v4l2_subdev_ops seninf_subdev_ops = {
	.core	= &seninf_subdev_core_ops,
	.video	= &seninf_subdev_video_ops,
	.pad	= &seninf_subdev_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media Entity Operations
 */

static int seninf_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct mtk_seninf *priv = v4l2_get_subdevdata(sd);

	if (!(local->flags & MEDIA_PAD_FL_SINK))
		return 0;

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (priv->active_input)
			return -EBUSY;

		priv->active_input = &priv->inputs[local->index];
	} else {
		priv->active_input = NULL;
	}

	return 0;
}

static bool seninf_has_route(struct media_entity *entity,
			unsigned int pad0, unsigned int pad1)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct mtk_seninf *priv = v4l2_get_subdevdata(sd);
	const struct mtk_seninf_conf *conf = priv->conf;
	unsigned int i;

	for (i = 0; i < conf->nb_inputs; ++i)
		if (pad0 == i && pad1 == priv->inputs[i].source_pad)
			return true;

	return false;
}

static const struct media_entity_operations seninf_media_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_setup = seninf_link_setup,
	.link_validate = v4l2_subdev_link_validate,
	.has_route = seninf_has_route,
};

/* -----------------------------------------------------------------------------
 * Async Subdev Notifier
 */

struct mtk_seninf_async_subdev {
	struct v4l2_async_subdev asd;
	struct mtk_seninf_input *input;
	unsigned int port;
};

static int mtk_seninf_fwnode_parse(struct device *dev,
				   struct v4l2_fwnode_endpoint *vep,
				   struct v4l2_async_subdev *asd)
{
	static const u32 port_to_seninf_id[] = {
		[CSI_PORT_0] = SENINF_1,
		[CSI_PORT_1] = SENINF_3,
		[CSI_PORT_2] = SENINF_5,
		[CSI_PORT_0B] = SENINF_2,
	};

	struct mtk_seninf *priv = dev_get_drvdata(dev);
	const struct mtk_seninf_conf *conf = priv->conf;
	struct mtk_seninf_async_subdev *s_asd =
		container_of(asd, struct mtk_seninf_async_subdev, asd);
	unsigned int port = vep->base.port;
	struct mtk_seninf_input *input;
	const char *phy_name;
	unsigned int i;

	/* Skip disabled fwnode. */
	if (!fwnode_device_is_available(asd->match.fwnode))
		return -ENOTCONN;

	if (port >= (conf->nb_inputs + conf->nb_outputs)) {
		dev_err(dev, "Invalid port %u\n", port);
		return -EINVAL;
	}

	s_asd->port = port;

	if (port >= conf->nb_inputs)
		return 0;

	if (vep->bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "Only CSI2 bus type is currently supported\n");
		return -EINVAL;
	}

	input = &priv->inputs[port];

	input->seninf = priv;
	input->pad = port;
	input->seninf_id = port_to_seninf_id[port];
	input->base = priv->base + 0x1000 * input->seninf_id;
	input->bus = vep->bus.mipi_csi2;
	input->source_pad = port + conf->nb_inputs;

	/*
	 * Select the PHY. SENINF2, SENINF3 and SENINF5 are hardwired to the
	 * CSI0B, CSI1 and CSI2 PHYs respectively. SENINF1 uses CSI0 or CSI0A
	 * depending on the clock and data lanes routing.
	 */
	switch (input->seninf_id) {
	case SENINF_1: {
	default:
		/*
		 * If all clock and data lanes are connected to the CSI0A half,
		 * use CSI0A. Otherwise use the full CSI0.
		 */
		input->phy_mode = SENINF_PHY_MODE_2D1C;

		for (i = 0; i < input->bus.num_data_lanes; ++i) {
			if (input->bus.data_lanes[i] > 2)
				input->phy_mode = SENINF_PHY_MODE_4D1C;
		}

		if (input->bus.clock_lane > 2)
			input->phy_mode = SENINF_PHY_MODE_4D1C;

		if (input->phy_mode == SENINF_PHY_MODE_4D1C) {
			input->phy = priv->phy[SENINF_PHY_CSI0];
			phy_name = "CSI0";
		} else {
			input->phy = priv->phy[SENINF_PHY_CSI0A];
			phy_name = "CSI0A";
		}

		break;
	}

	case SENINF_2:
		input->phy = priv->phy[SENINF_PHY_CSI0B];
		input->phy_mode = SENINF_PHY_MODE_2D1C;
		phy_name = "CSI0B";
		break;

	case SENINF_3:
		input->phy = priv->phy[SENINF_PHY_CSI1];
		input->phy_mode = SENINF_PHY_MODE_4D1C;
		phy_name = "CSI1";
		break;

	case SENINF_5:
		input->phy = priv->phy[SENINF_PHY_CSI2];
		input->phy_mode = SENINF_PHY_MODE_4D1C;
		phy_name = "CSI2";
		break;
	}

	s_asd->input = input;

	dev_dbg(dev, "%s: SENINF%u using %s (%uD1C mode, %u data lanes)\n",
		__func__, input->seninf_id + 1, phy_name,
		input->phy_mode == SENINF_PHY_MODE_4D1C ? 4 : 2,
		vep->bus.mipi_csi2.num_data_lanes);

	return 0;
}

static int mtk_seninf_notifier_bound(struct v4l2_async_notifier *notifier,
                                    struct v4l2_subdev *sd,
                                    struct v4l2_async_subdev *asd)
{
	struct mtk_seninf *priv = container_of(notifier, struct mtk_seninf, notifier);
	const struct mtk_seninf_conf *conf = priv->conf;
	struct mtk_seninf_async_subdev *s_asd =
		container_of(asd, struct mtk_seninf_async_subdev, asd);
	int ret;

	dev_dbg(priv->dev, "%s bound to SENINF port %u\n", sd->entity.name,
		s_asd->port);

	if (s_asd->port < conf->nb_inputs) {
		struct mtk_seninf_input *input = s_asd->input;

		input->subdev = sd;
		ret = v4l2_create_fwnode_links_to_pad(sd, &priv->pads[input->pad], 0);
	} else
		ret = v4l2_create_fwnode_links_to_pad(&priv->subdev,
					&sd->entity.pads[0],
					MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

	if (ret)
		dev_err(priv->dev, "Failed to create links between SENINF port %u and %s (%d)\n",
			s_asd->port, sd->entity.name, ret);

	return ret;
}

static int mtk_seninf_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mtk_seninf *priv = container_of(notifier, struct mtk_seninf, notifier);
	int ret;
	ret = v4l2_device_register_subdev_nodes(&priv->v4l2_dev);
	if (ret)
		dev_err(priv->dev, "Failed to register subdev nodes: %d\n", ret);

	return ret;
}

static const struct v4l2_async_notifier_operations mtk_seninf_async_ops = {
	.bound = mtk_seninf_notifier_bound,
	.complete = mtk_seninf_notifier_complete,
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int mtk_seninf_media_init(struct mtk_seninf *priv)
{
	struct media_pad *pads = priv->pads;
	const struct mtk_seninf_conf *conf = priv->conf;
	struct device *dev = priv->dev;
	struct media_device *media_dev = &priv->media_dev;
	u8 num_pads = conf->nb_outputs + conf->nb_inputs;
	unsigned int i;
	int ret;

	media_dev->dev = dev;
	strscpy(media_dev->model, conf->model, sizeof(media_dev->model));
	snprintf(media_dev->bus_info, sizeof(media_dev->bus_info),
		 "platform:%s", dev_name(dev));
	media_dev->hw_revision = 0;
	media_device_init(media_dev);

	ret = media_entity_pads_init(&priv->subdev.entity, num_pads, pads);
	if (ret)
		goto err_clean_media;

	for (i = 0; i < conf->nb_inputs; i++)
		pads[i].flags = MEDIA_PAD_FL_SINK;
	for (i = conf->nb_inputs; i < num_pads; i++)
		pads[i].flags = MEDIA_PAD_FL_SOURCE;

	return 0;
err_clean_media:
	media_device_cleanup(media_dev);

	return ret;
}

static int mtk_seninf_v4l2_async_register(struct mtk_seninf *priv)
{
	struct device *dev = priv->dev;
	const struct mtk_seninf_conf *conf = priv->conf;
	int ret;
	unsigned int i;

	v4l2_async_notifier_init(&priv->notifier);

	for (i = 0; i < (conf->nb_inputs + conf->nb_outputs); ++i) {
		ret = v4l2_async_notifier_parse_fwnode_endpoints_by_port(
			dev, &priv->notifier,
			sizeof(struct mtk_seninf_async_subdev), i,
			mtk_seninf_fwnode_parse);

		if (ret) {
			dev_err(dev, "Failed to parse endpoint at port %d, err: %d\n", i, ret);
			goto err_clean_notififer;
		}
	}

	priv->notifier.ops = &mtk_seninf_async_ops;
	ret = v4l2_async_notifier_register(&priv->v4l2_dev, &priv->notifier);
	if (ret)
		dev_err(dev, "Failed to register async notifier: %d\n", ret);

    return 0;

err_clean_notififer:
	v4l2_async_notifier_cleanup(&priv->notifier);

	return ret;
}

static int mtk_seninf_v4l2_register(struct mtk_seninf *priv)
{
	struct v4l2_subdev *sd = &priv->subdev;
	struct device *dev = priv->dev;
	int ret;

	/* Initialize media device & pads. */
	ret = mtk_seninf_media_init(priv);
	if (ret)
		return ret;

	/* Initialize & register v4l2 device. */
	priv->v4l2_dev.mdev = &priv->media_dev;

	ret = v4l2_device_register(dev, &priv->v4l2_dev);
	if (ret) {
		dev_err(dev, "Failed to register V4L2 device: %d\n", ret);
		goto err_clean_media;
	}

	/* Initialize & register subdev. */
	v4l2_subdev_init(sd, &seninf_subdev_ops);
	sd->dev = dev;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &seninf_media_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	strscpy(sd->name, dev_name(dev), V4L2_SUBDEV_NAME_SIZE);
	ret = seninf_initialize_controls(priv);
	if (ret) {
		dev_err(dev, "Failed to initialize controls: %d\n", ret);
		goto err_unreg_v4l2;
	}
	seninf_init_cfg(sd, NULL);
	v4l2_set_subdevdata(sd, priv);

	ret = v4l2_device_register_subdev(&priv->v4l2_dev, sd);
	if (ret) {
		dev_err(dev, "Failed to register subdev: %d\n", ret);
		goto err_free_handler;
	}

	/* Set up async device */
	ret = mtk_seninf_v4l2_async_register(priv);
	if (ret) {
		dev_err(dev, "Failed to register v4l2 async notifier: %d\n", ret);
		goto err_unreg_subdev;
	}

	/* Register media device */
	ret = media_device_register(&priv->media_dev);
	if (ret) {
		dev_err(dev, "failed to register media device: %d\n", ret);
		goto err_unreg_notifier;
	}

	return 0;

err_unreg_notifier:
	v4l2_async_notifier_unregister(&priv->notifier);
err_unreg_subdev:
	v4l2_device_unregister_subdev(sd);
err_free_handler:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
err_unreg_v4l2:
	v4l2_device_unregister(&priv->v4l2_dev);
err_clean_media:
	media_entity_cleanup(&sd->entity);
	media_device_cleanup(&priv->media_dev);

	return ret;
}

static int seninf_probe(struct platform_device *pdev)
{
	/* List of clocks and PHYs required by seninf. */
	static const char * const clk_names[] = {
		"cam_seninf", "top_mux_seninf"
	};
	static const char * const phy_names[] = {
		[SENINF_PHY_CSI0] = "csi0",
		[SENINF_PHY_CSI1] = "csi1",
		[SENINF_PHY_CSI2] = "csi2",
		[SENINF_PHY_CSI0A] = "csi0a",
		[SENINF_PHY_CSI0B] = "csi0b",
	};

	struct resource *res;
	struct mtk_seninf *priv;
	struct device *dev = &pdev->dev;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct mtk_seninf), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->conf = of_device_get_match_data(dev);

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	for (i = 0; i < priv->conf->nb_phy; ++i) {
		struct phy *phy;

		phy = devm_phy_get(dev, phy_names[i]);
		if (IS_ERR(phy)) {
			dev_err(dev, "failed to get phy:%ld\n", PTR_ERR(phy));
			return PTR_ERR(phy);
		}

		priv->phy[i] = phy;
	}

	priv->num_clks = ARRAY_SIZE(clk_names);
	priv->clks = devm_kcalloc(dev, priv->num_clks,
				  sizeof(*priv->clks), GFP_KERNEL);
	if (!priv->clks)
		return -ENOMEM;

	for (i = 0; i < priv->num_clks; ++i)
		priv->clks[i].id = clk_names[i];

	ret = devm_clk_bulk_get(dev, priv->num_clks, priv->clks);
	if (ret) {
		dev_err(dev, "failed to get seninf clock:%d\n", ret);
		return ret;
	}

	ret = mtk_seninf_v4l2_register(priv);
	if (!ret)
		pm_runtime_enable(dev);

	return ret;
}

static int seninf_pm_suspend(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);

	dev_dbg(dev, "seninf runtime suspend\n");
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);

	return 0;
}

static int seninf_pm_resume(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "seninf runtime resume\n");
	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret) {
		dev_err(dev, "failed to enable clock:%d\n", ret);
		return ret;
	}

	return 0;
}

static const struct dev_pm_ops runtime_pm_ops = {
	SET_RUNTIME_PM_OPS(seninf_pm_suspend, seninf_pm_resume, NULL)
};

static int seninf_remove(struct platform_device *pdev)
{
	struct mtk_seninf *priv = dev_get_drvdata(&pdev->dev);

	media_device_unregister(&priv->media_dev);
	media_device_cleanup(&priv->media_dev);
	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	media_entity_cleanup(&priv->subdev.entity);
	v4l2_device_unregister(&priv->v4l2_dev);

	pm_runtime_disable(priv->dev);

	return 0;
}

static const struct of_device_id mtk_seninf_of_match[] = {
	{
		.compatible = "mediatek,mt8183-seninf",
		.data = &seninf_8183_conf,
	},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_seninf_of_match);

static struct platform_driver seninf_pdrv = {
	.driver	= {
		.name	= "mtk-seninf",
		.pm  = &runtime_pm_ops,
		.of_match_table = of_match_ptr(mtk_seninf_of_match),
	},
	.probe	= seninf_probe,
	.remove	= seninf_remove,
};

module_platform_driver(seninf_pdrv);

MODULE_DESCRIPTION("MTK sensor interface driver");
MODULE_AUTHOR("Louis Kuo <louis.kuo@mediatek.com>");
MODULE_LICENSE("GPL v2");
