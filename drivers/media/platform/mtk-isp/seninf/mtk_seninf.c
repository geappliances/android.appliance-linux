// SPDX-License-Identifier: GPL-2.0

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <linux/phy/phy.h>
#include "mtk_seninf_reg.h"

#define SENINF_TIMESTAMP_STEP		0x67
#define SENINF_SETTLE_DELAY		0x15
#define SENINF_HS_TRAIL_PARAMETER	0x8

#define SENINF_NUM_INPUTS		4
#define SENINF_NUM_PADS			5

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

enum SENINF_ID {
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

#define SENINF_BITS(base, reg, field, val) do { \
		u32 __iomem *__p = (base) + (reg); \
		u32 __v = *__p; \
		__v &= ~reg##_##field##_MASK; \
		__v |= ((val) << reg##_##field##_SHIFT); \
		*__p = __v; \
	} while (0)

enum mtk_seninf_format_flag {
	MTK_SENINF_FORMAT_BAYER = BIT(0),
	MTK_SENINF_FORMAT_DPCM = BIT(1),
	MTK_SENINF_FORMAT_JPEG = BIT(2),
	MTK_SENINF_FORMAT_INPUT_ONLY = BIT(3),
};

struct mtk_seninf_format_info {
	u32 code;
	u32 flags;
};

struct mtk_seninf_input {
	enum mtk_seninf_port port;
	enum SENINF_ID seninf;
	void __iomem *base;

	struct phy *phy;
	enum mtk_seninf_phy_mode phy_mode;

	struct v4l2_fwnode_bus_mipi_csi2 bus;

	struct v4l2_subdev *subdev;
	struct v4l2_mbus_framefmt format;
};

struct mtk_seninf {
	struct device *dev;
	struct phy *phy[5];
	unsigned int num_clks;
	struct clk_bulk_data *clks;
	void __iomem *base;

	struct v4l2_subdev subdev;
	struct media_pad pads[SENINF_NUM_PADS];
	struct v4l2_async_notifier notifier;

	struct v4l2_ctrl_handler ctrl_handler;

	struct v4l2_mbus_framefmt source_format;

	struct mtk_seninf_input inputs[SENINF_NUM_INPUTS];
	struct mtk_seninf_input *active_input;

	unsigned int mux_sel;
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

static void mtk_seninf_set_mux(struct mtk_seninf *priv,
			       struct mtk_seninf_input *input)
{
	const struct mtk_seninf_format_info *fmtinfo;
	unsigned int mux = priv->mux_sel;
	void __iomem *pseninf_top = priv->base;
	void __iomem *pseninf = priv->base + 0x1000 * mux;
	unsigned int val;
	unsigned int pix_sel_ext;
	unsigned int pix_sel;
	unsigned int hs_pol = 0;
	unsigned int vs_pol = 0;
	unsigned int pixel_mode = TWO_PIXEL_MODE;

	fmtinfo = mtk_seninf_format_info(input->format.code);

	/* Enable mux */
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_MUX_EN, 1);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_SRC_SEL,
		    SENINF_MIPI_SENSOR);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL_EXT, SENINF_SRC_SEL_EXT,
		    SENINF_NORMAL_MODEL);

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

	SENINF_BITS(pseninf, SENINF_MUX_CTRL_EXT, SENINF_PIX_SEL_EXT,
		    pix_sel_ext);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_PIX_SEL, pix_sel);

	if (!(fmtinfo->flags & MTK_SENINF_FORMAT_JPEG)) {
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 2);
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FLUSH_EN, 0x1b);
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_PUSH_EN, 0x1f);
	} else {
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 0);
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FLUSH_EN, 0x18);
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_PUSH_EN, 0x1e);
	}

	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_HSYNC_POL, hs_pol);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_VSYNC_POL, vs_pol);

	val = readl(pseninf + SENINF_MUX_CTRL);
	writel(val | 0x00000003, pseninf + SENINF_MUX_CTRL);
	writel(val & 0xfffffffc, pseninf + SENINF_MUX_CTRL);

	/* Set top mux */
	val = (readl(pseninf_top + SENINF_TOP_MUX_CTRL) &
		(~(0xf << (mux * 4))))	| ((input->seninf & 0xF) << (mux * 4));
	writel(val, pseninf_top + SENINF_TOP_MUX_CTRL);
	/* HQ */
	writel(0xc2000, pseninf + SENINF_MUX_SPARE);

	/* HQ */
	writel(0x76543010, pseninf_top + SENINF_TOP_CAM_MUX_CTRL);
}

static void mtk_seninf_setup_phy(struct mtk_seninf *priv)
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

		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    DPHY_MODE, dphy_mode);
		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    CK_SEL_1, input_a->bus.clock_lane);
		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    CK_SEL_2, csi0b_clock);
		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    PHY_SENINF_LANE_MUX_CSI0_EN, 1);
	}

	/* CSI1 */
	if (priv->inputs[CSI_PORT_1].phy_mode) {
		struct mtk_seninf_input *input = &priv->inputs[CSI_PORT_1];

		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI1,
			    DPHY_MODE, 0 /* 4D1C */);
		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI1,
			    CK_SEL_1, input->bus.clock_lane);
		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI1,
			    PHY_SENINF_LANE_MUX_CSI1_EN, 1);
	}

	/* CSI2 */
	if (priv->inputs[CSI_PORT_2].phy_mode) {
		struct mtk_seninf_input *input = &priv->inputs[CSI_PORT_2];

		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI2,
			    DPHY_MODE, 0 /* 4D1C */);
		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI2,
			    CK_SEL_1, input->bus.clock_lane);
		SENINF_BITS(priv->base, SENINF_TOP_PHY_SENINF_CTL_CSI2,
			    PHY_SENINF_LANE_MUX_CSI2_EN, 1);
	}
}

static void mtk_seninf_rx_config(struct mtk_seninf *priv,
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

	SENINF_BITS(input->base, MIPI_RX_CON24_CSI0,
		    CSI0_BIST_LN0_MUX, lanes[0]);
	SENINF_BITS(input->base, MIPI_RX_CON24_CSI0,
		    CSI0_BIST_LN1_MUX, lanes[1]);
	SENINF_BITS(input->base, MIPI_RX_CON24_CSI0,
		    CSI0_BIST_LN2_MUX, lanes[2]);
	SENINF_BITS(input->base, MIPI_RX_CON24_CSI0,
		    CSI0_BIST_LN3_MUX, lanes[3]);
}

static void mtk_seninf_set_csi_mipi(struct mtk_seninf *priv,
				    struct mtk_seninf_input *input)
{
	const struct mtk_seninf_format_info *fmtinfo;
	unsigned int dpcm;
	unsigned int data_lane_num = input->bus.num_data_lanes;
	unsigned int data_header_order = 1;
	unsigned int val = 0;

	fmtinfo = mtk_seninf_format_info(input->format.code);

	/* Configure timestamp */
	writel(SENINF_TIMESTAMP_STEP, input->base + SENINF_TG1_TM_STP);

	/* HQ */
	writel(0x0, input->base + SENINF_TG1_PH_CNT);
	writel(0x10001, input->base + SENINF_TG1_SEN_CK);

	/* First Enable Sensor interface and select pad (0x1a04_0200) */
	SENINF_BITS(input->base, SENINF_CTRL, SENINF_EN, 1);
	SENINF_BITS(input->base, SENINF_CTRL, PAD2CAM_DATA_SEL, SENINF_PAD_10BIT);
	SENINF_BITS(input->base, SENINF_CTRL, SENINF_SRC_SEL, 0);
	SENINF_BITS(input->base, SENINF_CTRL_EXT, SENINF_CSI2_IP_EN, 1);
	SENINF_BITS(input->base, SENINF_CTRL_EXT, SENINF_NCSI2_IP_EN, 0);

	/* DPCM Enable */
	dpcm = fmtinfo->flags & MTK_SENINF_FORMAT_DPCM ? 0x2a : 0;
	val = 1 << ((dpcm == 0x2a) ? 15 : ((dpcm & 0xF) + 7));
	writel(val, input->base + SENINF_CSI2_DPCM);

	/* HQ */
	// writel(0x1, input->base + SENINF_CSI2_DPCM);

	/* Settle delay */
	SENINF_BITS(input->base, SENINF_CSI2_LNRD_TIMING,
		    DATA_SETTLE_PARAMETER, SENINF_SETTLE_DELAY);

	/* HQ */
	writel(0x10, input->base + SENINF_CSI2_LNRC_FSM);

	/* CSI2 control */
	val = readl(input->base + SENINF_CSI2_CTL) | (data_header_order << 16) |
		0x10 | ((1 << data_lane_num) - 1);
	writel(val, input->base + SENINF_CSI2_CTL);

	SENINF_BITS(input->base, SENINF_CSI2_RESYNC_MERGE_CTL,
		    BYPASS_LANE_RESYNC, 0);
	SENINF_BITS(input->base, SENINF_CSI2_RESYNC_MERGE_CTL, CDPHY_SEL, 0);
	SENINF_BITS(input->base, SENINF_CSI2_RESYNC_MERGE_CTL,
		    CPHY_LANE_RESYNC_CNT, 3);
	SENINF_BITS(input->base, SENINF_CSI2_MODE, CSR_CSI2_MODE, 0);
	SENINF_BITS(input->base, SENINF_CSI2_MODE, CSR_CSI2_HEADER_LEN, 0);
	SENINF_BITS(input->base, SENINF_CSI2_DPHY_SYNC, SYNC_SEQ_MASK_0, 0xff00);
	SENINF_BITS(input->base, SENINF_CSI2_DPHY_SYNC, SYNC_SEQ_PAT_0, 0x001d);

	SENINF_BITS(input->base, SENINF_CSI2_CTL, CLOCK_HS_OPTION, 0);
	SENINF_BITS(input->base, SENINF_CSI2_CTL, HSRX_DET_EN, 0);
	SENINF_BITS(input->base, SENINF_CSI2_CTL, HS_TRAIL_EN, 1);
	SENINF_BITS(input->base, SENINF_CSI2_HS_TRAIL, HS_TRAIL_PARAMETER,
		    SENINF_HS_TRAIL_PARAMETER);

	/* Set debug port to output packet number */
	SENINF_BITS(input->base, SENINF_CSI2_DGB_SEL, DEBUG_EN, 1);
	SENINF_BITS(input->base, SENINF_CSI2_DGB_SEL, DEBUG_SEL, 0x1a);

	/* HQ */
	writel(0xfffffffe, input->base + SENINF_CSI2_SPARE0);

	/* Enable CSI2 IRQ mask */
	/* Turn on all interrupt */
	writel(0xffffffff, input->base + SENINF_CSI2_INT_EN);
	/* Write clear CSI2 IRQ */
	writel(0xffffffff, input->base + SENINF_CSI2_INT_STATUS);
	/* Enable CSI2 Extend IRQ mask */
	/* Turn on all interrupt */
	SENINF_BITS(input->base, SENINF_CTRL, CSI2_SW_RST, 1);
	udelay(1);
	SENINF_BITS(input->base, SENINF_CTRL, CSI2_SW_RST, 0);
}

static int seninf_enable_test_pattern(struct mtk_seninf *priv)
{
	const struct mtk_seninf_format_info *fmtinfo;
	void __iomem *pseninf = priv->base;
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

	SENINF_BITS(pseninf, SENINF_TOP_CTRL, MUX_LP_MODE, 0);

	SENINF_BITS(pseninf, SENINF_TOP_CTRL, SENINF_PCLK_EN, 1);
	SENINF_BITS(pseninf, SENINF_TOP_CTRL, SENINF2_PCLK_EN, 1);

	SENINF_BITS(pseninf, SENINF_CTRL, SENINF_EN, 1);
	SENINF_BITS(pseninf, SENINF_CTRL, SENINF_SRC_SEL, 1);
	SENINF_BITS(pseninf, SENINF_CTRL_EXT, SENINF_TESTMDL_IP_EN, 1);

	SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_EN, 1);
	SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_PAT, 0xc);
	SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_VSYNC, 4);
	SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_DUMMYPXL, 0x28);

	if (fmtinfo->flags & MTK_SENINF_FORMAT_BAYER)
		SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_FMT, 0x0);
	else
		SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_FMT, 0x1);

	switch (priv->source_format.code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
		writel((priv->source_format.height + 8) << 16 |
		       priv->source_format.width * 2,
		       pseninf + SENINF_TG1_TM_SIZE);
		break;
	default:
		writel((priv->source_format.height + 8) << 16 |
		       priv->source_format.width,
		       pseninf + SENINF_TG1_TM_SIZE);
		break;
	}

	writel(0x8, pseninf + SENINF_TG1_TM_CLK);
	writel(0x1, pseninf + SENINF_TG1_TM_STP);

	/* Set top mux */
	val = (readl(pseninf + SENINF_TOP_MUX_CTRL) & (~(0xf << (mux * 4)))) |
	      ((seninf & 0xf) << (mux * 4));
	writel(val, pseninf + SENINF_TOP_MUX_CTRL);

	/* TODO : if mux != 0 => use pseninf + 0x1000 * mux */
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_MUX_EN, 1);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL_EXT, SENINF_SRC_SEL_EXT,
		    SENINF_TEST_MODEL);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_SRC_SEL, 1);

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
	SENINF_BITS(pseninf, SENINF_MUX_CTRL_EXT, SENINF_PIX_SEL_EXT,
		    pix_sel_ext);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_PIX_SEL, pix_sel);

	SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_PUSH_EN,
		    0x1f);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FLUSH_EN,
		    0x1b);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FULL_WR_EN,
		    2);

	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_HSYNC_POL, hs_pol);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_VSYNC_POL, vs_pol);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_HSYNC_MASK, 1);

	writel(SENINF_IRQ_CLR_SEL | SENINF_ALL_ERR_IRQ_EN,
	       pseninf + SENINF_MUX_INTEN);

	writel(0x3 | readl(pseninf + SENINF_MUX_CTRL),
	       pseninf + SENINF_MUX_CTRL);
	udelay(1);
	writel(~(0x3) & readl(pseninf + SENINF_MUX_CTRL),
	       pseninf + SENINF_MUX_CTRL);

	writel(0x76543010, pseninf + SENINF_TOP_CAM_MUX_CTRL);

	dev_dbg(priv->dev, "%s: OK\n", __func__);
	return 0;
}

static int mtk_seninf_power_on(struct mtk_seninf *priv)
{
	struct mtk_seninf_input *input = priv->active_input;
	int ret;

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to pm_runtime_get_sync: %d\n", ret);
		pm_runtime_put_noidle(priv->dev);
		return ret;
	}

	mtk_seninf_setup_phy(priv);

	phy_power_on(input->phy);

	mtk_seninf_rx_config(priv, input);
	mtk_seninf_set_csi_mipi(priv, input);
	mtk_seninf_set_mux(priv, input);

	return 0;
}

static void mtk_seninf_power_off(struct mtk_seninf *priv)
{
	if (priv->active_input) {
		struct mtk_seninf_input *input = priv->active_input;

		/* Disable CSI2(2.5G) first */
		writel(readl(input->base + SENINF_CSI2_CTL) & 0xffffffe0,
		       input->base + SENINF_CSI2_CTL);

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
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&priv->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		if (pad < ARRAY_SIZE(priv->inputs))
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

	if (code->index >= ARRAY_SIZE(mtk_seninf_formats))
		return -EINVAL;

	fmtinfo = &mtk_seninf_formats[code->index];
	if (fmtinfo->flags & MTK_SENINF_FORMAT_INPUT_ONLY &&
	    code->pad >= SENINF_NUM_INPUTS)
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
	*format = fmt->format;

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

static const struct media_entity_operations seninf_media_ops = {
	.link_setup = seninf_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Async Subdev Notifier
 */

struct mtk_seninf_async_subdev {
	struct v4l2_async_subdev asd;
	struct mtk_seninf_input *input;
};

static int mtk_seninf_notifier_bound(
			struct v4l2_async_notifier *notifier,
			struct v4l2_subdev *sd,
			struct v4l2_async_subdev *asd)
{
	struct mtk_seninf *priv =
		container_of(notifier, struct mtk_seninf, notifier);
	struct mtk_seninf_async_subdev *s_asd =
		container_of(asd, struct mtk_seninf_async_subdev, asd);
	struct mtk_seninf_input *input = s_asd->input;
	int ret;

	dev_dbg(priv->dev, "%s bound to SENINF%u\n", sd->entity.name,
		input->seninf + 1);

	input->subdev = sd;

	ret = media_create_pad_link(&sd->entity, 0, &priv->subdev.entity,
				    input->port, 0);
	if (ret) {
		dev_err(priv->dev, "failed to create link for %s\n",
			sd->entity.name);
		return ret;
	}

	return 0;
}

static const struct v4l2_async_notifier_operations mtk_seninf_async_ops = {
	.bound = mtk_seninf_notifier_bound,
};

static int mtk_seninf_fwnode_parse(struct device *dev,
				   struct v4l2_fwnode_endpoint *vep,
				   struct v4l2_async_subdev *asd)
{
	static const u32 port_to_seninf[] = {
		[CSI_PORT_0] = SENINF_1,
		[CSI_PORT_1] = SENINF_3,
		[CSI_PORT_2] = SENINF_5,
		[CSI_PORT_0B] = SENINF_2,
	};

	struct mtk_seninf *priv = dev_get_drvdata(dev);
	struct mtk_seninf_async_subdev *s_asd =
		container_of(asd, struct mtk_seninf_async_subdev, asd);
	unsigned int port = vep->base.port;
	struct mtk_seninf_input *input;
	const char *phy_name;
	unsigned int i;

	/* Skip disabled sensors. */
	if (!fwnode_device_is_available(asd->match.fwnode))
		return -ENOTCONN;

	if (port >= ARRAY_SIZE(priv->inputs)) {
		dev_err(dev, "Invalid port %u\n", port);
		return -EINVAL;
	}

	if (vep->bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "Only CSI2 bus type is currently supported\n");
		return -EINVAL;
	}

	input = &priv->inputs[port];

	input->port = port;
	input->seninf = port_to_seninf[port];
	input->base = priv->base + 0x1000 * input->seninf;
	input->bus = vep->bus.mipi_csi2;

	/*
	 * Select the PHY. SENINF2, SENINF3 and SENINF5 are hardwired to the
	 * CSI1, CSI2 and CSI0B PHYs respectively. SENINF1 uses CSI0 or CSI0A
	 * depending on the clock and data lanes routing.
	 */
	switch (input->seninf) {
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
		__func__, input->seninf + 1, phy_name,
		input->phy_mode == SENINF_PHY_MODE_4D1C ? 4 : 2,
		vep->bus.mipi_csi2.num_data_lanes);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int mtk_seninf_media_register(struct mtk_seninf *priv)
{
	struct v4l2_subdev *sd = &priv->subdev;
	struct media_pad *pads = priv->pads;
	struct device *dev = priv->dev;
	unsigned int i;
	int ret;

	v4l2_subdev_init(sd, &seninf_subdev_ops);

	ret = seninf_initialize_controls(priv);
	if (ret) {
		dev_err(dev, "Failed to initialize controls\n");
		return -EINVAL;
	}

	sd->flags |= (V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);

	priv->subdev.dev = dev;
	snprintf(sd->name, V4L2_SUBDEV_NAME_SIZE, "%s",
		 dev_name(dev));
	v4l2_set_subdevdata(sd, priv);

	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &seninf_media_ops;

	for (i = 0; i < SENINF_NUM_INPUTS; i++)
		pads[i].flags = MEDIA_PAD_FL_SINK;
	for (i = SENINF_NUM_INPUTS; i < SENINF_NUM_PADS; i++)
		pads[i].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, SENINF_NUM_PADS, pads);
	if (ret < 0)
		goto err_free_handler;

	seninf_init_cfg(sd, NULL);

	v4l2_async_notifier_init(&priv->notifier);

	for (i = 0; i < SENINF_NUM_INPUTS; ++i) {
		ret = v4l2_async_notifier_parse_fwnode_endpoints_by_port(
			dev, &priv->notifier,
			sizeof(struct mtk_seninf_async_subdev), i,
			mtk_seninf_fwnode_parse);
		if (ret < 0)
			goto err_clean_entity;
	}

	priv->subdev.subdev_notifier = &priv->notifier;
	priv->notifier.ops = &mtk_seninf_async_ops;
	ret = v4l2_async_subdev_notifier_register(sd, &priv->notifier);
	if (ret < 0) {
		dev_err(dev, "v4l2 async notifier register failed\n");
		goto err_clean_notififer;
	}

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_notififer;
	}
	return 0;

err_clean_notififer:
	v4l2_async_notifier_cleanup(&priv->notifier);
err_clean_entity:
	media_entity_cleanup(&sd->entity);
err_free_handler:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

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

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	for (i = 0; i < ARRAY_SIZE(priv->phy); ++i) {
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

	/*
	 * TODO: Support multiple source connections. For now only the first
	 * source port is used, hardcode mux_sel to 0.
	 */
	priv->mux_sel = 0;

	ret = mtk_seninf_media_register(priv);
	if (!ret) /* register success */
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
	struct v4l2_subdev *subdev = &priv->subdev;

	media_entity_cleanup(&subdev->entity);
	v4l2_async_unregister_subdev(subdev);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	pm_runtime_disable(priv->dev);

	return 0;
}

static const struct of_device_id mtk_seninf_of_match[] = {
	{.compatible = "mediatek,mt8183-seninf"},
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
