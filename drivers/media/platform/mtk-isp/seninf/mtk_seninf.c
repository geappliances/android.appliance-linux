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

#define NUM_PADS			12
#define NUM_SENSORS			4
#define CAM_MUX_IDX_MIN		NUM_SENSORS
#define DEFAULT_WIDTH			1920
#define DEFAULT_HEIGHT			1080

#define PAD_10BIT			0

#define TEST_MODEL          0
#define NORMAL_MODEL        1
#define SENINF_ALL_ERR_IRQ_EN	0x7F
#define SENINF_IRQ_CLR_SEL	0x80000000

#define MIPI_SENSOR			0x8

enum TEST_MODE {
	TEST_GEN_PATTERN = 0x0,
	TEST_DUMP_DEBUG_INFO,
};

enum CFG_CSI_PORT {
	CFG_CSI_PORT_0 = 0x0,	/* 4D1C */
	CFG_CSI_PORT_1,		/* 4D1C */
	CFG_CSI_PORT_2,		/* 4D1C */
	CFG_CSI_PORT_0A,	/* 2D1C */
	CFG_CSI_PORT_0B,	/* 2D1C */
	CFG_CSI_PORT_MAX_NUM,
	CFG_CSI_PORT_NONE	/*for non-MIPI sensor */
};

enum PIXEL_MODE {
	ONE_PIXEL_MODE  = 0x0,
	TWO_PIXEL_MODE  = 0x1,
	FOUR_PIXEL_MODE = 0x2,
};

enum SENINF_ID {
	SENINF_1 = 0x0,
	SENINF_2 = 0x1,
	SENINF_3 = 0x2,
	SENINF_4 = 0x3,
	SENINF_5 = 0x4,
	SENINF_NUM,
};

enum IMAGE_FMT {
	RAW_8BIT_FMT        = 0x0,
	RAW_10BIT_FMT       = 0x1,
	RAW_12BIT_FMT       = 0x2,
	YUV422_FMT          = 0x3,
	RAW_14BIT_FMT       = 0x4,
	RGB565_MIPI_FMT     = 0x5,
	RGB888_MIPI_FMT     = 0x6,
	JPEG_FMT            = 0x7
};

#define SENINF_BITS(base, reg, field, val) do { \
		u32 __iomem *__p = (base) + (reg); \
		u32 __v = *__p; \
		__v &= ~reg##_##field##_MASK; \
		__v |= ((val) << reg##_##field##_SHIFT); \
		*__p = __v; \
	} while (0)

struct mtk_seninf_sensor_cfg {
	unsigned char clock_lane;
	unsigned short num_data_lanes;
};

struct mtk_seninf {
	struct v4l2_subdev subdev;
	struct v4l2_async_notifier notifier;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev_format fmt[NUM_PADS];
	struct device *dev;
	struct media_pad pads[NUM_PADS];
	struct mtk_seninf_sensor_cfg sensor[NUM_SENSORS];
	struct phy *dphy;
	unsigned int num_clks;
	struct clk_bulk_data *clks;
	void __iomem *base;
	void __iomem *rx;
	unsigned int port;
	unsigned int mux_sel;
};

static inline int is_4d1c(unsigned int port)
{
	return port < CFG_CSI_PORT_0A;
}

static inline int is_cdphy_combo(unsigned int port)
{
	return port == CFG_CSI_PORT_0A ||
		port == CFG_CSI_PORT_0B ||
		port == CFG_CSI_PORT_0;
}

inline struct mtk_seninf *sd_to_mtk_seninf(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_seninf, subdev);
}

static unsigned int mtk_seninf_get_dpcm(struct mtk_seninf *priv)
{
	unsigned int dpcm;

	switch (priv->fmt[priv->port].format.code) {
	case MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8:
		dpcm = 0x2a;
		break;
	default:
		dpcm = 0;
		break;
	}

	return dpcm;
}

static unsigned int mtk_seninf_map_fmt(struct mtk_seninf *priv)
{
	int fmtidx = RAW_10BIT_FMT;

	switch (priv->fmt[priv->port].format.code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		fmtidx = RAW_8BIT_FMT;
		break;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		fmtidx = RAW_10BIT_FMT;
		break;
	case MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8:
		fmtidx = RAW_8BIT_FMT;
		break;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		fmtidx = RAW_12BIT_FMT;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
		fmtidx = YUV422_FMT;
		break;
	case MEDIA_BUS_FMT_JPEG_1X8:
	case MEDIA_BUS_FMT_S5C_UYVY_JPEG_1X8:
		fmtidx = JPEG_FMT;
		break;
	}

	return fmtidx;
}

static u32 mtk_seninf_csi_port_to_seninf(u32 port)
{
	static const u32 port_to_seninf[] = {
		[CFG_CSI_PORT_0] = SENINF_1,
		[CFG_CSI_PORT_1] = SENINF_3,
		[CFG_CSI_PORT_2] = SENINF_5,
		[CFG_CSI_PORT_0A] = SENINF_1,
		[CFG_CSI_PORT_0B] = SENINF_2,
	};
	if (WARN_ON(port >= ARRAY_SIZE(port_to_seninf)))
		return -EINVAL;

	return port_to_seninf[port];
}

static void mtk_seninf_set_mux(struct mtk_seninf *priv,
			       unsigned int seninf)
{
	unsigned int mux = priv->mux_sel;
	void __iomem *pseninf_top = priv->base;
	void __iomem *pseninf = priv->base + 0x1000 * mux;
	unsigned int val;
	unsigned int pix_sel_ext;
	unsigned int pix_sel;
	unsigned int hs_pol = 0;
	unsigned int vs_pol = 0;
	unsigned int pixel_mode = TWO_PIXEL_MODE;
	unsigned int input_data_type;

	/* Enable mux */
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_MUX_EN, 1);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_SRC_SEL, MIPI_SENSOR);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL_EXT, SENINF_SRC_SEL_EXT,
		    NORMAL_MODEL);
	input_data_type = mtk_seninf_map_fmt(priv);

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

	if (input_data_type != JPEG_FMT) {
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 2);
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FLUSH_EN, 0x1B);
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_PUSH_EN, 0x1F);
	} else {
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 0);
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FLUSH_EN, 0x18);
		SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_PUSH_EN, 0x1E);
	}

	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_HSYNC_POL, hs_pol);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_VSYNC_POL, vs_pol);

	val = readl(pseninf + SENINF_MUX_CTRL);
	writel(val | 0x00000003, pseninf + SENINF_MUX_CTRL);
	writel(val & 0xFFFFFFFC, pseninf + SENINF_MUX_CTRL);

	/* Set top mux */
	val = (readl(pseninf_top + SENINF_TOP_MUX_CTRL) &
		(~(0xF << (mux * 4))))	| ((seninf & 0xF) << (mux * 4));
	writel(val, pseninf_top + SENINF_TOP_MUX_CTRL);
}

static void mtk_seninf_rx_config(struct mtk_seninf *priv,
				 unsigned int seninf)
{
	unsigned int port = priv->port;
	void __iomem *pseninf = priv->base + 0x1000 * seninf;

	if (is_4d1c(port)) {
		SENINF_BITS(pseninf, MIPI_RX_CON24_CSI0,
			    CSI0_BIST_LN0_MUX, 1);
		SENINF_BITS(pseninf, MIPI_RX_CON24_CSI0,
			    CSI0_BIST_LN1_MUX, 2);
		SENINF_BITS(pseninf, MIPI_RX_CON24_CSI0,
			    CSI0_BIST_LN2_MUX, 0);
		SENINF_BITS(pseninf, MIPI_RX_CON24_CSI0,
			    CSI0_BIST_LN3_MUX, 3);
	} else {
		SENINF_BITS(pseninf, MIPI_RX_CON24_CSI0,
			    CSI0_BIST_LN0_MUX, 0);
		SENINF_BITS(pseninf, MIPI_RX_CON24_CSI0,
			    CSI0_BIST_LN1_MUX, 1);
		SENINF_BITS(pseninf, MIPI_RX_CON24_CSI0,
			    CSI0_BIST_LN2_MUX, 2);
		SENINF_BITS(pseninf, MIPI_RX_CON24_CSI0,
			    CSI0_BIST_LN3_MUX, 3);
	}
}

static void mtk_seninf_set_csi_mipi(struct mtk_seninf *priv,
				    unsigned int seninf)
{
	void __iomem *seninf_base = priv->base;
	void __iomem *pseninf = priv->base + 0x1000 * seninf;
	unsigned int dpcm = mtk_seninf_get_dpcm(priv);
	unsigned int data_lane_num = priv->sensor[priv->port].num_data_lanes;
	unsigned int cal_sel;
	unsigned int data_header_order = 1;
	unsigned int val = 0;

	dev_dbg(priv->dev, "IS_4D1C %d port %d\n",
		is_4d1c(priv->port), priv->port);

	switch (priv->port) {
	case CFG_CSI_PORT_1:
		cal_sel = 1;
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI1,
			    DPHY_MODE, 0);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI1,
			    CK_SEL_1, 2);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI1,
			    PHY_SENINF_LANE_MUX_CSI1_EN, 1);
		break;
	case CFG_CSI_PORT_2:
		cal_sel = 2;
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI2,
			    DPHY_MODE, 0);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI2,
			    CK_SEL_1, 2);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI2,
			    PHY_SENINF_LANE_MUX_CSI2_EN, 1);
		break;
	case CFG_CSI_PORT_0:
		cal_sel = 0;
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    DPHY_MODE, 0);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    CK_SEL_1, 2);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    PHY_SENINF_LANE_MUX_CSI0_EN, 1);
		break;
	case CFG_CSI_PORT_0A:
	case CFG_CSI_PORT_0B:
		cal_sel = 0;
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    DPHY_MODE, 1);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    CK_SEL_1, 1);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    CK_SEL_2, 1);
		SENINF_BITS(seninf_base, SENINF_TOP_PHY_SENINF_CTL_CSI0,
			    PHY_SENINF_LANE_MUX_CSI0_EN, 1);
		break;
	}

	/* First Enable Sensor interface and select pad (0x1a04_0200) */
	SENINF_BITS(pseninf, SENINF_CTRL, SENINF_EN, 1);
	SENINF_BITS(pseninf, SENINF_CTRL, PAD2CAM_DATA_SEL, PAD_10BIT);
	SENINF_BITS(pseninf, SENINF_CTRL, SENINF_SRC_SEL, 0);
	SENINF_BITS(pseninf, SENINF_CTRL_EXT, SENINF_CSI2_IP_EN, 1);
	SENINF_BITS(pseninf, SENINF_CTRL_EXT, SENINF_NCSI2_IP_EN, 0);

	/* DPCM Enable */
	val = 1 << ((dpcm == 0x2a) ? 15 : ((dpcm & 0xF) + 7));
	writel(val, pseninf + SENINF_CSI2_DPCM);

	/* Settle delay */
	SENINF_BITS(pseninf, SENINF_CSI2_LNRD_TIMING,
		    DATA_SETTLE_PARAMETER, SENINF_SETTLE_DELAY);
	/* CSI2 control */
	val = readl(pseninf + SENINF_CSI2_CTL) | (data_header_order << 16) |
		0x10 | ((1 << data_lane_num) - 1);
	writel(val, pseninf + SENINF_CSI2_CTL);

	SENINF_BITS(pseninf, SENINF_CSI2_RESYNC_MERGE_CTL,
		    BYPASS_LANE_RESYNC, 0);
	SENINF_BITS(pseninf, SENINF_CSI2_RESYNC_MERGE_CTL, CDPHY_SEL, 0);
	SENINF_BITS(pseninf, SENINF_CSI2_RESYNC_MERGE_CTL,
		    CPHY_LANE_RESYNC_CNT, 3);
	SENINF_BITS(pseninf, SENINF_CSI2_MODE, CSR_CSI2_MODE, 0);
	SENINF_BITS(pseninf, SENINF_CSI2_MODE, CSR_CSI2_HEADER_LEN, 0);
	SENINF_BITS(pseninf, SENINF_CSI2_DPHY_SYNC, SYNC_SEQ_MASK_0, 0xff00);
	SENINF_BITS(pseninf, SENINF_CSI2_DPHY_SYNC, SYNC_SEQ_PAT_0, 0x001d);

	SENINF_BITS(pseninf, SENINF_CSI2_CTL, CLOCK_HS_OPTION, 0);
	SENINF_BITS(pseninf, SENINF_CSI2_CTL, HSRX_DET_EN, 0);
	SENINF_BITS(pseninf, SENINF_CSI2_CTL, HS_TRAIL_EN, 1);
	SENINF_BITS(pseninf, SENINF_CSI2_HS_TRAIL, HS_TRAIL_PARAMETER,
		    SENINF_HS_TRAIL_PARAMETER);

	/* Set debug port to output packet number */
	SENINF_BITS(pseninf, SENINF_CSI2_DGB_SEL, DEBUG_EN, 1);
	SENINF_BITS(pseninf, SENINF_CSI2_DGB_SEL, DEBUG_SEL, 0x1a);
	/* Enable CSI2 IRQ mask */
	/* Turn on all interrupt */
	writel(0xffffffff, pseninf + SENINF_CSI2_INT_EN);
	/* Write clear CSI2 IRQ */
	writel(0xffffffff, pseninf + SENINF_CSI2_INT_STATUS);
	/* Enable CSI2 Extend IRQ mask */
	/* Turn on all interrupt */
	SENINF_BITS(pseninf, SENINF_CTRL, CSI2_SW_RST, 1);
	udelay(1);
	SENINF_BITS(pseninf, SENINF_CTRL, CSI2_SW_RST, 0);
}

static int mtk_seninf_power_on(struct mtk_seninf *priv)
{
	void __iomem *pseninf = priv->base;
	struct device *dev = priv->dev;
	unsigned int seninf;
	int ret;

	seninf = mtk_seninf_csi_port_to_seninf(priv->port);
	if (seninf < 0) {
		dev_err(dev, "seninf port mapping fail\n");
		return -EINVAL;
	}

	ret = pm_runtime_get_sync(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to pm_runtime_get_sync: %d\n", ret);
		pm_runtime_put_noidle(priv->dev);
		return ret;
	}

	/* Configure timestamp */
	SENINF_BITS(pseninf, SENINF_CTRL, SENINF_EN, 1);
	SENINF_BITS(pseninf, SENINF_CTRL_EXT, SENINF_CSI2_IP_EN, 1);
	writel(SENINF_TIMESTAMP_STEP, pseninf + SENINF_TG1_TM_STP);

	phy_set_mode_ext(priv->dphy, PHY_MODE_MIPI_DPHY, priv->port);
	phy_power_on(priv->dphy);

	mtk_seninf_rx_config(priv, seninf);

	mtk_seninf_set_csi_mipi(priv, seninf);

	mtk_seninf_set_mux(priv, seninf);

	writel(0x0, pseninf + SENINF_TOP_CAM_MUX_CTRL);

	return 0;
}

static void mtk_seninf_power_off(struct mtk_seninf *priv)
{
	unsigned int seninf = mtk_seninf_csi_port_to_seninf(priv->port);
	void __iomem *pseninf = priv->base + 0x1000 * seninf;

	/* Disable CSI2(2.5G) first */
	writel(readl(pseninf + SENINF_CSI2_CTL) & 0xFFFFFFE0,
	       pseninf + SENINF_CSI2_CTL);

	phy_power_off(priv->dphy);
	pm_runtime_put(priv->dev);
}

static const struct v4l2_mbus_framefmt mtk_seninf_default_fmt = {
	.code = MEDIA_BUS_FMT_SRGGB10_1X10,
	.width = DEFAULT_WIDTH,
	.height = DEFAULT_HEIGHT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
};

static void init_fmt(struct mtk_seninf *priv)
{
	unsigned int i;

	for (i = 0; i < NUM_PADS; i++)
		priv->fmt[i].format = mtk_seninf_default_fmt;
}

static int seninf_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_mbus_framefmt *mf;
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		mf = v4l2_subdev_get_try_format(sd, cfg, i);
		*mf = mtk_seninf_default_fmt;
	}

	return 0;
}

static int seninf_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->format.code == ~0U || fmt->format.code == 0)
		fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	} else {
		priv->fmt[fmt->pad].pad = fmt->pad;
		mf = &priv->fmt[fmt->pad].format;
	}
	*mf = fmt->format;

	return 0;
}

static int seninf_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	else
		fmt->format = priv->fmt[fmt->pad].format;

	return 0;
}

static int seninf_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);

	if (code->index >= NUM_PADS)
		return -EINVAL;

	code->code = priv->fmt[code->index].format.code;

	return 0;
}

static int seninf_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);

	if (on)
		return mtk_seninf_power_on(priv);
	mtk_seninf_power_off(priv);

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

static int seninf_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd;
	struct mtk_seninf *priv;
	struct device *dev;

	sd = media_entity_to_v4l2_subdev(entity);
	priv = v4l2_get_subdevdata(sd);
	dev = priv->dev;

	if (!(flags & MEDIA_LNK_FL_ENABLED))
		return 0;

	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		priv->mux_sel = local->index - CAM_MUX_IDX_MIN;
	} else {
		/* Select port */
		priv->port = local->index;
		if (priv->port >= NUM_SENSORS) {
			dev_err(dev, "port index is over number of ports\n");
			return -EINVAL;
		}
	}

	return 0;
}

static const struct media_entity_operations seninf_media_ops = {
	.link_setup = seninf_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

struct sensor_async_subdev {
	struct v4l2_async_subdev asd;
	u32 port;
	u32 lanes;
};

static int mtk_seninf_notifier_bound(
			struct v4l2_async_notifier *notifier,
			struct v4l2_subdev *sd,
			struct v4l2_async_subdev *asd)
{
	struct mtk_seninf *priv =
		container_of(notifier, struct mtk_seninf, notifier);
	struct sensor_async_subdev *s_asd =
		container_of(asd, struct sensor_async_subdev, asd);
	int ret;

	dev_dbg(priv->dev, "%s bound with port:%d lanes: %d\n",
		sd->entity.name, s_asd->port, s_asd->lanes);

	priv->sensor[s_asd->port].num_data_lanes = s_asd->lanes;

	ret = media_create_pad_link(&sd->entity, 0, &priv->subdev.entity,
				    s_asd->port, MEDIA_LNK_FL_ENABLED);
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

static int seninf_dump_debug_info(struct mtk_seninf *priv)
{
	void __iomem *pseninf = priv->base;
	struct device *dev = priv->dev;

	/* Sensor Interface Control */
	dev_dbg(dev,
		"SENINF_CSI2_CTL SENINF1:0x%x\n",
		readl(pseninf + SENINF_CSI2_CTL));
	/* Read width/height */
	/* Read interrupt status */
	dev_dbg(dev, "SENINF_IRQ:0x%x\n",
		readl(pseninf + SENINF_CSI2_INT_STATUS));
	/* Mux1 */
	dev_dbg(dev, "SENINF_MUX_CTRL:0x%x, INTSTA:0x%x, DEBUG_2(0x%x)\n",
		readl(pseninf + SENINF_MUX_CTRL),
		readl(pseninf + SENINF_MUX_INTSTA),
		readl(pseninf + SENINF_MUX_DEBUG_2));
	if (readl(pseninf + SENINF_MUX_INTSTA) & 0x1) {
		writel(0xffffffff, pseninf + SENINF_MUX_INTSTA);
		usleep_range(1000, 1000 * 2);
		dev_warn(dev, "overrun CTRL:%x INTSTA:%x DEBUG_2:%x\n",
			 readl(pseninf + SENINF_MUX_CTRL),
			 readl(pseninf + SENINF_MUX_INTSTA),
			 readl(pseninf + SENINF_MUX_DEBUG_2));
	}

	return 0;
}

static int seninf_enable_test_pattern(struct mtk_seninf *priv)
{
	void __iomem *pseninf = priv->base;
	unsigned int val;

	SENINF_BITS(pseninf, SENINF_TOP_CTRL, SENINF_PCLK_EN, 1);
	SENINF_BITS(pseninf, SENINF_TOP_CTRL, SENINF2_PCLK_EN, 1);
	SENINF_BITS(pseninf, SENINF_CTRL, SENINF_EN, 1);
	SENINF_BITS(pseninf, SENINF_CTRL, SENINF_SRC_SEL, 1);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_HSYNC_MASK, 1);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_SRC_SEL, 1);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_PUSH_EN, 0x1f);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FLUSH_EN, 0x1b);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 1);
	SENINF_BITS(pseninf, SENINF_MUX_CTRL, SENINF_MUX_EN, 1);
	writel(SENINF_IRQ_CLR_SEL | SENINF_ALL_ERR_IRQ_EN,
	       pseninf + SENINF_MUX_INTEN);
	writel(0x0, pseninf + SENINF_MUX_SPARE);
	writel(0xE2000, pseninf + SENINF_MUX_CTRL_EXT);
	writel(0x0, pseninf + SENINF_MUX_CTRL_EXT);
	SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_EN, 1);
	SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_PAT, 0xC);
	SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_VSYNC, 4);
	SENINF_BITS(pseninf, SENINF_TG1_TM_CTL, TM_DUMMYPXL, 4);
	val = (priv->fmt[priv->port].format.height + 0x100) << 16 |
		  (priv->fmt[priv->port].format.width + 0x100);
	writel(val, pseninf + SENINF_TG1_TM_SIZE);
	writel(0x0, pseninf + SENINF_TG1_TM_CLK);
	writel(0x1, pseninf + SENINF_TG1_TM_STP);
	writel(readl(pseninf + SENINF_CTRL_EXT) | 0x02,
	       pseninf + SENINF_CTRL_EXT);

	return 0;
}

static int seninf_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_seninf *priv = container_of(ctrl->handler,
					     struct mtk_seninf, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == TEST_GEN_PATTERN)
			return seninf_enable_test_pattern(priv);
		else if (ctrl->val == TEST_DUMP_DEBUG_INFO)
			return seninf_dump_debug_info(priv);
		else
			return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops seninf_ctrl_ops = {
	.s_ctrl = seninf_set_ctrl,
};

static const char * const seninf_test_pattern_menu[] = {
	"Horizontal bars",
	"Monitor status",
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

static int mtk_seninf_fwnode_parse(struct device *dev,
				   struct v4l2_fwnode_endpoint *vep,
				   struct v4l2_async_subdev *asd)
{
	struct sensor_async_subdev *s_asd =
		container_of(asd, struct sensor_async_subdev, asd);

	if (vep->bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "Only CSI2 bus type is currently supported\n");
		return -EINVAL;
	}

	s_asd->port = vep->base.port;
	s_asd->lanes = vep->bus.mipi_csi2.num_data_lanes;
	dev_info(dev,
		 "%s: s_asd->port=%d s_asd->lanes=%d\n", __func__,
		 s_asd->port, s_asd->lanes);
	return 0;
}

static int mtk_seninf_media_register(struct mtk_seninf *priv)
{
	struct v4l2_subdev *sd = &priv->subdev;
	struct media_pad *pads = priv->pads;
	struct device *dev = priv->dev;
	unsigned int i;
	int ret;

	v4l2_subdev_init(sd, &seninf_subdev_ops);

	init_fmt(priv);
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

	for (i = 0; i < NUM_SENSORS; i++)
		pads[i].flags = MEDIA_PAD_FL_SINK;

	for (i = CAM_MUX_IDX_MIN; i < NUM_PADS; i++)
		pads[i].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, NUM_PADS, pads);
	if (ret < 0)
		goto err_free_handler;

	// v4l2_async_notifier_init(&priv->notifier);

	for (i = 0; i < NUM_SENSORS; ++i) {
		ret = v4l2_async_notifier_parse_fwnode_endpoints_by_port(
			dev, &priv->notifier,
			sizeof(struct sensor_async_subdev), i,
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
	/* List of clocks required by seninf */
	static const char * const clk_names[] = {
		"cam_seninf", "top_mux_seninf"
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

	priv->dphy = devm_phy_get(dev, "mipi_dphy_rx");
	if (IS_ERR(priv->dphy)) {
		dev_err(dev, "failed to get phy:%d\n", PTR_ERR(priv->dphy));
		return PTR_ERR(priv->dphy);
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

#ifdef CONFIG_OF
static const struct of_device_id mtk_seninf_of_match[] = {
	{.compatible = "mediatek,mt8183-seninf"},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_seninf_of_match);
#endif

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
