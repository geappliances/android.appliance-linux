// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 * Author: Stu Hsieh <stu.hsieh@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/time64.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/iommu.h>
#include <linux/of_graph.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-core.h>
#include <linux/videodev2.h>
#include <soc/mediatek/smi.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/debugfs.h>

#define MTK_MIPICSI_DRV_NAME "mtk-mipicsi"
#define MTK_PLATFORM_STR "platform:mt8167"
#define MTK_DATAWIDTH_8					(0x01U << 7U)
#define MAX_SUPPORT_WIDTH             4096U
#define MAX_SUPPORT_HEIGHT            4096U
#define MAX_BUFFER_NUM                  32U

#define MIPI_RX_ANA00_CSI				0x00
#define MIPI_RX_ANA04_CSI				0x04
#define MIPI_RX_ANA08_CSI				0x08
#define MIPI_RX_ANA0C_CSI				0x0c
#define MIPI_RX_ANA10_CSI				0x10
#define MIPI_RX_ANA20_CSI				0x20
#define MIPI_RX_ANA24_CSI				0x24
#define MIPI_RX_ANA4C_CSI				0x4c
#define MIPI_RX_ANA50_CSI				0x50

#define SENINF_CTRL					0x00

#define SENINF_NCSI2_CAL_24				0x24
#define SENINF_NCSI2_CAL_38				0x38
#define SENINF_NCSI2_CAL_3C				0x3C
#define SENINF_NCSI2_CTL				0xA0
#define SENINF_NCSI2_LNRD_TIMING			0xA8
#define SENINF_NCSI2_INT_EN				0xB0
#define SENINF_NCSI2_INT_STATUS				0xB4
#define SENINF_NCSI2_DBG_SEL				0xB8
#define SENINF_NCSI2_DBG_PORT				0xBC
#define SENINF_NCSI2_HSRX_DBG				0xD8
#define SENINF_NCSI2_DI					0xDC
#define SENINF_NCSI2_DI_CTRL				0xE4

#define SENINF_TOP_CTRL					0x00
#define SENINF_TOP_CMODEL_PAR				0x04
#define SENINF_TOP_MUX					0x08

#define SENINF_MUX_CTRL					0x00
#define SENINF_MUX_DEBUG_2				0x14

#define CAMCTL_EN1					0x04
#define CAMCTL_DMA_EN					0x0C
#define CAMCTL_FMT_SEL					0x10
#define CAMCTL_INT_EN					0x20
#define CAMCTL_INT_STATUS				0x24
#define PASS1_DONE_STATUS				10

#define CAMCTL_MUX_SEL					0x74
#define CAMCTL_MUX_SEL2					0x78
#define CAMCTL_SRAM_MUX_CFG				0x7C

#define CAMCTL_CLK_EN					0x150

#define CAMIMGO_BASE_ADDR				0x300
#define CAMIMGO_XSIZE					0x308
#define CAMIMGO_YSIZE					0x30C
#define CAMIMGO_STRIDE					0x310

#define CAMTG_SEN_MODE					0x410
#define CAMTG_VF_CON					0x414
#define CAMTG_SEN_GRAB_PXL				0x418
#define CAMTG_SEN_GRAB_LIN				0x41C
#define CAMTG_PATH_CFG					0x420


#define CONFIG_DEBUG_FS 1

#define notifier_to_mipicsi(n) container_of(n, struct mtk_mipicsi_dev, \
					    notifier)
static int mtk_mipicsi_dbg_level;
#define mtk_mipicsi_dbg(level, fmt, args...)				 \
	do {								 \
		if (mtk_mipicsi_dbg_level >= level)			\
			pr_info("[MTK_MIPICSI%d] L%d %s %d: " fmt "\n", \
				mipicsi->id, level,  __func__, __LINE__, \
				##args);	\
	} while (0)

/* buffer for one video frame */
struct mtk_mipicsi_buf {
	struct list_head queue;
	struct vb2_buffer *vb;
	dma_addr_t vb_dma_addr_phy;
	int prepare_flag;
};

struct mtk_format {
	u32     fourcc;
	u32     mbus_code;
	u8      bpp;
};

struct mtk_mipicsi_subdev {
	struct device_node *node;
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *subdev;
	unsigned int max_vc;
	u32 link_reg;
};

struct mtk_mipicsi_channel {
	void __iomem            *seninf_mux;
	void __iomem            *camsv;
	struct clk		*clk;
	unsigned int		irq;
	bool			irq_status;
};

struct mtk_mipicsi_dev {
	struct platform_device		*pdev;
	struct mtk_mipicsi_channel	*channel;
	unsigned int		camsv_num;
	unsigned int		common_clk_num;
	struct clk		**common_clk;
	struct device		*larb_pdev;
	void __iomem		*ana;
	void __iomem		*seninf_ctrl;
	void __iomem		*seninf;
	struct regmap		*seninf_top;

	struct v4l2_device	v4l2_dev;
	struct video_device	*vdev;
	struct vb2_queue	queue;
	struct v4l2_async_notifier	notifier;
	struct mtk_mipicsi_subdev	mipicsi_sd;
	struct v4l2_format		fmt;
	unsigned int			num_user_formats;
	const struct mtk_format		**user_formats;
	const struct mtk_format		*current_fmt;
	u16			width_flags;	/* max 12 bits */

	struct mtk_mipicsi_buf	cam_buf[MAX_BUFFER_NUM];
	struct list_head	fb_list;
	bool streamon;
	unsigned int link;
	u8 link_reg_val;
	char drv_name[16];
	u32 id;
	struct timespec64 fps_time_cur;
	struct timespec64 fps_time_pre;

	spinlock_t		irqlock;
	spinlock_t		queue_lock;
	struct mutex		lock;
#ifdef CONFIG_DEBUG_FS
	struct dentry *mtk_mipicsi_debugfs;
#endif
};

static const struct mtk_format mtk_mipicsi_formats[] = {
{
	.fourcc = V4L2_PIX_FMT_YUYV,
	.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
	.bpp = 2,
}, {
	.fourcc = V4L2_PIX_FMT_YVYU,
	.mbus_code = MEDIA_BUS_FMT_YVYU8_2X8,
	.bpp = 2,
}, {
	.fourcc = V4L2_PIX_FMT_UYVY,
	.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
	.bpp = 2,
}, {
	.fourcc = V4L2_PIX_FMT_VYUY,
	.mbus_code = MEDIA_BUS_FMT_VYUY8_2X8,
	.bpp = 2,
},
};

#ifdef CONFIG_DEBUG_FS
static ssize_t mtk_mipicsi_debug_read(struct file *file, char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	struct device *dev = file->private_data;
	struct mtk_mipicsi_dev *mipicsi = dev_get_drvdata(dev);
	struct mtk_mipicsi_channel *ch = mipicsi->channel;
	u32 int_val;
	u32 dbg_port;
	u32 cnt_val;
	u32 hcnt;
	u32 vcnt;
	char buf[256];
	char cnt_info[150];
	int i;

	int_val = readl(mipicsi->seninf + SENINF_NCSI2_INT_STATUS);
	dbg_port = readl(mipicsi->seninf + SENINF_NCSI2_DBG_PORT);
	memset(buf, 0, sizeof(buf));
	snprintf(buf, sizeof(buf), "%s\nSENINF_NCSI2_INT_STATUS: 0x%X\n"
		"SENINF_NCSI2_DBG_PORT: 0x%X\n",
		dev_name(dev), int_val, dbg_port);

	for (i = 0; i < mipicsi->camsv_num; ++i) {
		cnt_val = readl(ch[i].seninf_mux + SENINF_MUX_DEBUG_2);
		hcnt = (cnt_val >> 16) & 0xFFFF;
		vcnt = cnt_val & 0xFFFF;
		memset(cnt_info, 0, sizeof(cnt_info));
		snprintf(cnt_info, sizeof(cnt_info),
			"HCNT[%d]: 0x%X\n"
			"VCNT[%d]: 0x%X\n",
			i, hcnt, i, vcnt);
		strcat(buf, cnt_info);
	}

	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}
static const struct file_operations mtk_mipicsi_debug_fops = {
	.open = simple_open,
	.read = mtk_mipicsi_debug_read,
};
#endif /* CONFIG_DEBUG_FS */

static int get_subdev_register(struct mtk_mipicsi_dev *mipicsi,
			       struct v4l2_dbg_register *reg)
{
	struct v4l2_subdev *sd = mipicsi->mipicsi_sd.subdev;
	struct device *dev = &mipicsi->pdev->dev;
	int ret = 0;

	reg->match.type = V4L2_CHIP_MATCH_SUBDEV;
	reg->match.addr = 0;
	ret = v4l2_subdev_call(sd, core, g_register, reg);
	if (ret != 2) {
		dev_err(dev, "mipicsi get des register 0x%llx fail, ret=%d\n",
		reg->reg, ret);
		return -EIO;
	}

	dev_info(dev, "read DES [reg/val/ret] is [0x%llx/0x%llx/%d]\n",
		 reg->reg, reg->val, ret);

	return ret;
}


static int get_subdev_link(struct mtk_mipicsi_dev *mipicsi,
	unsigned int *link, u8 *link_reg_val)
{
	struct device *dev = &mipicsi->pdev->dev;
	struct mtk_mipicsi_subdev *sd = &mipicsi->mipicsi_sd;
	struct v4l2_dbg_register reg;
	int ret = 0;
	unsigned int index = 0;

	if (sd->max_vc == 1) {
		*link = 1;
		*link_reg_val = 0x1;
		dev_info(dev, "mtk mipicsi support 1 channel\n");

		return 0;
	}

	dev_info(dev, "mtk mipicsi support %d channel\n", sd->max_vc);

	memset(&reg, 0, sizeof(reg));
	/*get camera link number*/
	reg.reg = sd->link_reg;
	ret = get_subdev_register(mipicsi, &reg);
	if (ret < 0)
		return ret;

	*link = 0;
	for (index = 0; index < sd->max_vc; index++) {
		if ((reg.val & 0x01) == 0x01) {
			*link += 1;
			*link_reg_val |= (0x01 << index);
		}
		reg.val >>= 1;
	}

	dev_info(dev, "%u camera linked to sub device\n", *link);

	return 0;
}

static void mtk_mipicsi_ana_clk_enable(void __iomem *base, bool enable)
{
	if (enable) {
		writel(1UL | readl(base + MIPI_RX_ANA00_CSI),
			base + MIPI_RX_ANA00_CSI);
		writel(1UL | readl(base + MIPI_RX_ANA04_CSI),
			base + MIPI_RX_ANA04_CSI);
		writel(1UL | readl(base + MIPI_RX_ANA08_CSI),
			base + MIPI_RX_ANA08_CSI);
		writel(1UL | readl(base + MIPI_RX_ANA0C_CSI),
			base + MIPI_RX_ANA0C_CSI);
		writel(1UL | readl(base + MIPI_RX_ANA10_CSI),
			base + MIPI_RX_ANA10_CSI);
		writel(1UL | readl(base + MIPI_RX_ANA20_CSI),
			base + MIPI_RX_ANA20_CSI);
		writel(1UL | readl(base + MIPI_RX_ANA24_CSI),
			base + MIPI_RX_ANA24_CSI);
	} else {
		writel(~1UL & readl(base + MIPI_RX_ANA00_CSI),
			base + MIPI_RX_ANA00_CSI);
		writel(~1UL & readl(base + MIPI_RX_ANA04_CSI),
			base + MIPI_RX_ANA04_CSI);
		writel(~1UL & readl(base + MIPI_RX_ANA08_CSI),
			base + MIPI_RX_ANA08_CSI);
		writel(~1UL & readl(base + MIPI_RX_ANA0C_CSI),
			base + MIPI_RX_ANA0C_CSI);
		writel(~1UL & readl(base + MIPI_RX_ANA10_CSI),
			base + MIPI_RX_ANA10_CSI);
		writel(~1UL & readl(base + MIPI_RX_ANA20_CSI),
			base + MIPI_RX_ANA20_CSI);
		writel(~1UL & readl(base + MIPI_RX_ANA24_CSI),
			base + MIPI_RX_ANA24_CSI);
	}
}

static void mtk_mipicsi_ana_init(void __iomem *base)
{
	writel(0xFEFBEFBEU & readl(base + MIPI_RX_ANA4C_CSI),
		base + MIPI_RX_ANA4C_CSI);
	writel(0xFEFBEFBEU & readl(base + MIPI_RX_ANA50_CSI),
		base + MIPI_RX_ANA50_CSI);

	/* clock lane and lane0-lane3 input select */
	writel(8UL | readl(base + MIPI_RX_ANA00_CSI),
		base + MIPI_RX_ANA00_CSI);
	writel(8UL | readl(base + MIPI_RX_ANA04_CSI),
		base + MIPI_RX_ANA04_CSI);
	writel(8UL | readl(base + MIPI_RX_ANA08_CSI),
		base + MIPI_RX_ANA08_CSI);
	writel(8UL | readl(base + MIPI_RX_ANA0C_CSI),
		base + MIPI_RX_ANA0C_CSI);
	writel(8UL | readl(base + MIPI_RX_ANA10_CSI),
		base + MIPI_RX_ANA10_CSI);

	/* BG chopper clock and CSI BG enable */
	writel(11UL | readl(base + MIPI_RX_ANA24_CSI),
		base + MIPI_RX_ANA24_CSI);
	mdelay(1);

	/* LDO core bias enable */
	writel(0xFF030003U | readl(base + MIPI_RX_ANA20_CSI),
		base + MIPI_RX_ANA20_CSI);
	mdelay(1);
}

static void mtk_mipicsi_seninf_ctrl_init(void __iomem *base)
{
	/*seninf enable. select NCSI2 as seninif input source */
	writel(0x8001U, base + SENINF_CTRL);
}

static void mtk_mipicsi_seninf_init(void __iomem *base)
{
	writel(1U, base + SENINF_NCSI2_CAL_38);
	writel(0x00051545U, base + SENINF_NCSI2_CAL_3C);
	writel(5U, base + SENINF_NCSI2_CAL_38);
	mdelay(1);
	writel(4U, base + SENINF_NCSI2_CAL_38);
	writel(0U, base + SENINF_NCSI2_CAL_3C);
	writel(0x11U, base + SENINF_NCSI2_DBG_SEL);
	writel(0x189617FU, base + SENINF_NCSI2_CTL);
	writel(~(1UL << 27) & readl(base + SENINF_NCSI2_CTL),
		base + SENINF_NCSI2_CTL);
	writel((1UL << 27) | readl(base + SENINF_NCSI2_CTL),
		base + SENINF_NCSI2_CTL);
	writel(0x2800U, base + SENINF_NCSI2_LNRD_TIMING);
	writel(0x7FFFU, base + SENINF_NCSI2_INT_STATUS);
	writel(0x7FCFFFFEU, base + SENINF_NCSI2_INT_EN);
	writel(0xE4000000U, base + SENINF_NCSI2_CAL_24);
	writel(0xFFFFFF00U & readl(base + SENINF_NCSI2_DBG_SEL),
		base + SENINF_NCSI2_DBG_SEL);
	writel(0xFFFFFF45U | readl(base + SENINF_NCSI2_DBG_SEL),
		base + SENINF_NCSI2_DBG_SEL);
	writel(0xFFFFFFEFU & readl(base + SENINF_NCSI2_HSRX_DBG),
		base + SENINF_NCSI2_HSRX_DBG);
	writel(0x01010101U, base + SENINF_NCSI2_DI_CTRL);
	writel(0x03020100U, base + SENINF_NCSI2_DI);
	writel(0x10, base + SENINF_NCSI2_DBG_SEL);
}

static int mtk_mipicsi_seninf_top_init(struct regmap *regmap)
{
	int ret;

	ret = regmap_write(regmap, SENINF_TOP_CTRL, 0x00010C00U);
	if (ret)
		return ret;

	ret = regmap_write(regmap, SENINF_TOP_CMODEL_PAR, 0x00079871);
	if (ret)
		return ret;

	ret = regmap_write(regmap, SENINF_TOP_MUX, 0x11110000);
	if (ret)
		return ret;

	return ret;
}

static void mtk_mipicsi_seninf_mux_init(void __iomem *base, unsigned int ch)
{
	unsigned int mux_ctrl_val = (((0x9EFF8U + ch) << 12U) | 0x180U);

	/* select seninf_mux1-4 as input for NCSI2 VC0-3*/
	writel(mux_ctrl_val, base + SENINF_MUX_CTRL);
}

static void mtk_mipicsi_camsv_csr_init(void __iomem *base)
{
	/* double buffer enable. IMGO enable. PAK sel. TG enable */
	writel(0x40000019U, base + CAMCTL_EN1);
	/* IMGO DP, PAK DP and TG clk enable */
	writel(0x00008005U, base + CAMCTL_CLK_EN);
	/* 0: raw8, 1:raw10, 2:raw12, 3:YUV422, 4:raw14, 7:JPEG */
	writel(0x00000003U, base + CAMCTL_FMT_SEL);
	/* write clear enable. pass1 down interrupt enable */
	writel(0x80000400U, base + CAMCTL_INT_EN);
}

static void mtk_mipicsi_camsv_tg_init(void __iomem *base, u32 b, u32 h)
{
	/* bit[30:16] grab end pixel clock number.
	 * bit[14:0] grab start pixel clock number
	 */
	writel(b << 16U, base + CAMTG_SEN_GRAB_PXL);
	/* bit[29:16] end line number. bit[13:0] start line number */
	writel(h << 16U, base + CAMTG_SEN_GRAB_LIN);
	/* YUV sensor unsigned to signed enable */
	writel(0x1000U, base + CAMTG_PATH_CFG);
	/* cmos enable YUV422 mode */
	writel(3U, base + CAMTG_SEN_MODE);
}

static void mtk_mipicsi_camsv_dma_init(void __iomem *base, u32 b, u32 h)
{
	/* enable SW format setting. YUV format. 16bit */
	writel(0x01810000U | b, base + CAMIMGO_STRIDE);
	/* b -1 bytes per line to write */
	writel(b - 1U, base + CAMIMGO_XSIZE);
	/* w - 1 lines to write */
	writel(h - 1U, base + CAMIMGO_YSIZE);
	/* disable frame header function */
	writel(0U, base + DMA_FRAME_HEADER_EN);
}

static void mtk_mipicsi_camsv_init(void __iomem *base, u32 b, u32 h)
{
	mtk_mipicsi_camsv_csr_init(base);
	mtk_mipicsi_camsv_tg_init(base, b, h);
	mtk_mipicsi_camsv_dma_init(base, b, h);
}

static void mtk_mipicsi_reg_init(struct mtk_mipicsi_dev *mipicsi)
{
	struct mtk_mipicsi_channel *ch = mipicsi->channel;
	struct device *dev = &mipicsi->pdev->dev;
	unsigned int i;
	int ret;

	mtk_mipicsi_ana_init(mipicsi->ana);
	mtk_mipicsi_seninf_ctrl_init(mipicsi->seninf_ctrl);
	mtk_mipicsi_seninf_init(mipicsi->seninf);
	ret = mtk_mipicsi_seninf_top_init(mipicsi->seninf_top);
	if (ret)
		dev_err(dev, "seninf_top_init error\n");

	for (i = 0; i < mipicsi->camsv_num; i++) {
		u32 b = 1280*2;
		u32 h = 720;

		mtk_mipicsi_seninf_mux_init(ch[i].seninf_mux, i);
		mtk_mipicsi_camsv_init(ch[i].camsv, b, h);
	}
}

static void mipicsi_clk_enable(struct mtk_mipicsi_dev *mipicsi, bool enable)
{
	struct mtk_mipicsi_channel *ch = mipicsi->channel;
	int i;

	for (i = 0; i < mipicsi->camsv_num; i++)
		enable ? clk_prepare_enable(ch[i].clk) :
			 clk_disable_unprepare(ch[i].clk);

	for (i = 0; i < mipicsi->common_clk_num; i++)
		enable ? clk_prepare_enable(mipicsi->common_clk[i]) :
			 clk_disable_unprepare(mipicsi->common_clk[i]);

	mtk_mipicsi_ana_clk_enable(mipicsi->ana, enable);
}

static int mtk_mipicsi_pm_suspend(struct device *dev)
{
	struct mtk_mipicsi_dev *mipicsi = dev_get_drvdata(dev);
	int ret = 0;

	mipicsi_clk_enable(mipicsi, false);

	if (mipicsi->larb_pdev != NULL)
		mtk_smi_larb_put(mipicsi->larb_pdev);

	return ret;
}

static int mtk_mipicsi_suspend(struct device *dev)
{
	if (pm_runtime_suspended(dev))
		return 0;

	return mtk_mipicsi_pm_suspend(dev);
}

static int mtk_mipicsi_pm_resume(struct device *dev)
{
	struct mtk_mipicsi_dev *mipicsi = dev_get_drvdata(dev);
	int ret = 0;

	if (mipicsi->larb_pdev != NULL) {
		ret = mtk_smi_larb_get(mipicsi->larb_pdev);
		if (ret != 0) {
			dev_err(dev, "failed to get larb, err %d", ret);

			return ret;
		}
	}

	mipicsi_clk_enable(mipicsi, true);

	mtk_mipicsi_reg_init(mipicsi);

	return ret;
}

static int mtk_mipicsi_resume(struct device *dev)
{
	if (pm_runtime_suspended(dev))
		return 0;

	return mtk_mipicsi_pm_resume(dev);
}

static const struct dev_pm_ops mtk_mipicsi_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(mtk_mipicsi_suspend, mtk_mipicsi_resume)
	SET_RUNTIME_PM_OPS(mtk_mipicsi_pm_suspend,
		mtk_mipicsi_pm_resume, NULL)
};

static int mtk_mipicsi_vb2_queue_setup(struct vb2_queue *vq,
		unsigned int *nbufs, unsigned int *num_planes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct mtk_mipicsi_dev *mipicsi = vb2_get_drv_priv(vq);
	u32 sizeimage = mipicsi->fmt.fmt.pix.sizeimage;

	if (*nbufs == 0U || *nbufs > MAX_BUFFER_NUM)
		*nbufs = MAX_BUFFER_NUM;

	/*
	 * Called from VIDIOC_REQBUFS or in compatibility mode For YUV422P
	 * format, even if there are 3 planes Y, U and V, we reply there is only
	 * one plane, containing Y, U and V data, one after the other.
	 */
	if (*num_planes != 0U)
		return sizes[0] < sizeimage ? -EINVAL : 0;

	sizes[0] = sizeimage;
	*num_planes = 1;

	return 0;
}

static int mtk_mipicsi_vb2_init(struct vb2_buffer *vb)
{
	struct mtk_mipicsi_dev *mipicsi = vb2_get_drv_priv(vb->vb2_queue);

	mipicsi->cam_buf[vb->index].prepare_flag = 0;

	return 0;
}

static int mtk_mipicsi_vb2_prepare(struct vb2_buffer *vb)
{
	struct mtk_mipicsi_dev *mipicsi = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_mipicsi_buf *buf;
	u32 size = 0;

	buf = &mipicsi->cam_buf[vb->index];
	size = mipicsi->fmt.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(&mipicsi->pdev->dev, "data will not fit into plane (%lu < %u)",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	if ((buf->prepare_flag) == 0) {
		buf->prepare_flag = 1;
		buf->vb_dma_addr_phy =
			vb2_dma_contig_plane_dma_addr(vb, 0);

		mtk_mipicsi_dbg(1, "vb_dma_addr_phy=%lx size=%d",
			(unsigned long)buf->vb_dma_addr_phy,
			vb->planes[0].bytesused);

		buf->vb = vb;
	}

	return 0;
}

static void mtk_mipicsi_fill_buffer(void __iomem *base, dma_addr_t dma_handle)
{
	writel(dma_handle, base + CAMIMGO_BASE_ADDR);
}

static void mtk_mipicsi_write_camsv(struct mtk_mipicsi_dev *mipicsi,
				    unsigned int index,
				    unsigned int max_camsv_num)
{
	struct mtk_mipicsi_channel *ch = mipicsi->channel;
	unsigned int i = 0;
	u8 link_index = 0;
	u32 bytesperline = mipicsi->fmt.fmt.pix.bytesperline;
	u32 height = mipicsi->fmt.fmt.pix.height;
	u64 offset = 0;

	for (i = 0; i < max_camsv_num; i++)
		if (((mipicsi->link_reg_val >> i) & 0x01) == 0x01) {
			offset = (u64)link_index * bytesperline * height;
			mtk_mipicsi_fill_buffer(ch[i].camsv,
				mipicsi->cam_buf[index].vb_dma_addr_phy
					+ offset);
			link_index++;
		}
}

static void mtk_mipicsi_vb2_queue(struct vb2_buffer *vb)
{
	struct mtk_mipicsi_dev *mipicsi = vb2_get_drv_priv(vb->vb2_queue);

	spin_lock(&mipicsi->queue_lock);
	list_add_tail(&(mipicsi->cam_buf[vb->index].queue),
		&(mipicsi->fb_list));
	spin_unlock(&mipicsi->queue_lock);

	spin_lock(&mipicsi->irqlock);
	if (!mipicsi->streamon)
		mtk_mipicsi_write_camsv(mipicsi, vb->index, mipicsi->camsv_num);

	spin_unlock(&mipicsi->irqlock);

	mtk_mipicsi_dbg(2, "enqueue NO.%d buffer(%p).", vb->index, vb);
}

static void mtk_mipicsi_cmos_vf_enable(struct mtk_mipicsi_dev *mipicsi,
				       unsigned int max_camsv_num,
				       bool enable)
{
	struct mtk_mipicsi_channel *ch = mipicsi->channel;
	void __iomem *base = NULL;
	u32 mask = enable ? (u32)1 : ~(u32)1;
	int i;

	for (i = 0; i < max_camsv_num; i++)
		if (((mipicsi->link_reg_val >> i) & 0x01U) == 0x01U) {
			base = ch[i].camsv;
			if (enable) {
				enable_irq(ch[i].irq);

				/*enable cmos_en and vf_en*/
				writel(readl(base + CAMTG_SEN_MODE) | mask,
				       base + CAMTG_SEN_MODE);
				writel(readl(base + CAMTG_VF_CON) | mask,
				       base + CAMTG_VF_CON);
			} else {
				/*disable cmos_en and vf_en*/
				writel(readl(base + CAMTG_SEN_MODE) & mask,
					base + CAMTG_SEN_MODE);
				writel(readl(base + CAMTG_VF_CON) & mask,
					base + CAMTG_VF_CON);

				disable_irq(ch[i].irq);
				ch[i].irq_status = false;
			}
		}
}

static int mtk_mipicsi_vb2_start_streaming(struct vb2_queue *vq,
		unsigned int count)
{
	struct mtk_mipicsi_dev *mipicsi = vb2_get_drv_priv(vq);

	mtk_mipicsi_cmos_vf_enable(mipicsi, mipicsi->camsv_num, true);

	mipicsi->streamon = true;

	return 0;
}

static void mtk_mipicsi_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_mipicsi_dev *mipicsi = vb2_get_drv_priv(vq);
	struct mtk_mipicsi_buf *buf = NULL;
	struct mtk_mipicsi_buf *tmp = NULL;
	unsigned int index = 0;

	mtk_mipicsi_cmos_vf_enable(mipicsi, mipicsi->camsv_num, false);

	spin_lock(&mipicsi->queue_lock);
	while (list_empty(&(mipicsi->fb_list)) == 0) {
		list_for_each_entry_safe(buf, tmp, &(mipicsi->fb_list), queue) {
			if (buf->vb->state == VB2_BUF_STATE_ACTIVE) {
				vb2_buffer_done(buf->vb, VB2_BUF_STATE_ERROR);
				break;
			}
		}
		buf->vb_dma_addr_phy = 0ULL;
		buf->prepare_flag = 0;
		index = buf->vb->index;
		list_del_init(&(mipicsi->cam_buf[index].queue));
	}
	spin_unlock(&mipicsi->queue_lock);

	INIT_LIST_HEAD(&(mipicsi->fb_list));

	mipicsi->streamon = false;
}

static struct vb2_ops mtk_vb2_ops = {
	.queue_setup		= mtk_mipicsi_vb2_queue_setup,
	.buf_init		= mtk_mipicsi_vb2_init,
	.buf_prepare		= mtk_mipicsi_vb2_prepare,
	.buf_queue		= mtk_mipicsi_vb2_queue,
	.start_streaming	= mtk_mipicsi_vb2_start_streaming,
	.stop_streaming		= mtk_mipicsi_vb2_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int mtk_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static int mtk_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int mtk_enum_input(struct file *file, void *priv,
				struct v4l2_input *i)
{
	if (i->index != 0)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(i->name, "Camera", sizeof(i->name));

	return 0;
}

static int mtk_enum_fmt_vid_cap(struct file *file, void  *priv,
				struct v4l2_fmtdesc *f)
{
	struct mtk_mipicsi_dev *mipicsi = video_drvdata(file);

	if (f->index >= mipicsi->num_user_formats)
		return -EINVAL;

	f->pixelformat = mipicsi->user_formats[f->index]->fourcc;

	return 0;
}

static const struct mtk_format *find_format_by_fourcc(
					struct mtk_mipicsi_dev *mipicsi,
					unsigned int fourcc)
{
	unsigned int num_formats = mipicsi->num_user_formats;
	const struct mtk_format *fmt;
	unsigned int i;

	for (i = 0; i < num_formats; i++) {
		fmt = mipicsi->user_formats[i];
		if (fmt->fourcc == fourcc)
			return fmt;
	}

	return NULL;
}

static int mtk_mipicsi_try_fmt(struct mtk_mipicsi_dev *mipicsi,
			      struct v4l2_format *f,
			      const struct mtk_format **current_fmt)
{
	const struct mtk_format *mtk_fmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev *sd = mipicsi->mipicsi_sd.subdev;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	int ret = 0;

	mtk_fmt = find_format_by_fourcc(mipicsi, pix->pixelformat);
	if (!mtk_fmt) {
		mtk_fmt = mipicsi->user_formats[0];
		pix->pixelformat = mtk_fmt->fourcc;
	}

	/* limit to MTK hardware capabilities */
	pix->height = clamp(pix->height, 0U, MAX_SUPPORT_HEIGHT);
	pix->width = clamp(pix->width, 0U, MAX_SUPPORT_WIDTH);
	v4l2_fill_mbus_format(&format.format, pix, mtk_fmt->mbus_code);
	ret = v4l2_subdev_call(sd, pad, set_fmt, &pad_cfg, &format);
	if (ret < 0)
		return ret;

	v4l2_fill_pix_format(pix, &format.format);
	pix->bytesperline = pix->width * mtk_fmt->bpp;
	pix->sizeimage = pix->bytesperline * pix->height;

	if (current_fmt)
		*current_fmt = mtk_fmt;

	return ret;
}

static int mtk_mipicsi_set_fmt(struct mtk_mipicsi_dev *mipicsi,
				struct v4l2_format *f)
{
	struct v4l2_subdev *sd = mipicsi->mipicsi_sd.subdev;
	struct device *dev = &mipicsi->pdev->dev;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	const struct mtk_format *current_fmt;
	int ret;

	ret = mtk_mipicsi_try_fmt(mipicsi, f, &current_fmt);
	if (ret)
		return ret;

	v4l2_fill_mbus_format(&format.format, &f->fmt.pix,
			current_fmt->mbus_code);

	ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &format);
	if (ret < 0)
		return ret;

	mipicsi->fmt = *f;
	mipicsi->current_fmt = current_fmt;

	dev_info(dev, "width/height/sizeimage %u/%u/%u", pix->width,
							 pix->height,
							 pix->sizeimage);

	return ret;
}

static int mtk_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mtk_mipicsi_dev *mipicsi = video_drvdata(file);

	if (vb2_is_streaming(&mipicsi->queue))
		return -EBUSY;

	return mtk_mipicsi_set_fmt(mipicsi, f);
}

static int mtk_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct mtk_mipicsi_dev *mipicsi = video_drvdata(file);

	*fmt = mipicsi->fmt;

	return 0;
}

static int mtk_try_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mtk_mipicsi_dev *mipicsi = video_drvdata(file);

	return mtk_mipicsi_try_fmt(mipicsi, f, NULL);
}

static int mtk_mipicsi_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	struct mtk_mipicsi_dev *mipicsi = video_drvdata(file);

	strlcpy(cap->card, MTK_PLATFORM_STR, sizeof(cap->card));
	strlcpy(cap->driver, mipicsi->drv_name, sizeof(cap->driver));
	strlcpy(cap->bus_info, MTK_PLATFORM_STR, sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static const struct v4l2_ioctl_ops mtk_mipicsi_ioctl_ops = {
	.vidioc_querycap                = mtk_mipicsi_querycap,

	.vidioc_try_fmt_vid_cap         = mtk_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap           = mtk_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap           = mtk_s_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap        = mtk_enum_fmt_vid_cap,

	.vidioc_enum_input              = mtk_enum_input,
	.vidioc_g_input                 = mtk_g_input,
	.vidioc_s_input                 = mtk_s_input,

	.vidioc_reqbufs                 = vb2_ioctl_reqbufs,
	.vidioc_create_bufs             = vb2_ioctl_create_bufs,
	.vidioc_querybuf                = vb2_ioctl_querybuf,
	.vidioc_qbuf                    = vb2_ioctl_qbuf,
	.vidioc_dqbuf                   = vb2_ioctl_dqbuf,
	.vidioc_expbuf                  = vb2_ioctl_expbuf,
	.vidioc_prepare_buf             = vb2_ioctl_prepare_buf,
	.vidioc_streamon                = vb2_ioctl_streamon,
	.vidioc_streamoff               = vb2_ioctl_streamoff,

	.vidioc_log_status              = v4l2_ctrl_log_status,
	.vidioc_subscribe_event         = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event       = v4l2_event_unsubscribe,
};

static int get_irq_channel(struct mtk_mipicsi_dev *mipicsi)
{
	struct mtk_mipicsi_channel *ch = mipicsi->channel;
	int i;
	u32 int_reg_val;

	for (i = 0; i < mipicsi->camsv_num; i++) {
		int_reg_val = readl(ch[i].camsv + CAMCTL_INT_STATUS);
		if ((int_reg_val & (1 << PASS1_DONE_STATUS)) != 0)
			return i;
	}

	return -1;
}

static void mtk_mipicsi_irq_buf_process(struct mtk_mipicsi_dev *mipicsi)
{
	struct mtk_mipicsi_channel *ch = mipicsi->channel;
	unsigned int i = 0;
	struct mtk_mipicsi_buf *new_cam_buf = NULL;
	struct mtk_mipicsi_buf *tmp = NULL;
	unsigned int index = 0;
	unsigned int next = 0;
	long time_interval;

	for (i = 0; i < mipicsi->camsv_num; ++i)
		ch[i].irq_status = false;

	i = 0;

	/* only one buffer left */
	if ((&(mipicsi->fb_list))->next->next == &(mipicsi->fb_list)) {
		mtk_mipicsi_dbg(1, "only 1 buffer left, drop frame");
		return;
	}

	/*for each fb_lst 2 times to get the top 2 buffer.*/
	list_for_each_entry_safe(new_cam_buf, tmp,
		&(mipicsi->fb_list), queue) {
		if (i == 0) {
			index = new_cam_buf->vb->index;
		} else {
			next = new_cam_buf->vb->index;
			break;
		}
		++i;
	}

	mtk_mipicsi_write_camsv(mipicsi, next, mipicsi->camsv_num);

	/*
	 * fb_list has one more buffer. Free the first buffer to user
	 * and fill the second buffer to HW.
	 */
	vb2_buffer_done(mipicsi->cam_buf[index].vb,
		VB2_BUF_STATE_DONE);

	list_del_init(&(mipicsi->cam_buf[index].queue));

	if (mtk_mipicsi_dbg_level >= 2) {
		ktime_get_real_ts64(&(mipicsi->fps_time_cur));

		time_interval = (mipicsi->fps_time_cur.tv_sec
			- mipicsi->fps_time_pre.tv_sec) * 1000000000
			+ (mipicsi->fps_time_cur.tv_nsec
			- mipicsi->fps_time_pre.tv_nsec);
		mtk_mipicsi_dbg(0, "time interval is %ld\n",
			time_interval);
		mipicsi->fps_time_pre.tv_sec =
			mipicsi->fps_time_cur.tv_sec;
		mipicsi->fps_time_pre.tv_nsec =
			mipicsi->fps_time_cur.tv_nsec;
	}
}

static irqreturn_t mtk_mipicsi_isr(int irq, void *data)
{
	struct mtk_mipicsi_dev *mipicsi = data;
	struct device *dev = &mipicsi->pdev->dev;
	struct mtk_mipicsi_channel *ch = mipicsi->channel;
	unsigned long flags = 0;
	int isr_ch;
	u8 irq_cnt = 0, i = 0;

	spin_lock_irqsave(&mipicsi->irqlock, flags);

	isr_ch = get_irq_channel(mipicsi);
	if (isr_ch < 0) {
		dev_info(dev, "no interrupt occur");
		spin_unlock_irqrestore(&mipicsi->irqlock, flags);
		return IRQ_HANDLED;
	}

	/* clear interrupt */
	writel(1UL << PASS1_DONE_STATUS,
		ch[isr_ch].camsv + CAMCTL_INT_STATUS);
	ch[isr_ch].irq_status = true;
	for (i = 0U; i < mipicsi->camsv_num; ++i) {
		if (ch[i].irq_status)
			++irq_cnt;
	}

	if (irq_cnt == mipicsi->link)
		mtk_mipicsi_irq_buf_process(mipicsi);
	spin_unlock_irqrestore(&mipicsi->irqlock, flags);

	return IRQ_HANDLED;
}

static int seninf_mux_camsv_node_parse(struct mtk_mipicsi_dev *mipicsi,
		int index)
{
	int ret;
	int irq;
	struct clk *clk = NULL;
	struct device *dev = NULL;
	struct resource *res = NULL;
	struct platform_device *camdma_pdev = NULL;
	struct device_node *np = NULL;
	struct mtk_mipicsi_channel *ch = mipicsi->channel;

	dev = &mipicsi->pdev->dev;

	np = of_parse_phandle(dev->of_node,
		"mediatek,seninf_mux_camsv", index);
	if (np == NULL) {
		dev_err(dev, "no NO.%d mediatek,seninf_mux_camsv node\n",
			index);
		return -ENODEV;
	}

	camdma_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (camdma_pdev == NULL) {
		camdma_pdev = of_platform_device_create(np, NULL,
					platform_bus_type.dev_root);
	}

	clk = of_clk_get(np, 0);
	if (clk == NULL) {
		dev_err(dev, "get clk fail in %s node\n", np->full_name);
		return -ENODEV;
	}
	ch[index].clk = clk;

	irq = of_irq_get(np, 0);
	if (irq <= 0) {
		dev_err(dev, "get irq fail in %s node\n", np->full_name);
		return -ENODEV;
	}
	ch[index].irq = irq;

	ret = devm_request_irq(dev, irq,
			mtk_mipicsi_isr, 0,
			mipicsi->drv_name, mipicsi);
	if (ret != 0) {
		dev_err(dev, "%s irq register failed\n", np->full_name);
		return -ENODEV;
	}
	disable_irq(ch[index].irq);
	ch[index].irq_status = false;

	res = platform_get_resource(camdma_pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "get seninf_mux memory failed in %s node\n",
			np->full_name);
		return -ENODEV;
	}
	ch[index].seninf_mux = devm_ioremap_resource(&camdma_pdev->dev, res);

	res = platform_get_resource(camdma_pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(dev, "get camsv memory failed in %s node\n",
			np->full_name);
		return -ENODEV;
	}
	ch[index].camsv = devm_ioremap_resource(&camdma_pdev->dev, res);

	dev_info(dev, "%s parse done\n", np->full_name);

	return 0;
}

static int mtk_mipicsi_common_node_parse(struct mtk_mipicsi_dev *mipicsi,
	struct device_node *node)
{
	int i = 0;
	struct regmap *seninf_top = NULL;
	struct device *dev = NULL;
	struct clk *clk = NULL;

	if ((mipicsi == NULL) || (node == NULL))
		return -EINVAL;

	dev = &mipicsi->pdev->dev;

	/* All the mipicsi HW share the same seninf_top */
	seninf_top = syscon_regmap_lookup_by_phandle(dev->of_node,
			"mediatek,mipicsi");
	if (seninf_top == NULL) {
		dev_err(dev, "Missing mediadek,mipicsi in %s node\n",
			node->full_name);
		return -EINVAL;
	}
	mipicsi->seninf_top = seninf_top;

	/* get IMG_SENINF_CAM_EN and IMG_SENINF_SCAM_EN clk*/
	mipicsi->common_clk_num = of_count_phandle_with_args(node, "clocks",
							     "#clock-cells");
	if (mipicsi->common_clk_num < 0) {
		dev_err(dev, "common clock number error\n");
		return -EINVAL;
	}

	mipicsi->common_clk = devm_kmalloc_array(dev, mipicsi->common_clk_num,
						 sizeof(*mipicsi->common_clk),
						 GFP_KERNEL);
	for (i = 0; i < mipicsi->common_clk_num; i++) {
		clk = of_clk_get(node, i);
		if (clk == NULL) {
			dev_err(dev, "get clk fail in %s node\n",
				node->full_name);
			return -EINVAL;
		}
		mipicsi->common_clk[i] = clk;
	}

	dev_info(dev, "%s parse done\n", node->full_name);

	return 0;
}

static int mtk_mipicsi_node_parse(struct mtk_mipicsi_dev *mipicsi)
{
	int ret;
	int camsv_num = 0;
	int i;
	struct device *dev = NULL;
	struct resource *res = NULL;
	struct device_node *common_node = NULL;
	struct platform_device *pdev = NULL;
	struct mtk_mipicsi_subdev *sd = &mipicsi->mipicsi_sd;

	dev = &mipicsi->pdev->dev;
	pdev = mipicsi->pdev;

	/* mediatek,mipicsiid is a flag to show which mipicsi HW */
	ret = of_property_read_u32(dev->of_node, "mediatek,mipicsiid",
		(u32 *)&mipicsi->id);
	if (ret != 0) {
		dev_info(dev, "not set mediatek,mipicsiid, use default id 0\n");
		mipicsi->id = 0;
	}
	(void)sprintf(mipicsi->drv_name, MTK_MIPICSI_DRV_NAME"%d",
		mipicsi->id);

	/*get the number of virtual channel*/
	ret = of_property_read_u32(dev->of_node, "mediatek,mipicsi_max_vc",
				   &sd->max_vc);
	if (ret != 0) {
		dev_info(dev, "not set mediatek,mipicsi_max_vc, use default value 1\n");
		sd->max_vc = 1;
	}

	ret = of_property_read_u32(dev->of_node, "mediatek,serdes_link_reg",
				   &sd->link_reg);
	if (ret != 0) {
		dev_info(dev, "not set mediatek,serdes_link_reg, can't read subdev link number\n");
		sd->link_reg = 0x0;
	}

	/* get and parse seninf_mux_camsv */
	camsv_num = of_count_phandle_with_args(dev->of_node,
		"mediatek,seninf_mux_camsv", NULL);
	if (camsv_num <= 0) {
		dev_err(dev, "no mediatek,seninf_mux_camsv\n");
		return -EINVAL;
	}
	mipicsi->camsv_num = camsv_num;
	dev_info(dev, "there are %d camsv node\n", camsv_num);

	mipicsi->channel = devm_kmalloc_array(dev, camsv_num,
					      sizeof(*mipicsi->channel),
					      GFP_KERNEL);

	for (i = 0; i < mipicsi->camsv_num; ++i) {
		ret = seninf_mux_camsv_node_parse(mipicsi, i);
		if (ret < 0) {
			dev_err(dev,
				"NO.%d seninf_mux_camsv node parse fail\n", i);
			return ret;
		}
	}

	/* get mediatek,mipicsi node and its resource */
	common_node = of_parse_phandle(dev->of_node, "mediatek,mipicsi", 0);
	if (common_node == NULL) {
		dev_err(dev, "no mediadek,mipicsi\n");
		return -EINVAL;
	}

	ret = mtk_mipicsi_common_node_parse(mipicsi, common_node);
	if (ret < 0)
		return ret;
	of_node_put(common_node);

	/*get ana and seninf reg*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "get ana register failed\n");
		return -ENODEV;
	}
	mipicsi->ana = devm_ioremap_resource(&pdev->dev, res);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(dev, "get seninf_ctrl register failed\n");
		return -ENODEV;
	}
	mipicsi->seninf_ctrl = devm_ioremap_resource(&pdev->dev, res);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res == NULL) {
		dev_err(dev, "get seninf register failed\n");
		return -ENODEV;
	}
	mipicsi->seninf = devm_ioremap_resource(&pdev->dev, res);

	dev_info(dev, "mipicsi node parse done\n");

	return 0;
}

static int mtk_mipicsi_set_default_fmt(struct mtk_mipicsi_dev *mipicsi)
{
	struct v4l2_format f = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = {
			.width          = 1280,
			.height         = 720,
			.field          = V4L2_FIELD_NONE,
			.pixelformat    = mipicsi->user_formats[0]->fourcc,
		},
	};
	int ret;

	ret = mtk_mipicsi_try_fmt(mipicsi, &f, NULL);
	if (ret)
		return ret;
	mipicsi->current_fmt = mipicsi->user_formats[0];
	mipicsi->fmt = f;

	return 0;
}

static int mipicsi_formats_init(struct mtk_mipicsi_dev *mipicsi)
{
	const struct mtk_format *mipicsi_fmts[ARRAY_SIZE(mtk_mipicsi_formats)];
	struct v4l2_subdev *sd = mipicsi->mipicsi_sd.subdev;
	unsigned int i, j, num_fmts = 0;
	struct v4l2_subdev_mbus_code_enum mbus_code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	while (!v4l2_subdev_call(sd, pad, enum_mbus_code, NULL, &mbus_code)) {
		for (i = 0; i < ARRAY_SIZE(mtk_mipicsi_formats); i++) {
			if (mtk_mipicsi_formats[i].mbus_code != mbus_code.code)
				continue;

			/* Code supported, have we got this fourcc yet? */
			for (j = 0; j < num_fmts; j++)
				if (mipicsi_fmts[j]->fourcc ==
				    mtk_mipicsi_formats[i].fourcc)
					/* Already available */
					break;

			if (j == num_fmts)
				/* new */
				mipicsi_fmts[num_fmts++] =
					&mtk_mipicsi_formats[i];
		}
		mbus_code.index++;
	}

	if (!num_fmts)
		return -ENXIO;

	mipicsi->num_user_formats = num_fmts;
	mipicsi->user_formats = devm_kcalloc(&mipicsi->pdev->dev,
					     num_fmts,
					     sizeof(struct isi_format *),
					     GFP_KERNEL);
	if (!mipicsi->user_formats)
		return -ENOMEM;

	memcpy(mipicsi->user_formats, mipicsi_fmts,
	       num_fmts * sizeof(struct mtk_format *));
	mipicsi->current_fmt = mipicsi->user_formats[0];

	return 0;
}

static int mipicsi_subdev_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct mtk_mipicsi_dev *mipicsi = notifier_to_mipicsi(notifier);
	struct device *dev = &mipicsi->pdev->dev;
	int ret;

	mipicsi->vdev->ctrl_handler = mipicsi->mipicsi_sd.subdev->ctrl_handler;
	ret = mipicsi_formats_init(mipicsi);
	if (ret) {
		dev_err(dev, "No supported mediabus format found\n");
		return ret;
	}

	ret = mtk_mipicsi_set_default_fmt(mipicsi);
	if (ret) {
		dev_err(dev, "Could not set default format\n");
		return ret;
	}

	ret = video_register_device(mipicsi->vdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(dev, "Failed to register video device\n");
		return ret;
	}

	dev_dbg(dev, "Device registered as %s\n",
		video_device_node_name(mipicsi->vdev));

	return 0;
}

static void mipicsi_subdev_notify_unbind(struct v4l2_async_notifier *notifier,
					 struct v4l2_subdev *sd,
					 struct v4l2_async_subdev *asd)
{
	struct mtk_mipicsi_dev *mipicsi = notifier_to_mipicsi(notifier);

	dev_dbg(&mipicsi->pdev->dev, "Removing %s\n",
		video_device_node_name(mipicsi->vdev));

	/* Checks internally if vdev have been init or not */
	video_unregister_device(mipicsi->vdev);
}

static int mipicsi_subdev_notify_bound(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *subdev,
				       struct v4l2_async_subdev *asd)
{
	struct mtk_mipicsi_dev *mipicsi = notifier_to_mipicsi(notifier);

	dev_dbg(&mipicsi->pdev->dev, "subdev %s bound\n", subdev->name);

	mipicsi->mipicsi_sd.subdev = subdev;

	return 0;
}

static const struct v4l2_async_notifier_operations mipicsi_subdev_notify_ops = {
	.bound = mipicsi_subdev_notify_bound,
	.unbind = mipicsi_subdev_notify_unbind,
	.complete = mipicsi_subdev_notify_complete,
};

static int mtk_mipicsi_graph_parse(struct mtk_mipicsi_dev *mipicsi,
					struct device_node *node)
{
	struct device_node *ep = NULL;
	struct device_node *remote;

	ep = of_graph_get_next_endpoint(node, ep);
	if (!ep)
		return -EINVAL;

	remote = of_graph_get_remote_port_parent(ep);
	of_node_put(ep);
	if (!remote)
		return -EINVAL;

	/* Remote node to connect */
	mipicsi->mipicsi_sd.node = remote;
	mipicsi->mipicsi_sd.asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
	mipicsi->mipicsi_sd.asd.match.fwnode = of_fwnode_handle(remote);
	return 0;
}

static int mtk_mipicsi_subdev_init(struct mtk_mipicsi_dev *mipicsi)
{
	int ret;
	struct device *dev = &mipicsi->pdev->dev;

	/* Parse the graph to extract a list of subdevice DT nodes. */
	ret = mtk_mipicsi_graph_parse(mipicsi, dev->of_node);
	if (ret < 0) {
		dev_err(&mipicsi->pdev->dev, "Graph parsing failed\n");
		return ret;
	}

	v4l2_async_notifier_init(&mipicsi->notifier);

	ret = v4l2_async_notifier_add_subdev(&mipicsi->notifier,
						&mipicsi->mipicsi_sd.asd);
	if (ret) {
		of_node_put(mipicsi->mipicsi_sd.node);
		return ret;
	}

	mipicsi->notifier.ops = &mipicsi_subdev_notify_ops;

	ret = v4l2_async_notifier_register(&mipicsi->v4l2_dev,
					   &mipicsi->notifier);
	if (ret < 0) {
		dev_err(&mipicsi->pdev->dev, "Notifier registration failed\n");
		v4l2_async_notifier_cleanup(&mipicsi->notifier);
		return ret;
	}

	return 0;
}

static int mtk_mipicsi_open(struct file *file)
{
	struct mtk_mipicsi_dev *mipicsi = video_drvdata(file);
	struct v4l2_subdev *sd = mipicsi->mipicsi_sd.subdev;
	int ret;

	if (mutex_lock_interruptible(&mipicsi->lock))
		return -ERESTARTSYS;

	ret = v4l2_fh_open(file);
	if (ret < 0)
		goto unlock;

	if (!v4l2_fh_is_singular_file(file))
		goto fh_rel;

	ret = v4l2_subdev_call(sd, core, s_power, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		goto fh_rel;

	ret = mtk_mipicsi_set_fmt(mipicsi, &mipicsi->fmt);
	if (ret)
		v4l2_subdev_call(sd, core, s_power, 0);

	get_subdev_link(mipicsi, &mipicsi->link, &mipicsi->link_reg_val);

	pm_runtime_get_sync(&mipicsi->pdev->dev);

fh_rel:
	if (ret)
		v4l2_fh_release(file);
unlock:
	mutex_unlock(&mipicsi->lock);
	return ret;
}

static int mtk_mipicsi_release(struct file *file)
{
	struct mtk_mipicsi_dev *mipicsi = video_drvdata(file);
	struct device *dev = &mipicsi->pdev->dev;
	struct v4l2_subdev *sd = mipicsi->mipicsi_sd.subdev;
	bool fh_singular;
	int ret;

	mutex_lock(&mipicsi->lock);

	pm_runtime_put_sync(dev);

	fh_singular = v4l2_fh_is_singular_file(file);

	ret = _vb2_fop_release(file, NULL);

	if (fh_singular)
		v4l2_subdev_call(sd, core, s_power, 0);

	mutex_unlock(&mipicsi->lock);

	return ret;
}

static const struct v4l2_file_operations mipicsi_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open           = mtk_mipicsi_open,
	.release        = mtk_mipicsi_release,
	.poll           = vb2_fop_poll,
	.mmap           = vb2_fop_mmap,
	.read           = vb2_fop_read,
};

static int mtk_mipicsi_probe(struct platform_device *pdev)
{
	struct mtk_mipicsi_dev *mipicsi = NULL;
	int ret = 0;
	struct iommu_domain *iommu = NULL;
	struct device_node *larb_node = NULL;
	struct platform_device *larb_pdev = NULL;
	struct vb2_queue *q;

	iommu = iommu_get_domain_for_dev(&pdev->dev);
	if (iommu == NULL) {
		dev_err(&pdev->dev, "Waiting iommu driver ready...\n");
		return -EPROBE_DEFER;
	}

	larb_node = of_parse_phandle(pdev->dev.of_node, "mediatek,larb", 0);
	if (larb_node == NULL) {
		dev_err(&pdev->dev, "Missing mediadek,larb in %s node\n",
			pdev->dev.of_node->full_name);
		return -EINVAL;
	}

	larb_pdev = of_find_device_by_node(larb_node);
	if (larb_pdev == NULL || !larb_pdev->dev.driver) {
		of_node_put(larb_node);
		dev_err(&pdev->dev, "Waiting for larb device %s\n",
			larb_node->full_name);
		return -EPROBE_DEFER;
	}
	of_node_put(larb_node);

	mipicsi = devm_kzalloc(&pdev->dev, sizeof(*mipicsi), GFP_KERNEL);
	if (mipicsi == NULL)
		return -ENOMEM;

	mipicsi->pdev = pdev;
	mipicsi->larb_pdev = &larb_pdev->dev;

	ret = mtk_mipicsi_node_parse(mipicsi);
	if (ret < 0)
		return ret;

	pm_runtime_enable(&pdev->dev);

	INIT_LIST_HEAD(&mipicsi->fb_list);
	spin_lock_init(&mipicsi->queue_lock);
	spin_lock_init(&mipicsi->irqlock);
	mutex_init(&mipicsi->lock);

	q = &mipicsi->queue;

	/* Initialize the top-level structure */
	ret = v4l2_device_register(&pdev->dev, &mipicsi->v4l2_dev);
	if (ret)
		return ret;

	mipicsi->vdev = video_device_alloc();
	if (mipicsi->vdev == NULL) {
		ret = -ENOMEM;
		goto err_vdev_alloc;
	}

	/* video node */
	mipicsi->vdev->fops = &mipicsi_fops;
	mipicsi->vdev->v4l2_dev = &mipicsi->v4l2_dev;
	mipicsi->vdev->queue = &mipicsi->queue;
	strscpy(mipicsi->vdev->name, mipicsi->drv_name,
		sizeof(mipicsi->vdev->name));
	mipicsi->vdev->release = video_device_release;
	mipicsi->vdev->ioctl_ops = &mtk_mipicsi_ioctl_ops;
	mipicsi->vdev->lock = &mipicsi->lock;
	mipicsi->vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE |
				     V4L2_CAP_STREAMING |
				     V4L2_CAP_READWRITE;
	video_set_drvdata(mipicsi->vdev, mipicsi);

	/* buffer queue */
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = mipicsi;
	q->buf_struct_size = sizeof(struct vb2_buffer);
	q->ops = &mtk_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->dev = mipicsi->v4l2_dev.dev;
	q->lock = &mipicsi->lock;

	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialize VB2 queue\n");
		goto err_vb2_queue;
	}

	mipicsi->width_flags = MTK_DATAWIDTH_8;
	mipicsi->streamon = false;

	ret = mtk_mipicsi_subdev_init(mipicsi);
	if (ret < 0)
		goto err_mipicsi_subdev_init;

	ret = vb2_dma_contig_set_max_seg_size(&pdev->dev, DMA_BIT_MASK(32U));
	if (ret != 0) {
		dev_err(&pdev->dev, "dma set max seg size fail\n");
		goto clean;
	}

#ifdef CONFIG_DEBUG_FS
	mipicsi->mtk_mipicsi_debugfs =
		debugfs_create_file(mipicsi->drv_name, 0444, NULL,
			(void *)(&pdev->dev), &mtk_mipicsi_debug_fops);
	if (mipicsi->mtk_mipicsi_debugfs == NULL) {
		dev_err(&pdev->dev, "debugfs_create_file fail\n");
		goto clean;
	}
#endif

	dev_set_drvdata(&pdev->dev, mipicsi);

	dev_info(&pdev->dev, "probe done\n");
	return ret;
clean:
err_mipicsi_subdev_init:
err_vb2_queue:
	video_device_release(mipicsi->vdev);
err_vdev_alloc:
	v4l2_device_unregister(&mipicsi->v4l2_dev);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int mtk_mipicsi_remove(struct platform_device *pdev)
{
#ifdef CONFIG_DEBUG_FS
	struct mtk_mipicsi_dev *mipicsi = dev_get_drvdata(&pdev->dev);

	debugfs_remove(mipicsi->mtk_mipicsi_debugfs);
#endif
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id mtk_mipicsi_of_match[] = {
	{ .compatible = "mediatek,mt2712-mipicsi", },
	{},
};

static struct platform_driver mtk_mipicsi_driver = {
	.driver		= {
		.name	= MTK_MIPICSI_DRV_NAME,
		.pm	= &mtk_mipicsi_pm,
		.of_match_table = of_match_ptr(mtk_mipicsi_of_match),
	},
	.probe		= mtk_mipicsi_probe,
	.remove		= mtk_mipicsi_remove,
};

module_platform_driver(mtk_mipicsi_driver);
module_param(mtk_mipicsi_dbg_level, int, 0644);
MODULE_DESCRIPTION("MediaTek SoC Camera Host driver");
MODULE_LICENSE("GPL v2");
