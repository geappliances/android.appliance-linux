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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <soc/mediatek/smi.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define MTK_MIPICSI_DRV_NAME "mtk-mipicsi"
#define MTK_PLATFORM_STR "platform:mt2712"

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
#define SENINF_NCSI2_HSRX_DBG				0xD8
#define SENINF_NCSI2_DI					0xDC
#define SENINF_NCSI2_DI_CTRL				0xE4

#define SENINF_TOP_CTRL					0x00
#define SENINF_TOP_CMODEL_PAR				0x04
#define SENINF_TOP_MUX					0x08

#define SENINF_MUX_CTRL					0x00

#define CAMSV_MODULE_EN					0x10
#define CAMSV_FMT_SEL					0x14
#define CAMSV_INT_EN					0x18
#define CAMSV_CLK_EN					0x30

#define CAMSV_TG_SEN_MODE				0x500
#define CAMSV_TG_SEN_GRAB_PXL				0x508
#define CAMSV_TG_SEN_GRAB_LIN				0x50C
#define CAMSV_TG_PATH_CFG				0x510

#define IMGO_XSIZE					0x230
#define IMGO_YSIZE					0x234
#define IMGO_STRIDE					0x238
#define DMA_FRAME_HEADER_EN				0xE00

struct mtk_mipicsi_channel {
	void __iomem            *seninf_mux;
	void __iomem            *camsv;
	struct clk		*clk;
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
};

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
	writel(0x40000019U, base + CAMSV_MODULE_EN);
	/* IMGO DP, PAK DP and TG clk enable */
	writel(0x00008005U, base + CAMSV_CLK_EN);
	/* 0: raw8, 1:raw10, 2:raw12, 3:YUV422, 4:raw14, 7:JPEG */
	writel(0x00000003U, base + CAMSV_FMT_SEL);
	/* write clear enable. pass1 down interrupt enable */
	writel(0x80000400U, base + CAMSV_INT_EN);
}

static void mtk_mipicsi_camsv_tg_init(void __iomem *base, u32 b, u32 h)
{
	/* bit[30:16] grab end pixel clock number.
	 * bit[14:0] grab start pixel clock number
	 */
	writel(b << 16U, base + CAMSV_TG_SEN_GRAB_PXL);
	/* bit[29:16] end line number. bit[13:0] start line number */
	writel(h << 16U, base + CAMSV_TG_SEN_GRAB_LIN);
	/* YUV sensor unsigned to signed enable */
	writel(0x1000U, base + CAMSV_TG_PATH_CFG);
	/* cmos enable YUV422 mode */
	writel(3U, base + CAMSV_TG_SEN_MODE);
}

static void mtk_mipicsi_camsv_dma_init(void __iomem *base, u32 b, u32 h)
{
	/* enable SW format setting. YUV format. 16bit */
	writel(0x01810000U | b, base + IMGO_STRIDE);
	/* b -1 bytes per line to write */
	writel(b - 1U, base + IMGO_XSIZE);
	/* w - 1 lines to write */
	writel(h - 1U, base + IMGO_YSIZE);
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

static int seninf_mux_camsv_node_parse(struct mtk_mipicsi_dev *mipicsi,
		int index)
{
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

	dev = &mipicsi->pdev->dev;
	pdev = mipicsi->pdev;

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

static int mtk_mipicsi_probe(struct platform_device *pdev)
{
	struct mtk_mipicsi_dev *mipicsi = NULL;
	int ret = 0;
	struct iommu_domain *iommu = NULL;
	struct device_node *larb_node = NULL;
	struct platform_device *larb_pdev = NULL;

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

	dev_set_drvdata(&pdev->dev, mipicsi);

	dev_info(&pdev->dev, "probe done\n");

	return ret;
}

static int mtk_mipicsi_remove(struct platform_device *pdev)
{
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
MODULE_DESCRIPTION("MediaTek SoC Camera Host driver");
MODULE_LICENSE("GPL v2");
