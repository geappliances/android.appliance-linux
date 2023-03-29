// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 BayLibre, SAS.
 * Author: Amjad Ouled-Ameur <aouledameur@baylibre.com>
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include "mtk_drm_ddp_comp.h"

/* LVDSTX REGS */
#define OUT_CTRL			0x18
#define LVDS_E					BIT(0)

#define CLK_CTRL			0x20
#define RG_TX_CK_EN				BIT(0)
#define RG_RX_CK_EN				BIT(1)
#define RG_TEST_CK_EN				BIT(2)

#define SOFT_RESET			0x34
#define RG_PCLK_SW_RST				BIT(0)
#define RG_CTSCLK_SW_RST			BIT(1)

/* LVDSANA REGS */
#define MIPI_LVDS_ANA04			0x04
#define RG_LVDSTX_TVO_MASK			GENMASK(3, 0)
#define RG_LVDSTX_TVO				0x7
#define RG_LVDSTX_TVCM_MASK			GENMASK(7, 4)
#define RG_LVDSTX_TVCM				(0x8 << 4)
#define RG_LVDSTX_LDO_EN			BIT(22)
#define RG_LVDSTX_BIAS_EN			BIT(23)

#define MIPI_LVDS_ANA08			0x08
#define RG_LVDSTX_EXT_EN			0x1f
#define RG_LVDSTX_DRV_EN			(0x1f << 6)
#define RG_LVDSTX_TSTPAD_EN			BIT(23)
#define RG_LVDSTX_MPX_EN			BIT(25)

#define MIPI_LVDS_ANA0C			0x0c
#define RG_LVDSTX_BIASLPF_EN			BIT(21)

#define MIPI_LVDS_ANA10			0x10
#define RG_LVDSTX_LDO1LPF_EN			BIT(8)

#define MIPI_LVDS_ANA14			0x14
#define RG_VPLL_RESERVE				BIT(1)
#define RG_VPLL_EN				BIT(2)
#define RG_VPLL_PREDIV_MASK			GENMASK(5, 4)
#define RG_VPLL_PREDIV				(0x1 << 4)

#define MIPI_LVDS_ANA18			0x18
#define RG_VPLL_SDM_PCW				(0x38 << 24)

#define MIPI_LVDS_ANA1C			0x1c
#define RG_VPLL_TXDIV1				0x2
#define RG_VPLL_LVDS_EN				BIT(5)
#define RG_VPLL_LVDS_DPIX_DIV2			BIT(6)
#define RG_VPLL_LVDS_TTL_EN			BIT(7)
#define RG_VPLL_TXDIV5_EN			BIT(11)
#define RG_VPLL_SDM_PWR_ON			BIT(13)
#define RG_VPLL_SDM_ISO_EN			BIT(14)

struct mtk_lvds {
	struct mtk_ddp_comp ddp_comp;
	struct device *dev;
	void __iomem *regs_tx;
	void __iomem *regs_ana;

	struct clk *lvdstx_cts_clk;
	struct clk *lvdstx_pxl_clk;
	struct clk *lvdstx_dig_clk;
};

static void lvds_update_bits(void __iomem *regs, unsigned int reg,
			     unsigned int mask, unsigned int val)
{
	unsigned int tmp = readl(regs + reg);

	tmp = (tmp & ~mask) | (val & mask);
	writel(tmp, regs + reg);
}

static inline void mtk_lvds_soft_reset(struct mtk_lvds *lvds)
{
	writel(0x0, lvds->regs_tx + SOFT_RESET);
	writel(RG_CTSCLK_SW_RST | RG_PCLK_SW_RST,
			lvds->regs_tx + SOFT_RESET);
}

static void mtk_lvds_config(struct mtk_ddp_comp *comp, unsigned int w,
			    unsigned int h, unsigned int vrefresh,
			    unsigned int bpc)
{
	struct mtk_lvds *lvds = container_of(comp, struct mtk_lvds, ddp_comp);

	lvds_update_bits(lvds->regs_ana, MIPI_LVDS_ANA04,
			 RG_LVDSTX_BIAS_EN | RG_LVDSTX_LDO_EN |
			 RG_LVDSTX_TVCM_MASK | RG_LVDSTX_TVO_MASK,
			 RG_LVDSTX_BIAS_EN | RG_LVDSTX_LDO_EN |
			 RG_LVDSTX_TVCM | RG_LVDSTX_TVO);

	writel(RG_LVDSTX_MPX_EN | RG_LVDSTX_TSTPAD_EN | RG_LVDSTX_DRV_EN |
	       RG_LVDSTX_EXT_EN, lvds->regs_ana + MIPI_LVDS_ANA08);

	writel(RG_LVDSTX_BIASLPF_EN, lvds->regs_ana + MIPI_LVDS_ANA0C);

	lvds_update_bits(lvds->regs_ana, MIPI_LVDS_ANA10,
			 RG_LVDSTX_LDO1LPF_EN, RG_LVDSTX_LDO1LPF_EN);

	writel(RG_VPLL_SDM_ISO_EN | RG_VPLL_SDM_PWR_ON | RG_VPLL_TXDIV5_EN |
	       RG_VPLL_LVDS_TTL_EN | RG_VPLL_LVDS_DPIX_DIV2 |
	       RG_VPLL_LVDS_EN | RG_VPLL_TXDIV1,
	       lvds->regs_ana + MIPI_LVDS_ANA1C);

	udelay(20);

	writel(RG_VPLL_SDM_PWR_ON | RG_VPLL_TXDIV5_EN |
	       RG_VPLL_LVDS_TTL_EN | RG_VPLL_LVDS_DPIX_DIV2 |
	       RG_VPLL_LVDS_EN | RG_VPLL_TXDIV1,
	       lvds->regs_ana + MIPI_LVDS_ANA1C);

	udelay(20);

	writel(RG_VPLL_SDM_PCW, lvds->regs_ana + MIPI_LVDS_ANA18);

	udelay(20);

	lvds_update_bits(lvds->regs_ana, MIPI_LVDS_ANA14,
			 RG_VPLL_PREDIV_MASK | RG_VPLL_EN |
			 RG_VPLL_RESERVE,
			 RG_VPLL_PREDIV | RG_VPLL_EN | RG_VPLL_RESERVE);
}

static void mtk_lvds_start(struct mtk_ddp_comp *comp)
{
	struct mtk_lvds *lvds = container_of(comp, struct mtk_lvds, ddp_comp);

	writel(LVDS_E, lvds->regs_tx + OUT_CTRL);
	writel(RG_TEST_CK_EN | RG_RX_CK_EN | RG_TX_CK_EN,
			lvds->regs_tx + CLK_CTRL);
	mtk_lvds_soft_reset(lvds);
}

static int mtk_lvds_enable_clks(struct mtk_lvds *lvds)
{
	int ret;

	ret = clk_prepare_enable(lvds->lvdstx_cts_clk);
	if (ret) {
		dev_err(lvds->dev,
			"Failed to enable lvdstx_cts clock: %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(lvds->lvdstx_pxl_clk);
	if (ret) {
		dev_err(lvds->dev,
			"Failed to enable lvdstx_pxl clock: %d\n", ret);
		goto err_lvdstx_pxl;
	}
	ret = clk_prepare_enable(lvds->lvdstx_dig_clk);
	if (ret) {
		dev_err(lvds->dev,
			"Failed to enable lvdstx_dig clock: %d\n", ret);
		goto err_lvdstx_dig;
	}

return 0;

err_lvdstx_dig:
	clk_disable_unprepare(lvds->lvdstx_pxl_clk);
err_lvdstx_pxl:
	clk_disable_unprepare(lvds->lvdstx_cts_clk);

	return ret;
}

static const struct mtk_ddp_comp_funcs mtk_lvds_funcs = {
	.config = mtk_lvds_config,
	.start = mtk_lvds_start,
};

static int mtk_lvds_bind(struct device *dev, struct device *master, void *data)
{
	int ret;
	struct drm_device *drm = data;
	struct mtk_lvds *lvds = dev_get_drvdata(dev);

	ret = mtk_ddp_comp_register(drm, &lvds->ddp_comp);
	if (ret < 0) {
		dev_err(dev, "Failed to register component %pOF: %d\n",
			dev->of_node, ret);
		return ret;
	}

	return 0;
}

static void mtk_lvds_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct drm_device *drm = data;
	struct mtk_lvds *lvds = dev_get_drvdata(dev);

	mtk_ddp_comp_unregister(drm, &lvds->ddp_comp);
}

static const struct component_ops mtk_lvds_component_ops = {
	.bind = mtk_lvds_bind,
	.unbind = mtk_lvds_unbind,
};

static int mtk_lvds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct mtk_lvds *lvds;
	struct resource *mem;
	int comp_id;
	int ret;

	lvds = devm_kzalloc(dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	lvds->dev = dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lvds->regs_tx = devm_ioremap_resource(dev, mem);
	if (IS_ERR(lvds->regs_tx)) {
		ret = PTR_ERR(lvds->regs_tx);
		dev_err(dev, "Failed to get memory resource: %d\n", ret);
		return ret;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	lvds->regs_ana = devm_ioremap_resource(dev, mem);
	if (IS_ERR(lvds->regs_ana)) {
		ret = PTR_ERR(lvds->regs_ana);
		dev_err(dev, "Failed to get memory resource: %d\n", ret);
		return ret;
	}

	lvds->lvdstx_cts_clk = devm_clk_get(dev, "lvdstx_cts");
	if (IS_ERR(lvds->lvdstx_cts_clk)) {
		ret = PTR_ERR(lvds->lvdstx_cts_clk);
		dev_err(dev, "Failed to get lvdstx_cts clock: %d\n", ret);
		return ret;
	}

	lvds->lvdstx_pxl_clk = devm_clk_get(dev, "lvdstx_pxl");
	if (IS_ERR(lvds->lvdstx_pxl_clk)) {
		ret = PTR_ERR(lvds->lvdstx_pxl_clk);
		dev_err(dev, "Failed to get lvdstx_pxl clock: %d\n", ret);
		return ret;
	}

	lvds->lvdstx_dig_clk = devm_clk_get(dev, "lvdstx_dig");
	if (IS_ERR(lvds->lvdstx_dig_clk)) {
		ret = PTR_ERR(lvds->lvdstx_dig_clk);
		dev_err(dev, "Failed to get lvdstx_dig clock: %d\n", ret);
		return ret;
	}

	ret = mtk_lvds_enable_clks(lvds);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, lvds);

	comp_id = mtk_ddp_comp_get_id(node, MTK_LVDS);
	if (comp_id < 0) {
		dev_err(dev, "Failed to identify by alias: %d\n", comp_id);
		ret = comp_id;
		goto err_comp;
	}

	ret = mtk_ddp_comp_init(dev, node, &lvds->ddp_comp, comp_id,
				&mtk_lvds_funcs);
	if (ret) {
		dev_err(dev, "Failed to initialize component: %d\n", ret);
		goto err_comp;
	}

	ret = component_add(dev, &mtk_lvds_component_ops);
	if (ret) {
		dev_err(dev, "failed to add component: %d\n", ret);
		goto err_comp;
	}

	return 0;

err_comp:
	clk_disable_unprepare(lvds->lvdstx_dig_clk);
	clk_disable_unprepare(lvds->lvdstx_pxl_clk);
	clk_disable_unprepare(lvds->lvdstx_cts_clk);

	return ret;
}

static int mtk_lvds_remove(struct platform_device *pdev)
{
	struct mtk_lvds *lvds = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &mtk_lvds_component_ops);

	clk_disable_unprepare(lvds->lvdstx_dig_clk);
	clk_disable_unprepare(lvds->lvdstx_pxl_clk);
	clk_disable_unprepare(lvds->lvdstx_cts_clk);

	return 0;
}

static const struct of_device_id mtk_lvds_of_ids[] = {
	{ .compatible = "mediatek,mt8365-lvds",},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_lvds_of_ids);

struct platform_driver mtk_lvds_driver = {
	.probe = mtk_lvds_probe,
	.remove = mtk_lvds_remove,
	.driver = {
		.name = "mediatek-lvds",
		.of_match_table = mtk_lvds_of_ids,
	},
};

MODULE_AUTHOR("Amjad Ouled-Ameur <aouledameur@baylibre.com>");
MODULE_DESCRIPTION("MediaTek LVDS Driver");
MODULE_LICENSE("GPL v2");
