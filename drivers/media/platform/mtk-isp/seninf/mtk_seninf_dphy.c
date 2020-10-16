// SPDX-License-Identifier: GPL-2.0

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/phy/phy.h>
#include "mtk_seninf_rx_reg.h"

#define CSI_PORT_0A_ADDR_OFST	0
#define CSI_PORT_0B_ADDR_OFST	0x1000
#define CSI_PORT_1_ADDR_OFST	0x2000
#define CSI_PORT_2_ADDR_OFST	0x4000
/* Mix DPHY/CPHY */
#define CSI0A_OFST              0
#define CSI0B_OFST              0x1000
/* DPHY only */
#define CSI1A_OFST              0
#define CSI1B_OFST              0x1000

enum {
	MTK_MIPI_PHY_PORT_0 = 0x0, /* 4D1C */
	MTK_MIPI_PHY_PORT_1, /* 4D1C */
	MTK_MIPI_PHY_PORT_2, /* 4D1C */
	MTK_MIPI_PHY_PORT_0A, /* 2D1C */
	MTK_MIPI_PHY_PORT_0B, /* 2D1C */
	MTK_MIPI_PHY_PORT_MAX_NUM
};

#define MIPI_BITS(base, reg, field, val) do { \
		u32 __iomem *__p = (base) + (reg); \
		u32 __v = *__p; \
		__v &= ~reg##_##field##_MASK; \
		__v |= ((val) << reg##_##field##_SHIFT); \
		*__p = __v; \
	} while (0)

struct mtk_mipi_dphy {
	struct device *dev;
	void __iomem *rx;
	unsigned char __iomem *csi2_rx[MTK_MIPI_PHY_PORT_MAX_NUM];
	unsigned int port;
};

static inline int is_4d1c(unsigned int port)
{
	return port < MTK_MIPI_PHY_PORT_0A;
}

static inline int is_cdphy_combo(unsigned int port)
{
	return port == MTK_MIPI_PHY_PORT_0A ||
		port == MTK_MIPI_PHY_PORT_0B ||
		port == MTK_MIPI_PHY_PORT_0;
}

static int mtk_mipi_phy_power_on(struct phy *phy)
{
	struct mtk_mipi_dphy *priv = phy_get_drvdata(phy);

	void __iomem *pmipi_rx_base = priv->csi2_rx[MTK_MIPI_PHY_PORT_0];
	unsigned int port = priv->port;
	void __iomem *pmipi_rx = priv->csi2_rx[port];

	/* Set analog phy mode to DPHY */
	if (is_cdphy_combo(port))
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A, RG_CSI0A_CPHY_EN, 0);

	if (is_4d1c(port)) {
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L0_CKMODE_EN, 0);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L0_CKSEL, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L1_CKMODE_EN, 0);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L1_CKSEL, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L2_CKMODE_EN, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L2_CKSEL, 1);
	} else {
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L0_CKMODE_EN, 0);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L0_CKSEL, 0);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L1_CKMODE_EN, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L1_CKSEL, 0);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L2_CKMODE_EN, 0);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L2_CKSEL, 0);
	}
	if (is_cdphy_combo(port))
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_CPHY_EN, 0);

	if (is_4d1c(port)) {
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L0_CKMODE_EN, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L0_CKSEL, 1);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L1_CKMODE_EN, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L1_CKSEL, 1);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L2_CKMODE_EN, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L2_CKSEL, 1);
	} else {
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L0_CKSEL, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L1_CKMODE_EN, 1);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L1_CKSEL, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L2_CKMODE_EN, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_DPHY_L2_CKSEL, 0);
	}
	/* Byte clock invert */
	MIPI_BITS(pmipi_rx, MIPI_RX_ANAA8_CSI0A,
		  RG_CSI0A_CDPHY_L0_T0_BYTECK_INVERT, 1);
	MIPI_BITS(pmipi_rx, MIPI_RX_ANAA8_CSI0A,
		  RG_CSI0A_DPHY_L1_BYTECK_INVERT, 1);
	MIPI_BITS(pmipi_rx, MIPI_RX_ANAA8_CSI0A,
		  RG_CSI0A_CDPHY_L2_T1_BYTECK_INVERT, 1);

	if (is_4d1c(port)) {
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANAA8_CSI0A,
			  RG_CSI0A_CDPHY_L0_T0_BYTECK_INVERT, 1);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANAA8_CSI0A,
			  RG_CSI0A_DPHY_L1_BYTECK_INVERT, 1);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANAA8_CSI0A,
			  RG_CSI0A_CDPHY_L2_T1_BYTECK_INVERT, 1);
	}

	/* Start ANA EQ tuning */
	if (is_cdphy_combo(port)) {
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA18_CSI0A,
			  RG_CSI0A_L0_T0AB_EQ_IS, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA18_CSI0A,
			  RG_CSI0A_L0_T0AB_EQ_BW, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA1C_CSI0A,
			  RG_CSI0A_L1_T1AB_EQ_IS, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA1C_CSI0A,
			  RG_CSI0A_L1_T1AB_EQ_BW, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA20_CSI0A,
			  RG_CSI0A_L2_T1BC_EQ_IS, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA20_CSI0A,
			  RG_CSI0A_L2_T1BC_EQ_BW, 1);

		if (is_4d1c(port)) {
			MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA18_CSI0A,
				  RG_CSI0A_L0_T0AB_EQ_IS, 1);
			MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA18_CSI0A,
				  RG_CSI0A_L0_T0AB_EQ_BW, 1);
			MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA1C_CSI0A,
				  RG_CSI0A_L1_T1AB_EQ_IS, 1);
			MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA1C_CSI0A,
				  RG_CSI0A_L1_T1AB_EQ_BW, 1);
			MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA20_CSI0A,
				  RG_CSI0A_L2_T1BC_EQ_IS, 1);
			MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA20_CSI0A,
				  RG_CSI0A_L2_T1BC_EQ_BW, 1);
		}
	} else {
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA18_CSI1A,
			  RG_CSI1A_L0_EQ_IS, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA18_CSI1A,
			  RG_CSI1A_L0_EQ_BW, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA18_CSI1A,
			  RG_CSI1A_L1_EQ_IS, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA18_CSI1A,
			  RG_CSI1A_L1_EQ_BW, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA1C_CSI1A,
			  RG_CSI1A_L2_EQ_IS, 1);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA1C_CSI1A,
			  RG_CSI1A_L2_EQ_BW, 1);

		if (is_4d1c(port)) {
			MIPI_BITS(pmipi_rx + CSI1B_OFST, MIPI_RX_ANA18_CSI1A,
				  RG_CSI1A_L0_EQ_IS, 1);
			MIPI_BITS(pmipi_rx + CSI1B_OFST, MIPI_RX_ANA18_CSI1A,
				  RG_CSI1A_L0_EQ_BW, 1);
			MIPI_BITS(pmipi_rx + CSI1B_OFST, MIPI_RX_ANA18_CSI1A,
				  RG_CSI1A_L1_EQ_IS, 1);
			MIPI_BITS(pmipi_rx + CSI1B_OFST, MIPI_RX_ANA18_CSI1A,
				  RG_CSI1A_L1_EQ_BW, 1);
			MIPI_BITS(pmipi_rx + CSI1B_OFST, MIPI_RX_ANA1C_CSI1A,
				  RG_CSI1A_L2_EQ_IS, 1);
			MIPI_BITS(pmipi_rx + CSI1B_OFST, MIPI_RX_ANA1C_CSI1A,
				  RG_CSI1A_L2_EQ_BW, 1);
		}
	}

	/* End ANA EQ tuning */
	writel(0x90, pmipi_rx_base + MIPI_RX_ANA40_CSI0A);
	MIPI_BITS(pmipi_rx, MIPI_RX_ANA24_CSI0A,
		  RG_CSI0A_RESERVE, 0x40);
	if (is_4d1c(port))
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA24_CSI0A,
			  RG_CSI0A_RESERVE, 0x40);
	MIPI_BITS(pmipi_rx, MIPI_RX_WRAPPER80_CSI0A,
		  CSR_CSI_RST_MODE, 0);
	if (is_4d1c(port))
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_WRAPPER80_CSI0A,
			  CSR_CSI_RST_MODE, 0);
	/* ANA power on */
	MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
		  RG_CSI0A_BG_CORE_EN, 1);
	if (is_4d1c(port))
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_CORE_EN, 1);
	usleep_range(20, 40);
	MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
		  RG_CSI0A_BG_LPF_EN, 1);
	if (is_4d1c(port))
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_LPF_EN, 1);

	return 0;
}

static int mtk_mipi_phy_power_off(struct phy *phy)
{
	struct mtk_mipi_dphy *priv = phy_get_drvdata(phy);

	void __iomem *pmipi_rx = priv->csi2_rx[priv->port];

	/* Disable mipi BG */
	switch (priv->port) {
	case MTK_MIPI_PHY_PORT_0A:
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_CORE_EN, 0);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_LPF_EN, 0);
		break;
	case MTK_MIPI_PHY_PORT_0B:
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_CORE_EN, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_LPF_EN, 0);
		break;
	default:
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_CORE_EN, 0);
		MIPI_BITS(pmipi_rx, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_LPF_EN, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_CORE_EN, 0);
		MIPI_BITS(pmipi_rx + CSI0B_OFST, MIPI_RX_ANA00_CSI0A,
			  RG_CSI0A_BG_LPF_EN, 0);
		break;
	}

	return 0;
}

static const struct phy_ops mtk_dphy_ops = {
	.power_on	= mtk_mipi_phy_power_on,
	.power_off	= mtk_mipi_phy_power_off,
	.owner		= THIS_MODULE,
};

static int mipi_dphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct mtk_mipi_dphy *priv;
	struct phy_provider *phy_provider;
	struct phy *phy;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->rx = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->rx))
		return PTR_ERR(priv->rx);

	priv->csi2_rx[MTK_MIPI_PHY_PORT_0] = priv->rx;
	priv->csi2_rx[MTK_MIPI_PHY_PORT_0A] = priv->rx + CSI_PORT_0A_ADDR_OFST;
	priv->csi2_rx[MTK_MIPI_PHY_PORT_0B] = priv->rx + CSI_PORT_0B_ADDR_OFST;
	priv->csi2_rx[MTK_MIPI_PHY_PORT_1] = priv->rx + CSI_PORT_1_ADDR_OFST;
	priv->csi2_rx[MTK_MIPI_PHY_PORT_2] = priv->rx + CSI_PORT_2_ADDR_OFST;

	/* TODO : As I don't know how to get the sensor port from the DT,
	hard-coded it to 3 here to use the PORT_0A which is a 2-lanes port */
	priv->port = 3;

	phy = devm_phy_create(dev, NULL, &mtk_dphy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create phy\n");
		return PTR_ERR(phy);
	}
	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return 0;
}

static const struct of_device_id mtk_mipi_dphy_of_match[] = {
	{.compatible = "mediatek,mt8183-mipi-dphy"},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_mipi_dphy_of_match);

static struct platform_driver mipi_dphy_pdrv = {
	.probe	= mipi_dphy_probe,
	.driver	= {
		.name	= "mtk-mipi-dphy",
		.of_match_table = of_match_ptr(mtk_mipi_dphy_of_match),
	},
};

module_platform_driver(mipi_dphy_pdrv);

MODULE_DESCRIPTION("MTK mipi dphy driver");
MODULE_AUTHOR("Louis Kuo <louis.kuo@mediatek.com>");
MODULE_LICENSE("GPL v2");
