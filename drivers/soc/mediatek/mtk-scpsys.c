// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 Pengutronix, Sascha Hauer <kernel@pengutronix.de>
 */
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/mediatek/infracfg.h>
#include <linux/soc/mediatek/scpsys-ext.h>

#include <dt-bindings/power/mt2701-power.h>
#include <dt-bindings/power/mt2712-power.h>
#include <dt-bindings/power/mt6797-power.h>
#include <dt-bindings/power/mt7622-power.h>
#include <dt-bindings/power/mt7623a-power.h>
#include <dt-bindings/power/mt8167-power.h>
#include <dt-bindings/power/mt8173-power.h>
#include <dt-bindings/power/mt8183-power.h>

#define MTK_POLL_DELAY_US   10
#define MTK_POLL_TIMEOUT    USEC_PER_SEC

#define MTK_SCPD_ACTIVE_WAKEUP		BIT(0)
#define MTK_SCPD_FWAIT_SRAM		BIT(1)
#define MTK_SCPD_CAPS(_scpd, _x)	((_scpd)->data->caps & (_x))

#define SPM_VDE_PWR_CON			0x0210
#define SPM_MFG_PWR_CON			0x0214
#define SPM_VEN_PWR_CON			0x0230
#define SPM_ISP_PWR_CON			0x0238
#define SPM_DIS_PWR_CON			0x023c
#define SPM_CONN_PWR_CON		0x0280
#define SPM_VEN2_PWR_CON		0x0298
#define SPM_AUDIO_PWR_CON		0x029c	/* MT8173, MT2712 */
#define SPM_BDP_PWR_CON			0x029c	/* MT2701 */
#define SPM_ETH_PWR_CON			0x02a0
#define SPM_HIF_PWR_CON			0x02a4
#define SPM_IFR_MSC_PWR_CON		0x02a8
#define SPM_MFG_2D_PWR_CON		0x02c0
#define SPM_MFG_ASYNC_PWR_CON		0x02c4
#define SPM_USB_PWR_CON			0x02cc
#define SPM_USB2_PWR_CON		0x02d4	/* MT2712 */
#define SPM_ETHSYS_PWR_CON		0x02e0	/* MT7622 */
#define SPM_HIF0_PWR_CON		0x02e4	/* MT7622 */
#define SPM_HIF1_PWR_CON		0x02e8	/* MT7622 */
#define SPM_WB_PWR_CON			0x02ec	/* MT7622 */

#define SPM_PWR_STATUS			0x060c
#define SPM_PWR_STATUS_2ND		0x0610

#define PWR_RST_B_BIT			BIT(0)
#define PWR_ISO_BIT			BIT(1)
#define PWR_ON_BIT			BIT(2)
#define PWR_ON_2ND_BIT			BIT(3)
#define PWR_CLK_DIS_BIT			BIT(4)
#define PWR_SRAM_CLKISO_BIT		BIT(5)
#define PWR_SRAM_ISOINT_B_BIT		BIT(6)

#define PWR_STATUS_CONN			BIT(1)
#define PWR_STATUS_DISP			BIT(3)
#define PWR_STATUS_MFG			BIT(4)
#define PWR_STATUS_ISP			BIT(5)
#define PWR_STATUS_VDEC			BIT(7)
#define PWR_STATUS_BDP			BIT(14)
#define PWR_STATUS_ETH			BIT(15)
#define PWR_STATUS_HIF			BIT(16)
#define PWR_STATUS_IFR_MSC		BIT(17)
#define PWR_STATUS_USB2			BIT(19)	/* MT2712 */
#define PWR_STATUS_VENC_LT		BIT(20)
#define PWR_STATUS_VENC			BIT(21)
#define PWR_STATUS_MFG_2D		BIT(22)	/* MT8173 */
#define PWR_STATUS_MFG_ASYNC		BIT(23)	/* MT8173 */
#define PWR_STATUS_AUDIO		BIT(24)	/* MT8173, MT2712 */
#define PWR_STATUS_USB			BIT(25)	/* MT8173, MT2712 */
#define PWR_STATUS_ETHSYS		BIT(24)	/* MT7622 */
#define PWR_STATUS_HIF0			BIT(25)	/* MT7622 */
#define PWR_STATUS_HIF1			BIT(26)	/* MT7622 */
#define PWR_STATUS_WB			BIT(27)	/* MT7622 */

enum clk_id {
	CLK_NONE,
	CLK_MM,
	CLK_MFG,
	CLK_VENC,
	CLK_VENC_LT,
	CLK_ETHIF,
	CLK_VDEC,
	CLK_HIFSEL,
	CLK_JPGDEC,
	CLK_AUDIO,
	CLK_AXI_MFG,
	CLK_MAX,
};

static const char * const clk_names[] = {
	NULL,
	"mm",
	"mfg",
	"venc",
	"venc_lt",
	"ethif",
	"vdec",
	"hif_sel",
	"jpgdec",
	"audio",
	NULL,
};

#define MAX_CLKS	3
#define MAX_SUBSYS_CLKS 10

/**
 * struct scp_domain_data - scp domain data for power on/off flow
 * @name: The domain name.
 * @sta_mask: The mask for power on/off status bit.
 * @ctl_offs: The offset for main power control register.
 * @sram_iso_ctrl: The flag to judge if the power domain need to do
 *                 the extra sram isolation control.
 * @sram_pdn_bits: The mask for sram power control bits.
 * @sram_pdn_ack_bits: The mask for sram power control acked bits.
 * @bus_prot_mask: The mask for single step bus protection.
 * @clk_id: The basic clocks required by this power domain.
 * @basic_clk_id: provide the same purpose with field "clk_id"
 *                by declaring basic clock prefix name rather than clk_id.
 * @subsys_clk_prefix: The prefix name of the clocks need to be enabled
 *                     before releasing bus protection.
 * @caps: The flag for active wake-up action.
 * @bp_table: The mask table for multiple step bus protection.
 */
struct scp_domain_data {
	const char *name;
	u32 sta_mask;
	int ctl_offs;
	bool sram_iso_ctrl;
	u32 sram_pdn_bits;
	u32 sram_pdn_ack_bits;
	u32 bus_prot_mask;
	enum clk_id clk_id[MAX_CLKS];
	const char *basic_clk_id[MAX_CLKS];
	const char *subsys_clk_prefix;
	u8 caps;
	struct bus_prot bp_table[MAX_STEPS];
};

struct scp;

struct scp_domain {
	struct generic_pm_domain genpd;
	struct scp *scp;
	struct clk *clk[MAX_CLKS];
	struct clk *subsys_clk[MAX_SUBSYS_CLKS];
	const struct scp_domain_data *data;
	struct regulator *supply;
};

struct scp_ctrl_reg {
	int pwr_sta_offs;
	int pwr_sta2nd_offs;
};

struct scp {
	struct scp_domain *domains;
	struct genpd_onecell_data pd_data;
	struct device *dev;
	void __iomem *base;
	struct regmap *infracfg;
	struct regmap *smi_common;
	struct scp_ctrl_reg ctrl_reg;
	bool bus_prot_reg_update;
};

struct scp_subdomain {
	int origin;
	int subdomain;
};

struct scp_soc_data {
	const struct scp_domain_data *domains;
	int num_domains;
	const struct scp_subdomain *subdomains;
	int num_subdomains;
	const struct scp_ctrl_reg regs;
	bool bus_prot_reg_update;
};

static int scpsys_domain_is_on(struct scp_domain *scpd)
{
	struct scp *scp = scpd->scp;

	u32 status = readl(scp->base + scp->ctrl_reg.pwr_sta_offs) &
						scpd->data->sta_mask;
	u32 status2 = readl(scp->base + scp->ctrl_reg.pwr_sta2nd_offs) &
						scpd->data->sta_mask;

	/*
	 * A domain is on when both status bits are set. If only one is set
	 * return an error. This happens while powering up a domain
	 */

	if (status && status2)
		return true;
	if (!status && !status2)
		return false;

	return -EINVAL;
}

static int scpsys_regulator_enable(struct scp_domain *scpd)
{
	if (!scpd->supply)
		return 0;

	return regulator_enable(scpd->supply);
}

static int scpsys_regulator_disable(struct scp_domain *scpd)
{
	if (!scpd->supply)
		return 0;

	return regulator_disable(scpd->supply);
}

static void scpsys_clk_disable(struct clk *clk[], int max_num)
{
	int i;

	for (i = max_num - 1; i >= 0; i--)
		clk_disable_unprepare(clk[i]);
}

static int scpsys_clk_enable(struct clk *clk[], int max_num)
{
	int i, ret = 0;

	for (i = 0; i < max_num && clk[i]; i++) {
		ret = clk_prepare_enable(clk[i]);
		if (ret) {
			scpsys_clk_disable(clk, i);
			break;
		}
	}

	return ret;
}

static int scpsys_sram_enable(struct scp_domain *scpd, void __iomem *ctl_addr)
{
	u32 val;
	u32 pdn_ack = scpd->data->sram_pdn_ack_bits;
	int tmp;

	val = readl(ctl_addr) & ~scpd->data->sram_pdn_bits;
	writel(val, ctl_addr);

	/* Either wait until SRAM_PDN_ACK all 0 or have a force wait */
	if (MTK_SCPD_CAPS(scpd, MTK_SCPD_FWAIT_SRAM)) {
		/*
		 * Currently, MTK_SCPD_FWAIT_SRAM is necessary only for
		 * MT7622_POWER_DOMAIN_WB and thus just a trivial setup
		 * is applied here.
		 */
		usleep_range(12000, 12100);
	} else {
		/* Either wait until SRAM_PDN_ACK all 1 or 0 */
		int ret = readl_poll_timeout(ctl_addr, tmp,
				(tmp & pdn_ack) == 0,
				MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
		if (ret < 0)
			return ret;
	}

	if (scpd->data->sram_iso_ctrl)	{
		val = readl(ctl_addr) | PWR_SRAM_ISOINT_B_BIT;
		writel(val, ctl_addr);
		udelay(1);
		val &= ~PWR_SRAM_CLKISO_BIT;
		writel(val, ctl_addr);
	}

	return 0;
}

static int scpsys_sram_disable(struct scp_domain *scpd, void __iomem *ctl_addr)
{
	u32 val;
	u32 pdn_ack = scpd->data->sram_pdn_ack_bits;
	int tmp;

	if (scpd->data->sram_iso_ctrl)	{
		val = readl(ctl_addr);
		val |= PWR_SRAM_CLKISO_BIT;
		writel(val, ctl_addr);
		val &= ~PWR_SRAM_ISOINT_B_BIT;
		writel(val, ctl_addr);
		udelay(1);
	}

	val = readl(ctl_addr) | scpd->data->sram_pdn_bits;
	writel(val, ctl_addr);

	/* Either wait until SRAM_PDN_ACK all 1 or 0 */
	return readl_poll_timeout(ctl_addr, tmp,
			(tmp & pdn_ack) == pdn_ack,
			MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
}

static int scpsys_bus_protect_enable(struct scp_domain *scpd)
{
	struct scp *scp = scpd->scp;

	if (scpd->data->bus_prot_mask) {
		return mtk_infracfg_set_bus_protection(scp->infracfg,
				scpd->data->bus_prot_mask,
				scp->bus_prot_reg_update);
	}

	return mtk_scpsys_ext_set_bus_protection(scpd->data->bp_table,
			scp->infracfg, scp->smi_common);
}

static int scpsys_bus_protect_disable(struct scp_domain *scpd)
{
	struct scp *scp = scpd->scp;

	if (scpd->data->bus_prot_mask) {
		return mtk_infracfg_clear_bus_protection(scp->infracfg,
				scpd->data->bus_prot_mask,
				scp->bus_prot_reg_update);
	}

	return mtk_scpsys_ext_clear_bus_protection(scpd->data->bp_table,
			scp->infracfg, scp->smi_common);
}

static int scpsys_power_on(struct generic_pm_domain *genpd)
{
	struct scp_domain *scpd = container_of(genpd, struct scp_domain, genpd);
	struct scp *scp = scpd->scp;
	void __iomem *ctl_addr = scp->base + scpd->data->ctl_offs;
	u32 val;
	int ret, tmp;

	ret = scpsys_regulator_enable(scpd);
	if (ret < 0)
		return ret;

	ret = scpsys_clk_enable(scpd->clk, MAX_CLKS);
	if (ret)
		goto err_clk;

	/* subsys power on */
	val = readl(ctl_addr);
	val |= PWR_ON_BIT;
	writel(val, ctl_addr);
	val |= PWR_ON_2ND_BIT;
	writel(val, ctl_addr);

	/* wait until PWR_ACK = 1 */
	ret = readx_poll_timeout(scpsys_domain_is_on, scpd, tmp, tmp > 0,
				 MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
	if (ret < 0)
		goto err_pwr_ack;

	val &= ~PWR_CLK_DIS_BIT;
	writel(val, ctl_addr);

	val &= ~PWR_ISO_BIT;
	writel(val, ctl_addr);

	val |= PWR_RST_B_BIT;
	writel(val, ctl_addr);

	ret = scpsys_clk_enable(scpd->subsys_clk, MAX_SUBSYS_CLKS);
	if (ret < 0)
		goto err_pwr_ack;

	ret = scpsys_sram_enable(scpd, ctl_addr);
	if (ret < 0)
		goto err_sram;

	ret = scpsys_bus_protect_disable(scpd);
	if (ret < 0)
		goto err_sram;

	return 0;

err_sram:
	scpsys_clk_disable(scpd->subsys_clk, MAX_SUBSYS_CLKS);
err_pwr_ack:
	scpsys_clk_disable(scpd->clk, MAX_CLKS);
err_clk:
	scpsys_regulator_disable(scpd);

	dev_err(scp->dev, "Failed to power on domain %s\n", genpd->name);

	return ret;
}

static int scpsys_power_off(struct generic_pm_domain *genpd)
{
	struct scp_domain *scpd = container_of(genpd, struct scp_domain, genpd);
	struct scp *scp = scpd->scp;
	void __iomem *ctl_addr = scp->base + scpd->data->ctl_offs;
	u32 val;
	int ret, tmp;

	ret = scpsys_bus_protect_enable(scpd);
	if (ret < 0)
		goto out;

	ret = scpsys_sram_disable(scpd, ctl_addr);
	if (ret < 0)
		goto out;

	scpsys_clk_disable(scpd->subsys_clk, MAX_SUBSYS_CLKS);

	/* subsys power off */
	val = readl(ctl_addr) | PWR_ISO_BIT;
	writel(val, ctl_addr);

	val &= ~PWR_RST_B_BIT;
	writel(val, ctl_addr);

	val |= PWR_CLK_DIS_BIT;
	writel(val, ctl_addr);

	val &= ~PWR_ON_BIT;
	writel(val, ctl_addr);

	val &= ~PWR_ON_2ND_BIT;
	writel(val, ctl_addr);

	/* wait until PWR_ACK = 0 */
	ret = readx_poll_timeout(scpsys_domain_is_on, scpd, tmp, tmp == 0,
				 MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
	if (ret < 0)
		goto out;

	scpsys_clk_disable(scpd->clk, MAX_CLKS);

	ret = scpsys_regulator_disable(scpd);
	if (ret < 0)
		goto out;

	return 0;

out:
	dev_err(scp->dev, "Failed to power off domain %s\n", genpd->name);

	return ret;
}

static int init_subsys_clks(struct platform_device *pdev,
		const char *prefix, struct clk **clk)
{
	struct device_node *node = pdev->dev.of_node;
	u32 prefix_len, sub_clk_cnt = 0;
	struct property *prop;
	const char *clk_name;

	if (!node) {
		dev_err(&pdev->dev, "Cannot find scpsys node: %ld\n",
			PTR_ERR(node));
		return PTR_ERR(node);
	}

	prefix_len = strlen(prefix);

	of_property_for_each_string(node, "clock-names", prop, clk_name) {
		if (!strncmp(clk_name, prefix, prefix_len) &&
				(clk_name[prefix_len] == '-')) {
			if (sub_clk_cnt >= MAX_SUBSYS_CLKS) {
				dev_err(&pdev->dev,
					"subsys clk out of range %d\n",
					sub_clk_cnt);
				return -ENOMEM;
			}

			clk[sub_clk_cnt] = devm_clk_get(&pdev->dev,
						clk_name);

			if (IS_ERR(clk)) {
				dev_err(&pdev->dev,
					"Subsys clk read fail %ld\n",
					PTR_ERR(clk));
				return PTR_ERR(clk);
			}
			sub_clk_cnt++;
		}
	}

	return sub_clk_cnt;
}

static void init_clks(struct platform_device *pdev, struct clk **clk)
{
	int i;

	for (i = CLK_NONE + 1; i < CLK_MAX; i++)
		clk[i] = devm_clk_get(&pdev->dev, clk_names[i]);
}

static struct scp *init_scp(struct platform_device *pdev,
			const struct scp_domain_data *scp_domain_data, int num,
			const struct scp_ctrl_reg *scp_ctrl_reg,
			bool bus_prot_reg_update)
{
	struct genpd_onecell_data *pd_data;
	struct resource *res;
	int i, j;
	struct scp *scp;
	struct clk *clk[CLK_MAX];

	scp = devm_kzalloc(&pdev->dev, sizeof(*scp), GFP_KERNEL);
	if (!scp)
		return ERR_PTR(-ENOMEM);

	scp->ctrl_reg.pwr_sta_offs = scp_ctrl_reg->pwr_sta_offs;
	scp->ctrl_reg.pwr_sta2nd_offs = scp_ctrl_reg->pwr_sta2nd_offs;

	scp->bus_prot_reg_update = bus_prot_reg_update;

	scp->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	scp->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(scp->base))
		return ERR_CAST(scp->base);

	scp->domains = devm_kcalloc(&pdev->dev,
				num, sizeof(*scp->domains), GFP_KERNEL);
	if (!scp->domains)
		return ERR_PTR(-ENOMEM);

	pd_data = &scp->pd_data;

	pd_data->domains = devm_kcalloc(&pdev->dev,
			num, sizeof(*pd_data->domains), GFP_KERNEL);
	if (!pd_data->domains)
		return ERR_PTR(-ENOMEM);

	scp->infracfg = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
			"infracfg");
	if (IS_ERR(scp->infracfg)) {
		dev_err(&pdev->dev, "Cannot find infracfg controller: %ld\n",
				PTR_ERR(scp->infracfg));
		return ERR_CAST(scp->infracfg);
	}

	scp->smi_common = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
			"smi_comm");

	if (scp->smi_common == ERR_PTR(-ENODEV)) {
		scp->smi_common = NULL;
	} else if (IS_ERR(scp->smi_common)) {
		dev_err(&pdev->dev, "Cannot find smi_common controller: %ld\n",
				PTR_ERR(scp->smi_common));
		return ERR_CAST(scp->smi_common);
	}

	for (i = 0; i < num; i++) {
		struct scp_domain *scpd = &scp->domains[i];
		const struct scp_domain_data *data = &scp_domain_data[i];

		scpd->supply = devm_regulator_get_optional(&pdev->dev, data->name);
		if (IS_ERR(scpd->supply)) {
			if (PTR_ERR(scpd->supply) == -ENODEV)
				scpd->supply = NULL;
			else
				return ERR_CAST(scpd->supply);
		}
	}

	pd_data->num_domains = num;

	init_clks(pdev, clk);

	for (i = 0; i < num; i++) {
		struct scp_domain *scpd = &scp->domains[i];
		struct generic_pm_domain *genpd = &scpd->genpd;
		const struct scp_domain_data *data = &scp_domain_data[i];
		int clk_cnt;

		pd_data->domains[i] = genpd;
		scpd->scp = scp;

		scpd->data = data;

		if (data->clk_id[0]) {
			WARN_ON(data->basic_clk_id[0]);

			for (j = 0; j < MAX_CLKS && data->clk_id[j]; j++) {
				struct clk *c = clk[data->clk_id[j]];

				if (IS_ERR(c)) {
					dev_err(&pdev->dev,
						"%s: clk unavailable\n",
						data->name);
					return ERR_CAST(c);
				}

				scpd->clk[j] = c;
			}
		} else if (data->basic_clk_id[0]) {
			for (j = 0; j < MAX_CLKS &&
					data->basic_clk_id[j]; j++)
				scpd->clk[j] = devm_clk_get(&pdev->dev,
						data->basic_clk_id[j]);
		}

		if (data->subsys_clk_prefix) {
			clk_cnt = init_subsys_clks(pdev,
					data->subsys_clk_prefix,
					scpd->subsys_clk);
			if (clk_cnt < 0) {
				dev_err(&pdev->dev,
					"%s: subsys clk unavailable\n",
					data->name);
				return ERR_PTR(clk_cnt);
			}
		}

		genpd->name = data->name;
		genpd->power_off = scpsys_power_off;
		genpd->power_on = scpsys_power_on;
		if (MTK_SCPD_CAPS(scpd, MTK_SCPD_ACTIVE_WAKEUP))
			genpd->flags |= GENPD_FLAG_ACTIVE_WAKEUP;
	}

	return scp;
}

static void mtk_register_power_domains(struct platform_device *pdev,
				struct scp *scp, int num)
{
	struct genpd_onecell_data *pd_data;
	int i, ret;

	for (i = 0; i < num; i++) {
		struct scp_domain *scpd = &scp->domains[i];
		struct generic_pm_domain *genpd = &scpd->genpd;
		bool on;

		/*
		 * Initially turn on all domains to make the domains usable
		 * with !CONFIG_PM and to get the hardware in sync with the
		 * software.  The unused domains will be switched off during
		 * late_init time.
		 */
		on = !WARN_ON(genpd->power_on(genpd) < 0);

		pm_genpd_init(genpd, NULL, !on);
	}

	/*
	 * We are not allowed to fail here since there is no way to unregister
	 * a power domain. Once registered above we have to keep the domains
	 * valid.
	 */

	pd_data = &scp->pd_data;

	ret = of_genpd_add_provider_onecell(pdev->dev.of_node, pd_data);
	if (ret)
		dev_err(&pdev->dev, "Failed to add OF provider: %d\n", ret);
}

/*
 * MT2701 power domain support
 */

static const struct scp_domain_data scp_domain_data_mt2701[] = {
	[MT2701_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = PWR_STATUS_CONN,
		.ctl_offs = SPM_CONN_PWR_CON,
		.bus_prot_mask = MT2701_TOP_AXI_PROT_EN_CONN_M |
				 MT2701_TOP_AXI_PROT_EN_CONN_S,
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2701_POWER_DOMAIN_DISP] = {
		.name = "disp",
		.sta_mask = PWR_STATUS_DISP,
		.ctl_offs = SPM_DIS_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.clk_id = {CLK_MM},
		.bus_prot_mask = MT2701_TOP_AXI_PROT_EN_MM_M0,
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2701_POWER_DOMAIN_MFG] = {
		.name = "mfg",
		.sta_mask = PWR_STATUS_MFG,
		.ctl_offs = SPM_MFG_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_MFG},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2701_POWER_DOMAIN_VDEC] = {
		.name = "vdec",
		.sta_mask = PWR_STATUS_VDEC,
		.ctl_offs = SPM_VDE_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_MM},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2701_POWER_DOMAIN_ISP] = {
		.name = "isp",
		.sta_mask = PWR_STATUS_ISP,
		.ctl_offs = SPM_ISP_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.clk_id = {CLK_MM},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2701_POWER_DOMAIN_BDP] = {
		.name = "bdp",
		.sta_mask = PWR_STATUS_BDP,
		.ctl_offs = SPM_BDP_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2701_POWER_DOMAIN_ETH] = {
		.name = "eth",
		.sta_mask = PWR_STATUS_ETH,
		.ctl_offs = SPM_ETH_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_ETHIF},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2701_POWER_DOMAIN_HIF] = {
		.name = "hif",
		.sta_mask = PWR_STATUS_HIF,
		.ctl_offs = SPM_HIF_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_ETHIF},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2701_POWER_DOMAIN_IFR_MSC] = {
		.name = "ifr_msc",
		.sta_mask = PWR_STATUS_IFR_MSC,
		.ctl_offs = SPM_IFR_MSC_PWR_CON,
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
};

/*
 * MT2712 power domain support
 */
static const struct scp_domain_data scp_domain_data_mt2712[] = {
	[MT2712_POWER_DOMAIN_MM] = {
		.name = "mm",
		.sta_mask = PWR_STATUS_DISP,
		.ctl_offs = SPM_DIS_PWR_CON,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_MM},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_VDEC] = {
		.name = "vdec",
		.sta_mask = PWR_STATUS_VDEC,
		.ctl_offs = SPM_VDE_PWR_CON,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_MM, CLK_VDEC},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_VENC] = {
		.name = "venc",
		.sta_mask = PWR_STATUS_VENC,
		.ctl_offs = SPM_VEN_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_MM, CLK_VENC, CLK_JPGDEC},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_ISP] = {
		.name = "isp",
		.sta_mask = PWR_STATUS_ISP,
		.ctl_offs = SPM_ISP_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.clk_id = {CLK_MM},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		.sta_mask = PWR_STATUS_AUDIO,
		.ctl_offs = SPM_AUDIO_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_AUDIO},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_USB] = {
		.name = "usb",
		.sta_mask = PWR_STATUS_USB,
		.ctl_offs = SPM_USB_PWR_CON,
		.sram_pdn_bits = GENMASK(10, 8),
		.sram_pdn_ack_bits = GENMASK(14, 12),
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_USB2] = {
		.name = "usb2",
		.sta_mask = PWR_STATUS_USB2,
		.ctl_offs = SPM_USB2_PWR_CON,
		.sram_pdn_bits = GENMASK(10, 8),
		.sram_pdn_ack_bits = GENMASK(14, 12),
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_MFG] = {
		.name = "mfg",
		.sta_mask = PWR_STATUS_MFG,
		.ctl_offs = SPM_MFG_PWR_CON,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(16, 16),
		.clk_id = {CLK_MFG},
		.bus_prot_mask = BIT(14) | BIT(21) | BIT(23),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_MFG_SC1] = {
		.name = "mfg_sc1",
		.sta_mask = BIT(22),
		.ctl_offs = 0x02c0,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(16, 16),
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_MFG_SC2] = {
		.name = "mfg_sc2",
		.sta_mask = BIT(23),
		.ctl_offs = 0x02c4,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(16, 16),
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT2712_POWER_DOMAIN_MFG_SC3] = {
		.name = "mfg_sc3",
		.sta_mask = BIT(30),
		.ctl_offs = 0x01f8,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(16, 16),
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
};

static const struct scp_subdomain scp_subdomain_mt2712[] = {
	{MT2712_POWER_DOMAIN_MM, MT2712_POWER_DOMAIN_VDEC},
	{MT2712_POWER_DOMAIN_MM, MT2712_POWER_DOMAIN_VENC},
	{MT2712_POWER_DOMAIN_MM, MT2712_POWER_DOMAIN_ISP},
	{MT2712_POWER_DOMAIN_MFG, MT2712_POWER_DOMAIN_MFG_SC1},
	{MT2712_POWER_DOMAIN_MFG_SC1, MT2712_POWER_DOMAIN_MFG_SC2},
	{MT2712_POWER_DOMAIN_MFG_SC2, MT2712_POWER_DOMAIN_MFG_SC3},
};

/*
 * MT6797 power domain support
 */

static const struct scp_domain_data scp_domain_data_mt6797[] = {
	[MT6797_POWER_DOMAIN_VDEC] = {
		.name = "vdec",
		.sta_mask = BIT(7),
		.ctl_offs = 0x300,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_VDEC},
	},
	[MT6797_POWER_DOMAIN_VENC] = {
		.name = "venc",
		.sta_mask = BIT(21),
		.ctl_offs = 0x304,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_NONE},
	},
	[MT6797_POWER_DOMAIN_ISP] = {
		.name = "isp",
		.sta_mask = BIT(5),
		.ctl_offs = 0x308,
		.sram_pdn_bits = GENMASK(9, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.clk_id = {CLK_NONE},
	},
	[MT6797_POWER_DOMAIN_MM] = {
		.name = "mm",
		.sta_mask = BIT(3),
		.ctl_offs = 0x30C,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_MM},
		.bus_prot_mask = (BIT(1) | BIT(2)),
	},
	[MT6797_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		.sta_mask = BIT(24),
		.ctl_offs = 0x314,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_NONE},
	},
	[MT6797_POWER_DOMAIN_MFG_ASYNC] = {
		.name = "mfg_async",
		.sta_mask = BIT(13),
		.ctl_offs = 0x334,
		.sram_pdn_bits = 0,
		.sram_pdn_ack_bits = 0,
		.clk_id = {CLK_MFG},
	},
	[MT6797_POWER_DOMAIN_MJC] = {
		.name = "mjc",
		.sta_mask = BIT(20),
		.ctl_offs = 0x310,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_NONE},
	},
};

#define SPM_PWR_STATUS_MT6797		0x0180
#define SPM_PWR_STATUS_2ND_MT6797	0x0184

static const struct scp_subdomain scp_subdomain_mt6797[] = {
	{MT6797_POWER_DOMAIN_MM, MT6797_POWER_DOMAIN_VDEC},
	{MT6797_POWER_DOMAIN_MM, MT6797_POWER_DOMAIN_ISP},
	{MT6797_POWER_DOMAIN_MM, MT6797_POWER_DOMAIN_VENC},
	{MT6797_POWER_DOMAIN_MM, MT6797_POWER_DOMAIN_MJC},
};

/*
 * MT7622 power domain support
 */

static const struct scp_domain_data scp_domain_data_mt7622[] = {
	[MT7622_POWER_DOMAIN_ETHSYS] = {
		.name = "ethsys",
		.sta_mask = PWR_STATUS_ETHSYS,
		.ctl_offs = SPM_ETHSYS_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_NONE},
		.bus_prot_mask = MT7622_TOP_AXI_PROT_EN_ETHSYS,
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT7622_POWER_DOMAIN_HIF0] = {
		.name = "hif0",
		.sta_mask = PWR_STATUS_HIF0,
		.ctl_offs = SPM_HIF0_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_HIFSEL},
		.bus_prot_mask = MT7622_TOP_AXI_PROT_EN_HIF0,
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT7622_POWER_DOMAIN_HIF1] = {
		.name = "hif1",
		.sta_mask = PWR_STATUS_HIF1,
		.ctl_offs = SPM_HIF1_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_HIFSEL},
		.bus_prot_mask = MT7622_TOP_AXI_PROT_EN_HIF1,
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT7622_POWER_DOMAIN_WB] = {
		.name = "wb",
		.sta_mask = PWR_STATUS_WB,
		.ctl_offs = SPM_WB_PWR_CON,
		.sram_pdn_bits = 0,
		.sram_pdn_ack_bits = 0,
		.clk_id = {CLK_NONE},
		.bus_prot_mask = MT7622_TOP_AXI_PROT_EN_WB,
		.caps = MTK_SCPD_ACTIVE_WAKEUP | MTK_SCPD_FWAIT_SRAM,
	},
};

/*
 * MT7623A power domain support
 */

static const struct scp_domain_data scp_domain_data_mt7623a[] = {
	[MT7623A_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = PWR_STATUS_CONN,
		.ctl_offs = SPM_CONN_PWR_CON,
		.bus_prot_mask = MT2701_TOP_AXI_PROT_EN_CONN_M |
				 MT2701_TOP_AXI_PROT_EN_CONN_S,
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT7623A_POWER_DOMAIN_ETH] = {
		.name = "eth",
		.sta_mask = PWR_STATUS_ETH,
		.ctl_offs = SPM_ETH_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_ETHIF},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT7623A_POWER_DOMAIN_HIF] = {
		.name = "hif",
		.sta_mask = PWR_STATUS_HIF,
		.ctl_offs = SPM_HIF_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_ETHIF},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT7623A_POWER_DOMAIN_IFR_MSC] = {
		.name = "ifr_msc",
		.sta_mask = PWR_STATUS_IFR_MSC,
		.ctl_offs = SPM_IFR_MSC_PWR_CON,
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
};

/*
 * MT8167 power domain support
 */
#define PWR_STATUS_MFG_2D_MT8167	BIT(24)	/* MT8167 */
#define PWR_STATUS_MFG_ASYNC_MT8167	BIT(25)	/* MT8167 */

static const struct scp_domain_data scp_domain_data_mt8167[] = {
	[MT8167_POWER_DOMAIN_DISP] = {
		.name = "disp",
		.sta_mask = PWR_STATUS_DISP,
		.ctl_offs = SPM_DIS_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bus_prot_mask = BIT(1) | BIT(11),
		.clk_id = {CLK_MM},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8167_POWER_DOMAIN_VDEC] = {
		.name = "vdec",
		.sta_mask = PWR_STATUS_VDEC,
		.ctl_offs = SPM_VDE_PWR_CON,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_MM, CLK_VDEC},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8167_POWER_DOMAIN_ISP] = {
		.name = "isp",
		.sta_mask = PWR_STATUS_ISP,
		.ctl_offs = SPM_ISP_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.clk_id = {CLK_MM},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8167_POWER_DOMAIN_MFG_ASYNC] = {
		.name = "mfg_async",
		.sta_mask = PWR_STATUS_MFG_ASYNC_MT8167,
		.ctl_offs = SPM_MFG_ASYNC_PWR_CON,
		.sram_pdn_bits = 0,
		.sram_pdn_ack_bits = 0,
		.bus_prot_mask = BIT(2) | BIT(5),
//		.axi_si1_way_en = BIT(7), XXX: FIXME
		.clk_id = {CLK_MFG, CLK_AXI_MFG},
	},
	[MT8167_POWER_DOMAIN_MFG_2D] = {
		.name = "mfg_2d",
		.sta_mask = PWR_STATUS_MFG_2D_MT8167,
		.ctl_offs = SPM_MFG_2D_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_NONE},
	},
	[MT8167_POWER_DOMAIN_MFG] = {
		.name = "mfg",
		.sta_mask = PWR_STATUS_MFG,
		.ctl_offs = SPM_MFG_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_NONE},
	},
	[MT8167_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = PWR_STATUS_CONN,
		.ctl_offs = SPM_CONN_PWR_CON,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = 0,
		.bus_prot_mask = BIT(4) | BIT(8) | BIT(9),
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
};

static const struct scp_subdomain scp_subdomain_mt8167[] = {
	{MT8167_POWER_DOMAIN_MFG_ASYNC, MT8167_POWER_DOMAIN_MFG_2D},
	{MT8167_POWER_DOMAIN_MFG_2D, MT8167_POWER_DOMAIN_MFG},
};

/*
 * MT8173 power domain support
 */

static const struct scp_domain_data scp_domain_data_mt8173[] = {
	[MT8173_POWER_DOMAIN_VDEC] = {
		.name = "vdec",
		.sta_mask = PWR_STATUS_VDEC,
		.ctl_offs = SPM_VDE_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_MM},
	},
	[MT8173_POWER_DOMAIN_VENC] = {
		.name = "venc",
		.sta_mask = PWR_STATUS_VENC,
		.ctl_offs = SPM_VEN_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_MM, CLK_VENC},
	},
	[MT8173_POWER_DOMAIN_ISP] = {
		.name = "isp",
		.sta_mask = PWR_STATUS_ISP,
		.ctl_offs = SPM_ISP_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.clk_id = {CLK_MM},
	},
	[MT8173_POWER_DOMAIN_MM] = {
		.name = "mm",
		.sta_mask = PWR_STATUS_DISP,
		.ctl_offs = SPM_DIS_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.clk_id = {CLK_MM},
		.bus_prot_mask = MT8173_TOP_AXI_PROT_EN_MM_M0 |
			MT8173_TOP_AXI_PROT_EN_MM_M1,
	},
	[MT8173_POWER_DOMAIN_VENC_LT] = {
		.name = "venc_lt",
		.sta_mask = PWR_STATUS_VENC_LT,
		.ctl_offs = SPM_VEN2_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_MM, CLK_VENC_LT},
	},
	[MT8173_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		.sta_mask = PWR_STATUS_AUDIO,
		.ctl_offs = SPM_AUDIO_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_NONE},
	},
	[MT8173_POWER_DOMAIN_USB] = {
		.name = "usb",
		.sta_mask = PWR_STATUS_USB,
		.ctl_offs = SPM_USB_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.clk_id = {CLK_NONE},
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8173_POWER_DOMAIN_MFG_ASYNC] = {
		.name = "mfg_async",
		.sta_mask = PWR_STATUS_MFG_ASYNC,
		.ctl_offs = SPM_MFG_ASYNC_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = 0,
		.clk_id = {CLK_MFG},
	},
	[MT8173_POWER_DOMAIN_MFG_2D] = {
		.name = "mfg_2d",
		.sta_mask = PWR_STATUS_MFG_2D,
		.ctl_offs = SPM_MFG_2D_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.clk_id = {CLK_NONE},
	},
	[MT8173_POWER_DOMAIN_MFG] = {
		.name = "mfg",
		.sta_mask = PWR_STATUS_MFG,
		.ctl_offs = SPM_MFG_PWR_CON,
		.sram_pdn_bits = GENMASK(13, 8),
		.sram_pdn_ack_bits = GENMASK(21, 16),
		.clk_id = {CLK_NONE},
		.bus_prot_mask = MT8173_TOP_AXI_PROT_EN_MFG_S |
			MT8173_TOP_AXI_PROT_EN_MFG_M0 |
			MT8173_TOP_AXI_PROT_EN_MFG_M1 |
			MT8173_TOP_AXI_PROT_EN_MFG_SNOOP_OUT,
	},
};

static const struct scp_subdomain scp_subdomain_mt8173[] = {
	{MT8173_POWER_DOMAIN_MFG_ASYNC, MT8173_POWER_DOMAIN_MFG_2D},
	{MT8173_POWER_DOMAIN_MFG_2D, MT8173_POWER_DOMAIN_MFG},
};

/*
 * MT8183 power domain support
 */

static const struct scp_domain_data scp_domain_data_mt8183[] = {
	[MT8183_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		.sta_mask = PWR_STATUS_AUDIO,
		.ctl_offs = 0x0314,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.basic_clk_id = {"audio", "audio1", "audio2"},
	},
	[MT8183_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = PWR_STATUS_CONN,
		.ctl_offs = 0x032c,
		.sram_pdn_bits = 0,
		.sram_pdn_ack_bits = 0,
		.bp_table = {
			BUS_PROT(IFR_TYPE, 0x2a0, 0x2a4, 0, 0x228,
				BIT(13) | BIT(14), BIT(13) | BIT(14)),
		},
	},
	[MT8183_POWER_DOMAIN_MFG_ASYNC] = {
		.name = "mfg_async",
		.sta_mask = PWR_STATUS_MFG_ASYNC,
		.ctl_offs = 0x0334,
		.sram_pdn_bits = 0,
		.sram_pdn_ack_bits = 0,
		.basic_clk_id = {"mfg"},
	},
	[MT8183_POWER_DOMAIN_MFG] = {
		.name = "mfg",
		.sta_mask = PWR_STATUS_MFG,
		.ctl_offs = 0x0338,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
	[MT8183_POWER_DOMAIN_MFG_CORE0] = {
		.name = "mfg_core0",
		.sta_mask = BIT(7),
		.ctl_offs = 0x034c,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
	[MT8183_POWER_DOMAIN_MFG_CORE1] = {
		.name = "mfg_core1",
		.sta_mask = BIT(20),
		.ctl_offs = 0x0310,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
	[MT8183_POWER_DOMAIN_MFG_2D] = {
		.name = "mfg_2d",
		.sta_mask = PWR_STATUS_MFG_2D,
		.ctl_offs = 0x0348,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_table = {
			BUS_PROT(IFR_TYPE, 0x2a8, 0x2ac, 0, 0x258,
				BIT(19) | BIT(20) | BIT(21),
				BIT(19) | BIT(20) | BIT(21)),
			BUS_PROT(IFR_TYPE, 0x2a0, 0x2a4, 0, 0x228,
				BIT(21) | BIT(22), BIT(21) | BIT(22)),
		},
	},
	[MT8183_POWER_DOMAIN_DISP] = {
		.name = "disp",
		.sta_mask = PWR_STATUS_DISP,
		.ctl_offs = 0x030c,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_id = {"mm"},
		.subsys_clk_prefix = "mm",
		.bp_table = {
			BUS_PROT(IFR_TYPE, 0x2a8, 0x2ac, 0, 0x258,
				BIT(16) | BIT(17), BIT(16) | BIT(17)),
			BUS_PROT(IFR_TYPE, 0x2a0, 0x2a4, 0, 0x228,
				BIT(10) | BIT(11), BIT(10) | BIT(11)),
			BUS_PROT(SMI_TYPE, 0x3c4, 0x3c8, 0, 0x3c0,
				GENMASK(7, 0), GENMASK(7, 0)),
		},
	},
	[MT8183_POWER_DOMAIN_CAM] = {
		.name = "cam",
		.sta_mask = BIT(25),
		.ctl_offs = 0x0344,
		.sram_pdn_bits = GENMASK(9, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.basic_clk_id = {"cam"},
		.subsys_clk_prefix = "cam",
		.bp_table = {
			BUS_PROT(IFR_TYPE, 0x2d4, 0x2d8, 0, 0x2ec,
				BIT(4) | BIT(5) | BIT(9) | BIT(13),
				BIT(4) | BIT(5) | BIT(9) | BIT(13)),
			BUS_PROT(IFR_TYPE, 0x2a0, 0x2a4, 0, 0x228,
				BIT(28), BIT(28)),
			BUS_PROT(IFR_TYPE, 0x2d4, 0x2d8, 0, 0x2ec,
				BIT(11), 0),
			BUS_PROT(SMI_TYPE, 0x3c4, 0x3c8, 0, 0x3c0,
				BIT(3) | BIT(4), BIT(3) | BIT(4)),
		},
	},
	[MT8183_POWER_DOMAIN_ISP] = {
		.name = "isp",
		.sta_mask = PWR_STATUS_ISP,
		.ctl_offs = 0x0308,
		.sram_pdn_bits = GENMASK(9, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.basic_clk_id = {"isp"},
		.subsys_clk_prefix = "isp",
		.bp_table = {
			BUS_PROT(IFR_TYPE, 0x2d4, 0x2d8, 0, 0x2ec,
				BIT(3) | BIT(8), BIT(3) | BIT(8)),
			BUS_PROT(IFR_TYPE, 0x2d4, 0x2d8, 0, 0x2ec,
				BIT(10), 0),
			BUS_PROT(SMI_TYPE, 0x3c4, 0x3c8, 0, 0x3c0,
				BIT(2), BIT(2)),
		},
	},
	[MT8183_POWER_DOMAIN_VDEC] = {
		.name = "vdec",
		.sta_mask = BIT(31),
		.ctl_offs = 0x0300,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_table = {
			BUS_PROT(SMI_TYPE, 0x3c4, 0x3c8, 0, 0x3c0,
				BIT(7), BIT(7)),
		},
	},
	[MT8183_POWER_DOMAIN_VENC] = {
		.name = "venc",
		.sta_mask = PWR_STATUS_VENC,
		.ctl_offs = 0x0304,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.bp_table = {
			BUS_PROT(SMI_TYPE, 0x3c4, 0x3c8, 0, 0x3c0,
				BIT(1), BIT(1)),
		},
	},
	[MT8183_POWER_DOMAIN_VPU_TOP] = {
		.name = "vpu_top",
		.sta_mask = BIT(26),
		.ctl_offs = 0x0324,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.basic_clk_id = {"vpu", "vpu1"},
		.subsys_clk_prefix = "vpu",
		.bp_table = {
			BUS_PROT(IFR_TYPE, 0x2d4, 0x2d8, 0, 0x2ec,
				GENMASK(9, 6) | BIT(12),
				GENMASK(9, 6) | BIT(12)),
			BUS_PROT(IFR_TYPE, 0x2a0, 0x2a4, 0, 0x228,
				BIT(27), BIT(27)),
			BUS_PROT(IFR_TYPE, 0x2d4, 0x2d8, 0, 0x2ec,
				BIT(10) | BIT(11), BIT(10) | BIT(11)),
			BUS_PROT(SMI_TYPE, 0x3c4, 0x3c8, 0, 0x3c0,
				BIT(5) | BIT(6), BIT(5) | BIT(6)),
		},
	},
	[MT8183_POWER_DOMAIN_VPU_CORE0] = {
		.name = "vpu_core0",
		.sta_mask = BIT(27),
		.ctl_offs = 0x33c,
		.sram_iso_ctrl = true,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.basic_clk_id = {"vpu2"},
		.bp_table = {
			BUS_PROT(IFR_TYPE, 0x2c4, 0x2c8, 0, 0x2e4,
				BIT(6), BIT(6)),
			BUS_PROT(IFR_TYPE, 0x2c4, 0x2c8, 0, 0x2e4,
				BIT(0) | BIT(2) | BIT(4),
				BIT(0) | BIT(2) | BIT(4)),
		},
	},
	[MT8183_POWER_DOMAIN_VPU_CORE1] = {
		.name = "vpu_core1",
		.sta_mask = BIT(28),
		.ctl_offs = 0x0340,
		.sram_iso_ctrl = true,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
		.basic_clk_id = {"vpu3"},
		.bp_table = {
			BUS_PROT(IFR_TYPE, 0x2c4, 0x2c8, 0, 0x2e4,
				BIT(7), BIT(7)),
			BUS_PROT(IFR_TYPE, 0x2c4, 0x2c8, 0, 0x2e4,
				BIT(1) | BIT(3) | BIT(5),
				BIT(1) | BIT(3) | BIT(5)),
		},
	},
};

static const struct scp_subdomain scp_subdomain_mt8183[] = {
	{MT8183_POWER_DOMAIN_MFG_ASYNC, MT8183_POWER_DOMAIN_MFG},
	{MT8183_POWER_DOMAIN_MFG, MT8183_POWER_DOMAIN_MFG_2D},
	{MT8183_POWER_DOMAIN_MFG, MT8183_POWER_DOMAIN_MFG_CORE0},
	{MT8183_POWER_DOMAIN_MFG, MT8183_POWER_DOMAIN_MFG_CORE1},
	{MT8183_POWER_DOMAIN_DISP, MT8183_POWER_DOMAIN_CAM},
	{MT8183_POWER_DOMAIN_DISP, MT8183_POWER_DOMAIN_ISP},
	{MT8183_POWER_DOMAIN_DISP, MT8183_POWER_DOMAIN_VDEC},
	{MT8183_POWER_DOMAIN_DISP, MT8183_POWER_DOMAIN_VENC},
	{MT8183_POWER_DOMAIN_DISP, MT8183_POWER_DOMAIN_VPU_TOP},
	{MT8183_POWER_DOMAIN_VPU_TOP, MT8183_POWER_DOMAIN_VPU_CORE0},
	{MT8183_POWER_DOMAIN_VPU_TOP, MT8183_POWER_DOMAIN_VPU_CORE1},
};

static const struct scp_soc_data mt2701_data = {
	.domains = scp_domain_data_mt2701,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt2701),
	.regs = {
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND
	},
	.bus_prot_reg_update = true,
};

static const struct scp_soc_data mt2712_data = {
	.domains = scp_domain_data_mt2712,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt2712),
	.subdomains = scp_subdomain_mt2712,
	.num_subdomains = ARRAY_SIZE(scp_subdomain_mt2712),
	.regs = {
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND
	},
	.bus_prot_reg_update = false,
};

static const struct scp_soc_data mt6797_data = {
	.domains = scp_domain_data_mt6797,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt6797),
	.subdomains = scp_subdomain_mt6797,
	.num_subdomains = ARRAY_SIZE(scp_subdomain_mt6797),
	.regs = {
		.pwr_sta_offs = SPM_PWR_STATUS_MT6797,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND_MT6797
	},
	.bus_prot_reg_update = true,
};

static const struct scp_soc_data mt7622_data = {
	.domains = scp_domain_data_mt7622,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt7622),
	.regs = {
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND
	},
	.bus_prot_reg_update = true,
};

static const struct scp_soc_data mt7623a_data = {
	.domains = scp_domain_data_mt7623a,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt7623a),
	.regs = {
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND
	},
	.bus_prot_reg_update = true,
};

static const struct scp_soc_data mt8167_data = {
	.domains = scp_domain_data_mt8167,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt8167),
	.subdomains = scp_subdomain_mt8167,
	.num_subdomains = ARRAY_SIZE(scp_subdomain_mt8167),
	.regs = {
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND
	},
	.bus_prot_reg_update = true,
};

static const struct scp_soc_data mt8173_data = {
	.domains = scp_domain_data_mt8173,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt8173),
	.subdomains = scp_subdomain_mt8173,
	.num_subdomains = ARRAY_SIZE(scp_subdomain_mt8173),
	.regs = {
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND
	},
	.bus_prot_reg_update = true,
};

static const struct scp_soc_data mt8183_data = {
	.domains = scp_domain_data_mt8183,
	.num_domains = ARRAY_SIZE(scp_domain_data_mt8183),
	.subdomains = scp_subdomain_mt8183,
	.num_subdomains = ARRAY_SIZE(scp_subdomain_mt8183),
	.regs = {
		.pwr_sta_offs = 0x0180,
		.pwr_sta2nd_offs = 0x0184
	}
};

/*
 * scpsys driver init
 */

static const struct of_device_id of_scpsys_match_tbl[] = {
	{
		.compatible = "mediatek,mt2701-scpsys",
		.data = &mt2701_data,
	}, {
		.compatible = "mediatek,mt2712-scpsys",
		.data = &mt2712_data,
	}, {
		.compatible = "mediatek,mt6797-scpsys",
		.data = &mt6797_data,
	}, {
		.compatible = "mediatek,mt7622-scpsys",
		.data = &mt7622_data,
	}, {
		.compatible = "mediatek,mt7623a-scpsys",
		.data = &mt7623a_data,
	}, {
		.compatible = "mediatek,mt8167-scpsys",
		.data = &mt8167_data,
	}, {
		.compatible = "mediatek,mt8173-scpsys",
		.data = &mt8173_data,
	}, {
		.compatible = "mediatek,mt8183-scpsys",
		.data = &mt8183_data,
	}, {
		/* sentinel */
	}
};

static int scpsys_probe(struct platform_device *pdev)
{
	const struct scp_subdomain *sd;
	const struct scp_soc_data *soc;
	struct scp *scp;
	struct genpd_onecell_data *pd_data;
	int i, ret;

	soc = of_device_get_match_data(&pdev->dev);

	scp = init_scp(pdev, soc->domains, soc->num_domains, &soc->regs,
			soc->bus_prot_reg_update);
	if (IS_ERR(scp))
		return PTR_ERR(scp);

	mtk_register_power_domains(pdev, scp, soc->num_domains);

	pd_data = &scp->pd_data;

	for (i = 0, sd = soc->subdomains; i < soc->num_subdomains; i++, sd++) {
		ret = pm_genpd_add_subdomain(pd_data->domains[sd->origin],
					     pd_data->domains[sd->subdomain]);
		if (ret && IS_ENABLED(CONFIG_PM))
			dev_err(&pdev->dev, "Failed to add subdomain: %d\n",
				ret);
	}

	return 0;
}

static struct platform_driver scpsys_drv = {
	.probe = scpsys_probe,
	.driver = {
		.name = "mtk-scpsys",
		.suppress_bind_attrs = true,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_scpsys_match_tbl),
	},
};
builtin_platform_driver(scpsys_drv);
