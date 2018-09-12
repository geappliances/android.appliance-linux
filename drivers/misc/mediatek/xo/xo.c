/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/mfd/mt6397/rtc_misc.h>
#include <dt-bindings/clock/mt8167-clk.h>

#define BSI_BASE          (xo_inst->base)
#define BSI_CON	          0x0000
#define BSI_WRDAT_CON     0x0004
#define BSI_WRDAT         0x0008
#define BSI_RDCON         0x0c40
#define BSI_RDADDR_CON    0x0c44
#define BSI_RDADDR        0x0c48
#define BSI_RDCS_CON      0x0c4c
#define BSI_RDDAT         0x0c50

#define BSI_WRITE_READY (1 << 31)
#define BSI_READ_READY (1 << 31)
#define BSI_READ_BIT (1 << 8)
#define BITS(m, n) (~(BIT(m)-1) & ((BIT(n) - 1) | BIT(n)))

#define READ_REGISTER_UINT32(reg)          readl((void __iomem *)reg)
#define WRITE_REGISTER_UINT32(reg, val)     writel((val), (void __iomem *)(reg))

#define XOCAP_NVRAM_FILE_NAME   "/data/nvram/APCFG/APRDEB/XOCAP"

#define KEEP_LDOH

struct xo_dev {
	struct device *dev;
	void __iomem *base;
	void __iomem *top_rtc32k;
	struct clk *bsi_clk;
	struct clk *rg_bsi_clk;
	struct clk *bsi_sel_clk;
	struct clk *top_26m_clk;
	uint32_t cur_xo_capid;
	uint32_t ori_xo_capid;
	bool has_ext_crystal;
	bool crystal_check_done;
};

void __iomem *pxo_efuse;
unsigned long xo_data;
struct timer_list xocap_timer;
struct work_struct xocap_work;
static struct xo_dev *xo_inst;
static const struct of_device_id apxo_of_ids[] = {
	{ .compatible = "mediatek,mt8167-xo", },
	{},
};

MODULE_DEVICE_TABLE(of, apxo_of_ids);

static uint32_t BSI_read(uint32_t rdaddr)
{
	uint32_t readaddr = BSI_READ_BIT | rdaddr;
	uint32_t ret;

	WRITE_REGISTER_UINT32(BSI_BASE + BSI_RDCON, 0x9f8b);
	WRITE_REGISTER_UINT32(BSI_BASE + BSI_RDADDR_CON, 0x0902);
	WRITE_REGISTER_UINT32(BSI_BASE + BSI_RDADDR, readaddr);
	WRITE_REGISTER_UINT32(BSI_BASE + BSI_RDCS_CON, 0x0);
	WRITE_REGISTER_UINT32(BSI_BASE + BSI_RDCON, 0x89f8b);

	while (!(READ_REGISTER_UINT32(BSI_BASE + BSI_RDCON) & BSI_READ_READY))
		pr_debug("wait bsi read done!\n");

	ret = READ_REGISTER_UINT32(BSI_BASE + BSI_RDDAT) & 0x0000ffff;
	pr_debug("BSI Read Done: value = 0x%x\n", ret);
	return ret;
}

static void BSI_write(uint32_t wraddr, uint32_t wrdata)
{
	uint32_t wrdat;

	WRITE_REGISTER_UINT32(BSI_BASE + BSI_WRDAT_CON, 0x1d00);
	wrdat = (wraddr << 20) + wrdata;

	pr_debug("BSI_write: wrdat = 0x%x\n", wrdat);
	WRITE_REGISTER_UINT32(BSI_BASE + BSI_WRDAT, wrdat);
	WRITE_REGISTER_UINT32(BSI_BASE + BSI_CON, 0x80401);
	while (!(READ_REGISTER_UINT32(BSI_BASE + BSI_CON) & BSI_WRITE_READY))
		pr_debug("wait bsi write done!\n");

	pr_debug("BSI Write Done\n");
}

static void XO_trim_write(uint32_t cap_code)
{
	uint32_t wrdat = 0;
	/* 0x09 [14:12] = cap_code[6:4] */
	wrdat = BSI_read(0x09) & ~BITS(12, 14);
	wrdat |= (cap_code & BITS(4, 6)) << 8;
	BSI_write(0x09, wrdat);
	/* 0x09 [10:4] = cap_code[6:0] */
	wrdat = BSI_read(0x09) & ~BITS(4, 10);
	wrdat |= (cap_code & BITS(0, 6)) << 4;
	BSI_write(0x09, wrdat);
	/* 0x01 [11:10] = 2'b11 */
	BSI_write(0x01, 0xC00);
	mdelay(10);
	/* 0x01 [11:10] = 2'b01 */
	BSI_write(0x01, 0x400);
	/* 0x1f [5:3] =  cap_code[6:4] */
	wrdat = BSI_read(0x1f) & ~BITS(3, 5);
	wrdat |= (cap_code & BITS(4, 6)) >> 1;
	BSI_write(0x1f, wrdat);
	/* 0x1f [2:0] =  cap_code[6:4] */
	wrdat = BSI_read(0x1f) & ~BITS(0, 2);
	wrdat |= (cap_code & BITS(4, 6)) >> 4;
	BSI_write(0x1f, wrdat);
	/* 0x1e [15:12] =  cap_code[3:0] */
	wrdat = BSI_read(0x1e) & ~BITS(12, 15);
	wrdat |= (cap_code & BITS(0, 3)) << 12;
	BSI_write(0x1e, wrdat);
	/* 0x4b [5:3] =  cap_code[6:4] */
	wrdat = BSI_read(0x4b) & ~BITS(3, 5);
	wrdat |= (cap_code & BITS(4, 6)) >> 1;
	BSI_write(0x4b, wrdat);
	/* 0x4b [2:0] =  cap_code[6:4] */
	wrdat = BSI_read(0x4b) & ~BITS(0, 2);
	wrdat |= (cap_code & BITS(4, 6)) >> 4;
	BSI_write(0x4b, wrdat);
	/* 0x4a [15:12] =  cap_code[3:0] */
	wrdat = BSI_read(0x4a) & ~BITS(12, 15);
	wrdat |= (cap_code & BITS(0, 3)) << 12;
	BSI_write(0x4a, wrdat);
}

static uint32_t XO_trim_read(void)
{
	uint32_t cap_code = 0;
	/* cap_code[4:0] = 0x00 [15:11] */
	cap_code = (BSI_read(0x00) & BITS(11, 15)) >> 11;
	/* cap_code[6:5] = 0x01 [1:0] */
	cap_code |= (BSI_read(0x01) & BITS(0, 1)) << 5;
	return cap_code;
}

static void get_xo_status(void)
{
	uint32_t status = 0;

	status = (BSI_read(0x26) & BITS(4, 9))>>4;
	pr_notice("[xo] status: 0x%x\n", status);
}

void enable_32K_clock_to_pmic(void)
{
	uint32_t value = 0;

	/* Set DIG_CR_XO_24[3:2]=2'b10. */
	value = BSI_read(0x34) | BITS(2, 3);
	BSI_write(0x34, value);
}

void disable_32K_clock_to_pmic(void)
{
	uint32_t value = 0;

	/* Set DIG_CR_XO_24[3:2]=2'b10. */
	value = BSI_read(0x34) & ~BITS(2, 3);
	value = value | (1<<3);
	BSI_write(0x34, value);
}

void enable_26M_clock_to_pmic(void)
{
	uint32_t value = 0;

	/* Set DIG_CR_XO_02[2]=1 */
	value = BSI_read(0x04) | 0x4;
	BSI_write(0x04, value);
	/* Set DIG_CR_XO_02[1]=1 */
	value = BSI_read(0x04) | 0x2;
	BSI_write(0x04, value);
	/* Set DIG_CR_XO_03[29]=1 */
	value = BSI_read(0x7) | (1<<13);
	BSI_write(0x07, value);
	/* Set DIG_CR_XO_03[28]=1 */
	value = BSI_read(0x7) | (1<<12);
	BSI_write(0x07, value);
}

void disable_26M_clock_to_pmic(void)
{
	uint32_t value = 0;

	/* Set DIG_CR_XO_02[2]=1 */
	value = BSI_read(0x04) | 0x4;
	BSI_write(0x04, value);
	/* Set DIG_CR_XO_02[1]=0 */
	value = BSI_read(0x04) & 0xFFFD;
	BSI_write(0x04, value);
	/* Set DIG_CR_XO_03[29]=1 */
	value = BSI_read(0x7) | (1<<13);
	BSI_write(0x07, value);
	/* Set DIG_CR_XO_03[28]=0 */
	value = BSI_read(0x7) & 0xEFFF;
	BSI_write(0x07, value);
}

void disable_26M_clock_to_conn_rf(void)
{
	uint32_t value = 0;

	/* RG_CLKBUF_XO_EN<7:0>=8'h00 */
	value = BSI_read(0x33) & ~BITS(8, 15);
	BSI_write(0x33, value);

	/* Toggle RG_XO_1_2=0'1'0 */
	value = BSI_read(0x29) & 0xFFFE;
	BSI_write(0x29, value);
	value = BSI_read(0x29) | 0x1;
	BSI_write(0x29, value);
	value = BSI_read(0x29) & 0xFFFE;
	BSI_write(0x29, value);
}

void enable_26M_clock_to_conn_rf(void)
{
	uint32_t value = 0;

	/* RG_CLKBUF_XO_EN<7:0>=8'hFF */
	value = BSI_read(0x33) | BITS(8, 15);
	BSI_write(0x33, value);

	/* Toggle RG_XO_1_2=0'1'0 */
	value = BSI_read(0x29) & 0xFFFE;
	BSI_write(0x29, value);
	value = BSI_read(0x29) | 0x1;
	BSI_write(0x29, value);
	value = BSI_read(0x29) & 0xFFFE;
	BSI_write(0x29, value);
}

static void bsi_clock_enable(bool en)
{
	if (en) {
		clk_prepare_enable(xo_inst->bsi_clk);
		clk_prepare_enable(xo_inst->rg_bsi_clk);
	} else {
		clk_disable_unprepare(xo_inst->rg_bsi_clk);
		clk_disable_unprepare(xo_inst->bsi_clk);
	}
}

static uint32_t xo_capid_add_offset(uint32_t capid, uint32_t offset)
{
	uint32_t capid_sign, capid_value;
	uint32_t offset_sign, offset_value;
	int32_t tmp_value;
	uint32_t final_capid;

	capid_sign = !!(capid & 0x40);
	capid_value = capid & 0x3F;
	offset_sign = !!(offset & 0x40);
	offset_value = offset & 0x3F;

	/* process plus/minus overflow */
	if (capid_sign ^ offset_sign) {	/* minus */
		tmp_value = (int32_t)capid_value - (int32_t)offset_value;
		if (tmp_value < 0) {
			capid_sign = !capid_sign;
			tmp_value = -tmp_value;
		}
		final_capid = (capid_sign << 6) | (uint32_t)tmp_value;
	} else {	/* plus */
		tmp_value = (int32_t)capid_value + (int32_t)offset_value;
		if (tmp_value > 0x3F) { /* value overflow */
			final_capid = (capid_sign << 6) | 0x3F;
		} else {
			final_capid = (capid_sign << 6) | (uint32_t)tmp_value;
		}
	}
	return final_capid;
}

/* for SPM driver to get cap code at suspend */
uint32_t mt_xo_get_current_capid(void)
{
	return xo_inst->cur_xo_capid;
}
EXPORT_SYMBOL(mt_xo_get_current_capid);

/* for SPM driver to get crystal status at suspend */
bool mt_xo_has_ext_crystal(void)
{
	return xo_inst->has_ext_crystal;
}
EXPORT_SYMBOL(mt_xo_has_ext_crystal);

static void xocap_work_func(struct work_struct *work)
{
	uint32_t capid;

	pr_debug("[XO] xocap_work_func\n");
	bsi_clock_enable(true);

	capid = XO_trim_read();

	bsi_clock_enable(false);
}

static void xocap_timer_func(struct timer_list *timer)
{
	pr_debug("[XO] xocap_timer_func\n");
	schedule_work(&xocap_work);
}

void mt_xo_init_pre(uint32_t default_capid)
{
	uint32_t xo_efuse;
	uint32_t cap_code;
	int ret;

	bsi_clock_enable(false);
	ret = clk_set_parent(xo_inst->bsi_sel_clk, xo_inst->top_26m_clk);
	if (ret != 0)
		pr_notice("[xo] clk_set_parent fail. ret is %d\n", ret);
	bsi_clock_enable(true);

	pr_notice("[xo] default cap_code: 0x%x\n", XO_trim_read());

	xo_efuse = READ_REGISTER_UINT32(pxo_efuse);

	if ((xo_efuse>>31) & 0x1) {
		pr_notice("[xo] get xo efuse: %x\n", xo_efuse);
		cap_code = (xo_efuse & BITS(24, 30))>>24;

		if ((xo_efuse>>23) & 0x1)
			cap_code = xo_capid_add_offset(cap_code, (xo_efuse & BITS(16, 22))>>16);

		if ((xo_efuse>>15) & 0x1)
			cap_code = xo_capid_add_offset(cap_code, (xo_efuse & BITS(8, 14))>>8);

		cap_code = xo_capid_add_offset(cap_code, default_capid);
	} else {
		pr_notice("[xo] no efuse, apply sw default cap code!\n");
		#ifdef MTK_MT8167_EVB
		cap_code = xo_capid_add_offset(0x22, default_capid);
		#else
		cap_code = xo_capid_add_offset(0x1c, default_capid);
		#endif
	}
	XO_trim_write(cap_code);
	mdelay(10);

	pr_notice("[xo] set cap_code: 0x%x\n", cap_code);
	pr_notice("[xo] current cap_code: 0x%x\n", XO_trim_read());

	/*Audio use XO path, so add the workaround setting for Audio 26M*/

	if((BSI_read(0x25) & 0x1000) != 0){
		BSI_write(0x25, BSI_read(0x25) & ~(1 << 12));
		pr_notice("[Preloader] BSI read: [0x25] = 0x%x\n", BSI_read(0x25));
		BSI_write(0x29, BSI_read(0x29) | (1 << 0));
		pr_notice("[Preloader] BSI read: [0x29] = 0x%x\n", BSI_read(0x29));
		/*delay 100us*/
		udelay(100);
		BSI_write(0x29, BSI_read(0x29) & ~(1 << 0));
		pr_notice("[Preloader] BSI read: [0x29] = 0x%x\n", BSI_read(0x29));
	}

	get_xo_status();
}

static int mt_xo_dts_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct resource *res;
	uint32_t default_capid = 0;

	xo_inst = devm_kzalloc(&pdev->dev, sizeof(*xo_inst), GFP_KERNEL);
	if (!xo_inst)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	xo_inst->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(xo_inst->base))
		return PTR_ERR(xo_inst->base);

	xo_inst->top_rtc32k = devm_ioremap(&pdev->dev, 0x10018000, PAGE_SIZE);
	if (IS_ERR(xo_inst->top_rtc32k))
		return PTR_ERR(xo_inst->top_rtc32k);

	pxo_efuse = devm_ioremap(&pdev->dev, 0x10009264, PAGE_SIZE);
	if (IS_ERR(pxo_efuse))
		return PTR_ERR(pxo_efuse);

	xo_inst->crystal_check_done = false;
	xo_inst->dev = &pdev->dev;
	platform_set_drvdata(pdev, xo_inst);

	xo_inst->bsi_clk = devm_clk_get(&pdev->dev, "bsi");
	if (IS_ERR(xo_inst->bsi_clk)) {
		dev_err(&pdev->dev, "fail to get bsi clock: %ld\n", PTR_ERR(xo_inst->bsi_clk));
		return PTR_ERR(xo_inst->bsi_clk);
	}

	xo_inst->rg_bsi_clk = devm_clk_get(&pdev->dev, "rgbsi");
	if (IS_ERR(xo_inst->rg_bsi_clk)) {
		dev_err(&pdev->dev, "fail to get rgbsi clock: %ld\n", PTR_ERR(xo_inst->rg_bsi_clk));
		return PTR_ERR(xo_inst->rg_bsi_clk);
	}

	xo_inst->bsi_sel_clk= devm_clk_get(&pdev->dev, "bsisel");
	if (IS_ERR(xo_inst->bsi_sel_clk)) {
		dev_err(&pdev->dev, "fail to get bsi_sel clock: %ld\n", PTR_ERR(xo_inst->bsi_sel_clk));
		return PTR_ERR(xo_inst->bsi_sel_clk);
	}

	xo_inst->top_26m_clk= devm_clk_get(&pdev->dev, "clk26m");
	if (IS_ERR(xo_inst->top_26m_clk)) {
		dev_err(&pdev->dev, "fail to get top_26m clock: %ld\n", PTR_ERR(xo_inst->top_26m_clk));
		return PTR_ERR(xo_inst->top_26m_clk);
	}

	bsi_clock_enable(true);

	retval = of_property_read_u32(xo_inst->dev->of_node, "default_capid", &default_capid);
	if (retval != 0) {
		dev_err(&pdev->dev, "fail to get default_capid from dts: %d\n", retval);
		default_capid = 0;
		retval = 0;
	}
	default_capid = default_capid & 0x7f;
	pr_notice("[xo] dts default cap code: 0x%x\n", default_capid);
	mt_xo_init_pre(default_capid);
	xo_inst->cur_xo_capid = XO_trim_read();
	xo_inst->ori_xo_capid = XO_trim_read();
	pr_notice("[xo] current cap code: 0x%x\n", xo_inst->cur_xo_capid);

	bsi_clock_enable(false);

	INIT_WORK(&xocap_work, xocap_work_func);
	timer_setup(&xocap_timer, xocap_timer_func, 0);
	mod_timer(&xocap_timer, jiffies + msecs_to_jiffies(5000));

	return retval;
}

static int mt_xo_dts_remove(struct platform_device *pdev)
{
	bsi_clock_enable(false);
	return 0;
}

static int xo_pm_suspend(struct device *device)
{
	if (!xo_inst->crystal_check_done) {
		xo_inst->has_ext_crystal = !mtk_misc_crystal_exist_status();
		xo_inst->crystal_check_done = true;

		/* let XO use external RTC32K */
		if (xo_inst->has_ext_crystal)
			WRITE_REGISTER_UINT32(xo_inst->top_rtc32k, READ_REGISTER_UINT32(xo_inst->top_rtc32k) | (1<<10));
	}

	return 0;
}

static int xo_pm_resume(struct device *device)
{
	uint32_t value = 0;

	/* re-setting XO audio path for external 32k */
	if (xo_inst->has_ext_crystal) {
		bsi_clock_enable(true);
		/* RG_XO2AUDIO_XO_EN = 0*/
		value = BSI_read(0x25) & ~(1 << 12);
		BSI_write(0x25, value);
		/*XO_EN_MAN = 1*/
		value = BSI_read(0x29) | (1 << 0);
		BSI_write(0x29, value);
		/*delay 100us*/
		udelay(100);
		/*XO_EN_MAN = 0*/
		value = BSI_read(0x29) & ~(1 << 0);
		BSI_write(0x29, value);
		bsi_clock_enable(false);
	}

	return 0;
}

struct dev_pm_ops const xo_pm_ops = {
	.suspend = xo_pm_suspend,
	.resume = xo_pm_resume,
};

static struct platform_driver mt_xo_driver = {
	.remove		= mt_xo_dts_remove,
	.probe		= mt_xo_dts_probe,
	.driver		= {
		.name	= "mt_dts_xo",
		.of_match_table = apxo_of_ids,
		.pm = &xo_pm_ops,
	},
};

static int __init mt_xo_init(void)
{
	int ret;

	ret = platform_driver_register(&mt_xo_driver);

	return ret;
}

module_init(mt_xo_init);

static void __exit mt_xo_exit(void)
{
	platform_driver_unregister(&mt_xo_driver);
}
module_exit(mt_xo_exit);

