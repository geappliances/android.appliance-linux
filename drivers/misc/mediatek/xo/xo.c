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

/*----------------------------------------------------------------------------*/
/*!
* \brief Utility function for reading data from files on NVRAM-FS
*
* \param[in]
*           filename
*           len
*           offset
* \param[out]
*           buf
* \return
*           actual length of data being read
*/
/*----------------------------------------------------------------------------*/
static int nvram_read(char *filename, char *buf, ssize_t len, int offset)
{
#if 1
	struct file *fd;
	int retLen = -1;
	loff_t pos;
	char __user *p;

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	fd = filp_open(filename, O_RDONLY, 0644);

	if (IS_ERR(fd)) {
		pr_err("[MT6620][nvram_read] : failed to open!!\n");
		return -1;
	}

	do {
		if (fd->f_op == NULL) {
			pr_err("[MT6620][nvram_read] : f_op is NULL!!\n");
			break;
		}

		if (fd->f_pos != offset) {
			if (fd->f_op->llseek) {
				if (fd->f_op->llseek(fd, offset, 0) != offset) {
					pr_err("[MT6620][nvram_read] : failed to seek!!\n");
					break;
				}
			} else {
				fd->f_pos = offset;
			}
		}

		p = (__force char __user *)buf;
		pos = (loff_t)offset;
		retLen = __vfs_read(fd, p, len, &pos);
		if (retLen < 0)
			pr_err("[MT6620][nvram_read] : read failed!! Error code: %d\n", retLen);

	} while (0);

	filp_close(fd, NULL);

	set_fs(old_fs);

	return retLen;

#else /* !CFG_SUPPORT_NVRAM */

	return -EIO;

#endif
}

/*----------------------------------------------------------------------------*/
/*!
* \brief Utility function for writing data to files on NVRAM-FS
*
* \param[in]
*           filename
*           buf
*           len
*           offset
* \return
*           actual length of data being written
*/
/*----------------------------------------------------------------------------*/
static int nvram_write(char *filename, char *buf, ssize_t len, int offset)
{
//#if CFG_SUPPORT_NVRAM
#if 1
	struct file *fd;
	int retLen = -1;
	loff_t pos;
	char __user *p;

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	fd = filp_open(filename, O_WRONLY | O_CREAT, 0644);

	if (IS_ERR(fd)) {
		pr_debug("[MT6620][nvram_write] : failed to open!!\n");
		return -1;
	}

	do {
		if (fd->f_op == NULL) {
			pr_debug("[MT6620][nvram_write] : f_op is NULL!!\n");
			break;
		}
		/* End of if */
		if (fd->f_pos != offset) {
			if (fd->f_op->llseek) {
				if (fd->f_op->llseek(fd, offset, 0) != offset) {
					pr_debug("[MT6620][nvram_write] : failed to seek!!\n");
					break;
				}
			} else {
				fd->f_pos = offset;
			}
		}

		p = (__force char __user *)buf;
		pos = (loff_t)offset;

		retLen = vfs_write(fd, p, len, &pos);
		if (retLen < 0)
			pr_debug("[MT6620][nvram_write] : write failed!! Error code: %d\n", retLen);

	} while (0);

	filp_close(fd, NULL);

	set_fs(old_fs);

	return retLen;

#else /* !CFG_SUPPORT_NVRAMS */

	return -EIO;

#endif
}

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

static void enable_xo_low_power_mode(void)
{
	uint32_t value = 0;

	/* RG_DA_EN_XO_BG_MANVALUE = 1 */
	value = BSI_read(0x03) | (1<<12);
	BSI_write(0x03, value);
	/* RG_DA_EN_XO_BG_MAN = 1 */
	value = BSI_read(0x03) | (1<<13);
	BSI_write(0x03, value);

#if defined(KEEP_LDOH)
	/* RG_DA_EN_XO_LDOH_MANVALUE = 1 */
	value = BSI_read(0x03) | (1<<8);
	BSI_write(0x03, value);
	/* RG_DA_EN_XO_LDOH_MAN = 1 */
	value = BSI_read(0x03) | (1<<9);
	BSI_write(0x03, value);
#endif
	/* RG_DA_EN_XO_LDOL_MANVALUE = 1 */
	value = BSI_read(0x03) | 0x1;
	BSI_write(0x03, value);
	/* RG_DA_EN_XO_LDOL_MAN = 1 */
	value = BSI_read(0x03) | (1<<1);
	BSI_write(0x03, value);
	/* RG_DA_EN_XO_PRENMBUF_VALUE = 1 */
	value = BSI_read(0x02) | (1<<6);
	BSI_write(0x02, value);
	/* RG_DA_EN_XO_PRENMBUF_MAN = 1 */
	value = BSI_read(0x02) | (1<<7);
	BSI_write(0x02, value);
	/* RG_DA_EN_XO_PLLGP_BUF_MANVALUE = 1 */
	value = BSI_read(0x34) | 0x1;
	BSI_write(0x34, value);
	/* RG_DA_EN_XO_PLLGP_BUF_MAN = 1 */
	value = BSI_read(0x34) | (1<<1);
	BSI_write(0x34, value);

	/* RG_DA_EN_XO_VGTIELOW_MANVALUE=0 */
	value = BSI_read(0x05) & 0xFEFF;
	BSI_write(0x05, value);

	/* RG_DA_EN_XO_VGTIELOW_MAN=1 */
	value = BSI_read(0x05) | (1<<9);
	BSI_write(0x05, value);

	/* RG_DA_XO_LPM_BIAS1/2/3_MAN=1 */
	value = BSI_read(0x06) | (1<<13);
	BSI_write(0x06, value);
	value = BSI_read(0x06) | (1<<11);
	BSI_write(0x06, value);
#if defined(KEEP_LDOH)
	value = BSI_read(0x06) | (1<<9);
	BSI_write(0x06, value);
#endif
	/* RG_DA_XO_LPM_BIAS1/2/3_MANVALUE=0 */
	value = BSI_read(0x06) & ~BIT(12);
	BSI_write(0x06, value);
	value = BSI_read(0x06) & ~BIT(10);
	BSI_write(0x06, value);
#if defined(KEEP_LDOH)
	value = BSI_read(0x06) & ~BIT(8);
	BSI_write(0x06, value);
#endif
	/* bit 10 set 0 */
	value = BSI_read(0x08) & 0xFBFF;
	BSI_write(0x08, value);

	/* DIG_CR_XO_04_L[9]:RG_XO_INT32K_NOR2LPM_TRIGGER = 1 */
	value = BSI_read(0x08) | (1<<9);
	BSI_write(0x08, value);
	mdelay(5);
	pr_notice("[xo] enable xo low power mode!\n");
}

static void disable_xo_low_power_mode(void)
{
	uint32_t value = 0;

	/* DIG_CR_XO_04_L[9]:RG_XO_INT32K_NOR2LPM_TRIGGER = 0 */
	value = BSI_read(0x08) & ~BIT(9);
	BSI_write(0x08, value);
	mdelay(5);

	/* RG_DA_EN_XO_BG_MAN = 0 */
	value = BSI_read(0x03) & ~BIT(13);
	BSI_write(0x03, value);

#if defined(KEEP_LDOH)
	/* RG_DA_EN_XO_LDOH_MAN = 0 */
	value = BSI_read(0x03) & ~BIT(9);
	BSI_write(0x03, value);
#endif
	/* RG_DA_EN_XO_LDOL_MAN = 0 */
	value = BSI_read(0x03) & ~BIT(1);
	BSI_write(0x03, value);

	/* RG_DA_EN_XO_PRENMBUF_MAN = 0 */
	value = BSI_read(0x02) & ~BIT(7);
	BSI_write(0x02, value);

	/* RG_DA_EN_XO_PLLGP_BUF_MAN = 0 */
	value = BSI_read(0x34) & ~BIT(1);
	BSI_write(0x34, value);

	/* RG_DA_EN_XO_VGTIELOW_MAN= 0 */
	value = BSI_read(0x05) & ~BIT(9);
	BSI_write(0x05, value);

	/* RG_DA_XO_LPM_BIAS1/2_MAN=0 */
	value = BSI_read(0x06) & ~BIT(13);
	BSI_write(0x06, value);
	value = BSI_read(0x06) & ~BIT(11);
	BSI_write(0x06, value);
#if defined(KEEP_LDOH)
	value = BSI_read(0x06) & ~BIT(9);
	BSI_write(0x06, value);
#endif

	pr_notice("[xo] disable xo low power mode!\n");
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

static ssize_t show_xo_nvram_board_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	char capid = 0;

	nvram_read(XOCAP_NVRAM_FILE_NAME, &capid, sizeof(unsigned char), 0);

	return sprintf(buf, "xo nvram capid: 0x%x\n", capid);
}

static ssize_t store_xo_nvram_board_offset(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	uint32_t capid_in = 0;
	char capid;
	int ret;

	if (buf != NULL) {
		ret = kstrtouint(buf, 0, &capid_in);
		if (ret) {
			pr_err("wrong format!\n");
			return 0;
		}
	}

	capid = capid_in & 0x7f;

	pr_notice("store_xo_nvram_board_offset xo set buf is 0x%x!\n", capid);
	nvram_write(XOCAP_NVRAM_FILE_NAME, (char *)(&capid), sizeof(unsigned char), 0);

	return size;
}

static DEVICE_ATTR(xo_nvram_board_offset, 0664, show_xo_nvram_board_offset, store_xo_nvram_board_offset);


static ssize_t show_xo_capid(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t capid;

	bsi_clock_enable(true);
	capid = XO_trim_read();
	bsi_clock_enable(false);
	return sprintf(buf, "xo capid: 0x%x\n", capid);
}

static ssize_t store_xo_capid(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	uint32_t capid;
	int ret;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 0, &capid);
		if (ret) {
			pr_err("wrong format!\n");
			return size;
		}
		if (capid > 0x7f) {
			pr_err("cap code should be 7bit!\n");
			return size;
		}

		bsi_clock_enable(true);

		pr_notice("original cap code: 0x%x\n", XO_trim_read());
		XO_trim_write(capid);
		mdelay(10);
		xo_inst->cur_xo_capid = XO_trim_read();
		pr_notice("write cap code 0x%x done. current cap code:0x%x\n", capid, xo_inst->cur_xo_capid);

		bsi_clock_enable(false);
	}

	return size;
}

static DEVICE_ATTR(xo_capid, 0664, show_xo_capid, store_xo_capid);

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

static uint32_t xo_capid_sub_offset(uint32_t cur_capid, uint32_t ori_capid)
{
	uint32_t cur_capid_sign, cur_capid_value;
	uint32_t ori_capid_sign, ori_capid_value;
	int32_t tmp_value;
	uint32_t final_offset;

	cur_capid_sign = !!(cur_capid & 0x40);
	cur_capid_value = cur_capid & 0x3F;
	ori_capid_sign = !!(ori_capid & 0x40);
	ori_capid_value = ori_capid & 0x3F;

	/* process plus/minus error */
	if (cur_capid_sign ^ ori_capid_sign) {	/* plus */
		tmp_value = (int32_t)cur_capid_value + (int32_t)ori_capid_value;
		if (tmp_value > 0x3F) { /* value overflow */
			final_offset = (cur_capid_sign << 6) | 0x3F;
		} else {
			final_offset = (cur_capid_sign << 6) | (uint32_t)tmp_value;
		}
	} else {	/* minus */
		tmp_value = (int32_t)cur_capid_value - (int32_t)ori_capid_value;
		if (tmp_value < 0) {
			cur_capid_sign = !cur_capid_sign;
			tmp_value = -tmp_value;
		}
		final_offset = (cur_capid_sign << 6) | (uint32_t)tmp_value;
	}
	return final_offset;
}

static ssize_t show_xo_board_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t offset;

	offset = xo_capid_sub_offset(xo_inst->cur_xo_capid, xo_inst->ori_xo_capid);

	return sprintf(buf, "xo capid offset: 0x%x\n", offset);
}

static ssize_t store_xo_board_offset(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	uint32_t offset, capid;
	int ret;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 0, &offset);
		if (ret) {
			pr_err("wrong format!\n");
			return size;
		}
		if (offset > 0x7f) {
			pr_err("offset should be within 7bit!\n");
			return size;
		}

		bsi_clock_enable(true);

		capid = xo_inst->ori_xo_capid;
		pr_notice("original cap code: 0x%x\n", capid);

		capid = xo_capid_add_offset(capid, offset);
		XO_trim_write(capid);
		mdelay(10);
		xo_inst->cur_xo_capid = XO_trim_read();
		pr_notice("write cap code 0x%x done. current cap code:0x%x\n", capid, xo_inst->cur_xo_capid);

		bsi_clock_enable(false);
	}

	return size;
}

static DEVICE_ATTR(xo_board_offset, 0664, show_xo_board_offset, store_xo_board_offset);

static ssize_t show_xo_cmd(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1: status 2/3: in/out LPM 4/5: dis/en 26M 6/7: dis/en 32K 8/9: dis/en rf\n");
}

static ssize_t store_xo_cmd(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	uint32_t cmd;
	int ret;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 0, &cmd);
		if (ret) {
			pr_err("wrong format!\n");
			return size;
		}

		bsi_clock_enable(true);

		switch (cmd) {
		case 1:
			get_xo_status();
			break;
		case 2:
			get_xo_status();
			enable_xo_low_power_mode();
			mdelay(10);
			get_xo_status();
			break;
		case 3:
			get_xo_status();
			disable_xo_low_power_mode();
			mdelay(10);
			get_xo_status();
			break;
		case 4:
			disable_26M_clock_to_pmic();
			break;
		case 5:
			enable_26M_clock_to_pmic();
			break;
		case 6:
			disable_32K_clock_to_pmic();
			break;
		case 7:
			enable_32K_clock_to_pmic();
			break;
		case 8:
			disable_26M_clock_to_conn_rf();
			break;
		case 9:
			enable_26M_clock_to_conn_rf();
			break;
		default:
			pr_notice("cmd not support!\n");
		}

		bsi_clock_enable(false);
	}

	return size;
}

static DEVICE_ATTR(xo_cmd, 0664, show_xo_cmd, store_xo_cmd);

static ssize_t show_bsi_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "not support!\n");
}

static ssize_t store_bsi_read(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	uint32_t addr, value;
	int ret;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 0, &addr);
		if (ret) {
			pr_err("wrong format!\n");
			return size;
		}

		bsi_clock_enable(true);
		value = BSI_read(addr);
		bsi_clock_enable(false);
		pr_notice("bsi read 0x%x: 0x%x\n", addr, value);
	}

	return size;
}

static DEVICE_ATTR(bsi_read, 0664, show_bsi_read, store_bsi_read);

static ssize_t show_bsi_write(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "not support!\n");
}

static ssize_t store_bsi_write(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	char temp_buf[32];
	char *pvalue;
	uint32_t addr, value;
	int ret;

	strncpy(temp_buf, buf, sizeof(temp_buf));
	temp_buf[sizeof(temp_buf) - 1] = 0;
	pvalue = temp_buf;

	if (buf != NULL && size > 5) {

		ret = kstrtouint(strsep(&pvalue, " "), 0, &addr);
		if (ret)
			return ret;
		ret = kstrtouint(pvalue, 0, &value);
		if (ret)
			return ret;

		bsi_clock_enable(true);
		pr_notice("bsi read 0x%x: 0x%x\n", addr, BSI_read(addr));
		BSI_write(addr, value);
		pr_notice("bsi write 0x%x: 0x%x\n", addr, value);
		pr_notice("bsi read 0x%x: 0x%x\n", addr, BSI_read(addr));
		bsi_clock_enable(false);
	}

	return size;
}

static DEVICE_ATTR(bsi_write, 0664, show_bsi_write, store_bsi_write);

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
	char xo_nvram_cap = 0;
	uint32_t capid;
	int ret;

	pr_err("[XO] xocap_work_func\n");
	bsi_clock_enable(true);

	capid = XO_trim_read();

	ret = nvram_read(XOCAP_NVRAM_FILE_NAME, &xo_nvram_cap, sizeof(unsigned char), 0);
	if( ret < 0)
		pr_err(" ret is %d!\n", ret);

	if (xo_nvram_cap > 0x7f)
		pr_err("offset should be within 7bit!\n");
	else {
		capid = xo_capid_add_offset(capid, xo_nvram_cap & 0x7f);
		XO_trim_write(capid);
		mdelay(10);
		xo_inst->cur_xo_capid = XO_trim_read();
		pr_notice("current cap code(after nvram):0x%x\n", xo_inst->cur_xo_capid);
	}

	bsi_clock_enable(false);
}

static void xocap_timer_func(struct timer_list *timer)
{
	pr_err("[XO] xocap_timer_func\n");
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

	retval = device_create_file(&pdev->dev, &dev_attr_bsi_read);
	if (retval != 0)
		dev_dbg(&pdev->dev, "fail to create file: %d\n", retval);

	retval = device_create_file(&pdev->dev, &dev_attr_bsi_write);
	if (retval != 0)
		dev_dbg(&pdev->dev, "fail to create file: %d\n", retval);

	retval = device_create_file(&pdev->dev, &dev_attr_xo_nvram_board_offset);
	if (retval != 0)
		dev_dbg(&pdev->dev, "fail to create file: %d\n", retval);

	retval = device_create_file(&pdev->dev, &dev_attr_xo_capid);
	if (retval != 0)
		dev_dbg(&pdev->dev, "fail to create file: %d\n", retval);

	retval = device_create_file(&pdev->dev, &dev_attr_xo_cmd);
	if (retval != 0)
		dev_dbg(&pdev->dev, "fail to create cmd file: %d\n", retval);

	retval = device_create_file(&pdev->dev, &dev_attr_xo_board_offset);
	if (retval != 0)
		dev_dbg(&pdev->dev, "fail to create offset file: %d\n", retval);

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

