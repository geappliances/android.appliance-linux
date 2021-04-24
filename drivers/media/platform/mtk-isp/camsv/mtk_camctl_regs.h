/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __MTK_CAMCTL_REGS_H__
#define __MTK_CAMCTL_REGS_H__

#define CAMCTL_MODULE_EN			0x0004
#define CAMCTL_DMA_EN				0x000c
#define CAMCTL_FMT_SEL				0x0010
#define CAMCTL_INT_EN				0x0020
#define CAMCTL_INT_STATUS			0x0024
#define CAMCTL_SW_CTL				0x005c
#define CAMCTL_MUX_SEL				0x0074
#define CAMCTL_MUX_SEL2				0x0078
#define CAMCTL_SRAM_MUX_CFG			0x007c
#define CAMCTL_CLK_EN				0x0150

#define CAMTG_SEN_MODE				0x0410
#define CAMTG_VF_CON				0x0414
#define CAMTG_SEN_GRAB_PXL			0x0418
#define CAMTG_SEN_GRAB_LIN			0x041c
#define CAMTG_PATH_CFG				0x0420

#define CAMIMGO_BASE_ADDR			0x0300
#define CAMIMGO_XSIZE				0x0308
#define CAMIMGO_YSIZE				0x030c
#define CAMIMGO_STRIDE				0x0310
#define CAMIMGO_CON				0x0314
#define CAMIMGO_CON2				0x0318

/* CAMCTL_CLK_EN bits */
#define CAMCTL_RAW_DP_CLK_EN			BIT(0)
#define CAMCTL_DIP_DP_CLK_EN			BIT(2)
#define CAMCTL_DMA_DP_CLK_EN			BIT(15)

/* CAMCTL_SW_CTL bits */
#define CAMCTL_IMGO_RST_TRIG			BIT(0)
#define CAMCTL_IMGO_RST_ST			BIT(1)
#define CAMCTL_SW_RST				BIT(2)

/* CAMTG_VF_CON bits */
#define VFDATA_EN				BIT(0)

/* IRQ BITS */
#define CAMCTL_IRQ_TG_ERR			BIT(4)
#define CAMCTL_IRQ_IMGO_ERR			BIT(20)
#define CAMCTL_IRQ_SW_PASS1_DON			BIT(10)

#define INT_ST_MASK_CAMCTL  (CAMCTL_IRQ_SW_PASS1_DON | CAMCTL_IRQ_TG_ERR | \
			     CAMCTL_IRQ_IMGO_ERR)

#define INT_ST_MASK_CAMCTL_ERR                                             \
	(CAMCTL_IRQ_TG_ERR | CAMCTL_IRQ_IMGO_ERR)

#endif /* __MTK_CAMCTL_REGS_H__ */
