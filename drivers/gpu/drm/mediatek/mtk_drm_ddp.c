// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "mtk_drm_ddp.h"
#include "mtk_drm_ddp_comp.h"

#define DISP_REG_CONFIG_DISP_DITHER_MOUT_EN	0x038
#define DISP_REG_CONFIG_DISP_OVL0_MOUT_EN	0x040
#define DISP_REG_CONFIG_DISP_OVL1_MOUT_EN	0x044
#define DISP_REG_CONFIG_DISP_OD_MOUT_EN		0x048
#define DISP_REG_CONFIG_DISP_GAMMA_MOUT_EN	0x04c
#define DISP_REG_CONFIG_DISP_UFOE_MOUT_EN	0x050
#define DISP_REG_CONFIG_DISP_RDMA0_SOUT_SEL_IN	0x06c
#define DISP_REG_CONFIG_DISP_COLOR0_SEL_IN	0x084
#define DISP_REG_CONFIG_DISP_COLOR1_SEL_IN	0x088
#define DISP_REG_CONFIG_DSIE_SEL_IN		0x0a4
#define DISP_REG_CONFIG_DSIO_SEL_IN		0x0a8
#define DISP_REG_CONFIG_DPI_SEL_IN		0x0ac
#define DISP_REG_CONFIG_DISP_RDMA2_SOUT		0x0b8
#define DISP_REG_CONFIG_DISP_RDMA0_SOUT_EN	0x0c4
#define DISP_REG_CONFIG_DISP_RDMA1_SOUT_EN	0x0c8
#define DISP_REG_CONFIG_MMSYS_CG_CON0		0x100
#define LVDS_SYS_CFG_00_LVDS_PXL_CLK		0x30000

#define DISP_REG_CONFIG_DISP_OVL_MOUT_EN	0x030
#define DISP_REG_CONFIG_OUT_SEL			0x04c
#define DISP_REG_CONFIG_DSI_SEL			0x050
#define DISP_REG_CONFIG_DPI_SEL			0x064

#define MT8183_DISP_OVL0_MOUT_EN		0xf00
#define MT8183_DISP_OVL0_2L_MOUT_EN		0xf04
#define MT8183_DISP_OVL1_2L_MOUT_EN		0xf08
#define MT8183_DISP_DITHER0_MOUT_EN		0xf0c
#define MT8183_DISP_PATH0_SEL_IN		0xf24
#define MT8183_DISP_DSI0_SEL_IN			0xf2c
#define MT8183_DISP_DPI0_SEL_IN			0xf30
#define MT8183_DISP_RDMA0_SOUT_SEL_IN		0xf50
#define MT8183_DISP_RDMA1_SOUT_SEL_IN		0xf54

#define OVL0_2L_MOUT_EN_DISP_PATH0			BIT(0)
#define OVL1_2L_MOUT_EN_RDMA1				BIT(4)
#define DITHER0_MOUT_IN_DSI0				BIT(0)
#define DISP_PATH0_SEL_IN_OVL0_2L			0x1
#define DSI0_SEL_IN_RDMA0				0x1
#define MT8183_DSI0_SEL_IN_RDMA1			0x3
#define MT8183_DPI0_SEL_IN_RDMA0			0x1
#define MT8183_DPI0_SEL_IN_RDMA1			0x2
#define MT8183_RDMA0_SOUT_COLOR0			0x1
#define MT8183_RDMA1_SOUT_DSI0				0x1

#define MT2701_DISP_MUTEX0_MOD0			0x2c
#define MT2701_DISP_MUTEX0_SOF0			0x30
#define MT8183_DISP_MUTEX0_MOD0			0x30
#define MT8183_DISP_MUTEX0_SOF0			0x2c

#define DISP_REG_MUTEX_EN(n)			(0x20 + 0x20 * (n))
#define DISP_REG_MUTEX(n)			(0x24 + 0x20 * (n))
#define DISP_REG_MUTEX_RST(n)			(0x28 + 0x20 * (n))
#define DISP_REG_MUTEX_MOD(mutex_mod_reg, n)	(mutex_mod_reg + 0x20 * (n))
#define DISP_REG_MUTEX_SOF(mutex_sof_reg, n)	(mutex_sof_reg + 0x20 * (n))
#define DISP_REG_MUTEX_MOD2(n)			(0x34 + 0x20 * (n))

#define INT_MUTEX				BIT(1)

#define MT8183_MUTEX_MOD_DISP_RDMA0		0
#define MT8183_MUTEX_MOD_DISP_RDMA1		1
#define MT8183_MUTEX_MOD_DISP_OVL0		9
#define MT8183_MUTEX_MOD_DISP_OVL0_2L		10
#define MT8183_MUTEX_MOD_DISP_OVL1_2L		11
#define MT8183_MUTEX_MOD_DISP_WDMA0		12
#define MT8183_MUTEX_MOD_DISP_COLOR0		13
#define MT8183_MUTEX_MOD_DISP_CCORR0		14
#define MT8183_MUTEX_MOD_DISP_AAL0		15
#define MT8183_MUTEX_MOD_DISP_GAMMA0		16
#define MT8183_MUTEX_MOD_DISP_DITHER0		17

#define MT8167_MUTEX_MOD_DISP_OVL0		6
#define MT8167_MUTEX_MOD_DISP_OVL1		7
#define MT8167_MUTEX_MOD_DISP_RDMA0		8
#define MT8167_MUTEX_MOD_DISP_RDMA1		9
#define MT8167_MUTEX_MOD_DISP_WDMA0		10
#define MT8167_MUTEX_MOD_DISP_CCORR		11
#define MT8167_MUTEX_MOD_DISP_COLOR            12
#define MT8167_MUTEX_MOD_DISP_AAL              13
#define MT8167_MUTEX_MOD_DISP_GAMMA            14
#define MT8167_MUTEX_MOD_DISP_DITHER   15
#define MT8167_MUTEX_MOD_DISP_UFOE             16
#define MT8167_MUTEX_MOD_DISP_PWM              1

#define MT8173_MUTEX_MOD_DISP_OVL0		11
#define MT8173_MUTEX_MOD_DISP_OVL1		12
#define MT8173_MUTEX_MOD_DISP_RDMA0		13
#define MT8173_MUTEX_MOD_DISP_RDMA1		14
#define MT8173_MUTEX_MOD_DISP_RDMA2		15
#define MT8173_MUTEX_MOD_DISP_WDMA0		16
#define MT8173_MUTEX_MOD_DISP_WDMA1		17
#define MT8173_MUTEX_MOD_DISP_COLOR0		18
#define MT8173_MUTEX_MOD_DISP_COLOR1		19
#define MT8173_MUTEX_MOD_DISP_AAL		20
#define MT8173_MUTEX_MOD_DISP_GAMMA		21
#define MT8173_MUTEX_MOD_DISP_UFOE		22
#define MT8173_MUTEX_MOD_DISP_PWM0		23
#define MT8173_MUTEX_MOD_DISP_PWM1		24
#define MT8173_MUTEX_MOD_DISP_OD		25

#define MT2712_MUTEX_MOD_DISP_PWM2		10
#define MT2712_MUTEX_MOD_DISP_OVL0		11
#define MT2712_MUTEX_MOD_DISP_OVL1		12
#define MT2712_MUTEX_MOD_DISP_RDMA0		13
#define MT2712_MUTEX_MOD_DISP_RDMA1		14
#define MT2712_MUTEX_MOD_DISP_RDMA2		15
#define MT2712_MUTEX_MOD_DISP_WDMA0		16
#define MT2712_MUTEX_MOD_DISP_WDMA1		17
#define MT2712_MUTEX_MOD_DISP_COLOR0		18
#define MT2712_MUTEX_MOD_DISP_COLOR1		19
#define MT2712_MUTEX_MOD_DISP_AAL0		20
#define MT2712_MUTEX_MOD_DISP_UFOE		22
#define MT2712_MUTEX_MOD_DISP_PWM0		23
#define MT2712_MUTEX_MOD_DISP_PWM1		24
#define MT2712_MUTEX_MOD_DISP_OD0		25
#define MT2712_MUTEX_MOD2_DISP_AAL1		33
#define MT2712_MUTEX_MOD2_DISP_OD1		34

#define MT2701_MUTEX_MOD_DISP_OVL		3
#define MT2701_MUTEX_MOD_DISP_WDMA		6
#define MT2701_MUTEX_MOD_DISP_COLOR		7
#define MT2701_MUTEX_MOD_DISP_BLS		9
#define MT2701_MUTEX_MOD_DISP_RDMA0		10
#define MT2701_MUTEX_MOD_DISP_RDMA1		12

#define MT8168_DISP_MUTEX_CFG			0x8
#define MUTEX_DISABLE_CLK_GATING		0x0
#define MUTEX_ENABLE_CLK_GATING			0x1

#define MUTEX_SOF_SINGLE_MODE		0
#define MUTEX_SOF_DSI0			1
#define MUTEX_SOF_DSI1			2
#define MUTEX_SOF_DPI0			3
#define MUTEX_SOF_DPI1			4
#define MUTEX_SOF_DSI2			5
#define MUTEX_SOF_DSI3			6
#define MT8167_MUTEX_SOF_DPI0		2
#define MT8167_MUTEX_SOF_DPI1		3

#define MT8183_MUTEX_SOF_DPI0			2
#define MT8183_MUTEX_EOF_DSI0			(MUTEX_SOF_DSI0 << 6)
#define MT8183_MUTEX_EOF_DPI0			(MT8183_MUTEX_SOF_DPI0 << 6)

#define OVL0_MOUT_EN_COLOR0		0x1
#define OD_MOUT_EN_RDMA0		0x1
#define OD1_MOUT_EN_RDMA1		BIT(16)
#define UFOE_MOUT_EN_DSI0		0x1
#define COLOR0_SEL_IN_OVL0		0x1
#define OVL1_MOUT_EN_COLOR1		0x1
#define GAMMA_MOUT_EN_RDMA1		0x1
#define RDMA0_SOUT_DPI0			0x2
#define RDMA0_SOUT_DPI1			0x3
#define RDMA0_SOUT_DSI1			0x1
#define RDMA0_SOUT_DSI2			0x4
#define RDMA0_SOUT_DSI3			0x5
#define RDMA1_SOUT_DPI0			0x2
#define RDMA1_SOUT_DPI1			0x3
#define RDMA1_SOUT_DSI1			0x1
#define RDMA1_SOUT_DSI2			0x4
#define RDMA1_SOUT_DSI3			0x5
#define RDMA2_SOUT_DPI0			0x2
#define RDMA2_SOUT_DPI1			0x3
#define RDMA2_SOUT_DSI1			0x1
#define RDMA2_SOUT_DSI2			0x4
#define RDMA2_SOUT_DSI3			0x5
#define DPI0_SEL_IN_RDMA1		0x1
#define DPI0_SEL_IN_RDMA2		0x3
#define DPI1_SEL_IN_RDMA1		(0x1 << 8)
#define DPI1_SEL_IN_RDMA2		(0x3 << 8)
#define DSI0_SEL_IN_RDMA1		0x1
#define DSI0_SEL_IN_RDMA2		0x4
#define DSI1_SEL_IN_RDMA1		0x1
#define DSI1_SEL_IN_RDMA2		0x4
#define DSI2_SEL_IN_RDMA1		(0x1 << 16)
#define DSI2_SEL_IN_RDMA2		(0x4 << 16)
#define DSI3_SEL_IN_RDMA1		(0x1 << 16)
#define DSI3_SEL_IN_RDMA2		(0x4 << 16)
#define COLOR1_SEL_IN_OVL1		0x1

#define OVL_MOUT_EN_RDMA		0x1
#define BLS_TO_DSI_RDMA1_TO_DPI1	0x8
#define BLS_TO_DPI_RDMA1_TO_DSI		0x2
#define DSI_SEL_IN_BLS			0x0
#define DPI_SEL_IN_BLS			0x0
#define DSI_SEL_IN_RDMA			0x1

#define OVL0_MOUT_EN_OVL0_2L		BIT(4)

#define DITHER_MOUT_EN_RDMA            0x1
#define RDMA0_SOUT_SEL_IN_DSI0         0x2

#define MUTEX_EOF_DSI0				BIT(6) /*1<<6*/
#define MUTEX_EOF_DPI0				BIT(7) /*2<<6*/

#define MT8365_DISP_MUTEX0_MOD0			0x30
#define MT8365_DISP_MUTEX0_SOF0			0x2c

#define MT8365_MUTEX_MOD_DISP_OVL0		7
#define MT8365_MUTEX_MOD_DISP_OVL0_2L		8
#define MT8365_MUTEX_MOD_DISP_RDMA0		9
#define MT8365_MUTEX_MOD_DISP_RDMA1		10
#define MT8365_MUTEX_MOD_DISP_WDMA0		11
#define MT8365_MUTEX_MOD_DISP_COLOR0		12
#define MT8365_MUTEX_MOD_DISP_CCORR		13
#define MT8365_MUTEX_MOD_DISP_AAL		14
#define MT8365_MUTEX_MOD_DISP_GAMMA		15
#define MT8365_MUTEX_MOD_DISP_DITHER		16
#define MT8365_MUTEX_MOD_DISP_DSI0		17
#define MT8365_MUTEX_MOD_DISP_RSZ0		19
#define MT8365_MUTEX_MOD_DISP_PWM0		20
#define MT8365_MUTEX_MOD_DISP_DPI0		22

struct mtk_disp_mutex {
	int id;
	bool claimed;
};

enum mtk_ddp_mutex_sof_id {
	DDP_MUTEX_SOF_SINGLE_MODE,
	DDP_MUTEX_SOF_DSI0,
	DDP_MUTEX_SOF_DSI1,
	DDP_MUTEX_SOF_DPI0,
	DDP_MUTEX_SOF_DPI1,
	DDP_MUTEX_SOF_DSI2,
	DDP_MUTEX_SOF_DSI3,
};

struct mtk_ddp_data {
	const unsigned int *mutex_mod;
	const unsigned int *mutex_sof;
	const unsigned int mutex_mod_reg;
	const unsigned int mutex_sof_reg;
	bool clk_gating_config;
};

struct mtk_ddp {
	struct device			*dev;
	struct clk			*clk;
	void __iomem			*regs;
	struct mtk_disp_mutex		mutex[10];
	const struct mtk_ddp_data	*data;
};

struct mtk_mmsys_reg_data {
	u32 ovl0_mout_en;
	u32 rdma0_sout_sel_in;
	u32 rdma0_sout_color0;
	u32 rdma1_sout_sel_in;
	u32 rdma1_sout_dpi0;
	u32 rdma1_sout_dpi1;
	u32 rdma1_sout_dsi0;
	u32 dpi0_sel_in;
	u32 dpi0_sel_in_rdma1;
	u32 dpi1_sel_in;
	u32 dpi1_sel_in_rdma1;
	u32 dsi0_sel_in;
	u32 dsi0_sel_in_rdma1;
	u32 dsi0_sel_in_dither;
	u32 color0_sel_in;
	u32 dither0_mout_en;
	u32 lvds_sys_cfg_00;
	u32 lvds_sys_cfg_00_lvds_pxl_clk;
};

static const unsigned int mt2701_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_BLS] = MT2701_MUTEX_MOD_DISP_BLS,
	[DDP_COMPONENT_COLOR0] = MT2701_MUTEX_MOD_DISP_COLOR,
	[DDP_COMPONENT_OVL0] = MT2701_MUTEX_MOD_DISP_OVL,
	[DDP_COMPONENT_RDMA0] = MT2701_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT2701_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_WDMA0] = MT2701_MUTEX_MOD_DISP_WDMA,
};

static const unsigned int mt2712_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_AAL0] = MT2712_MUTEX_MOD_DISP_AAL0,
	[DDP_COMPONENT_AAL1] = MT2712_MUTEX_MOD2_DISP_AAL1,
	[DDP_COMPONENT_COLOR0] = MT2712_MUTEX_MOD_DISP_COLOR0,
	[DDP_COMPONENT_COLOR1] = MT2712_MUTEX_MOD_DISP_COLOR1,
	[DDP_COMPONENT_OD0] = MT2712_MUTEX_MOD_DISP_OD0,
	[DDP_COMPONENT_OD1] = MT2712_MUTEX_MOD2_DISP_OD1,
	[DDP_COMPONENT_OVL0] = MT2712_MUTEX_MOD_DISP_OVL0,
	[DDP_COMPONENT_OVL1] = MT2712_MUTEX_MOD_DISP_OVL1,
	[DDP_COMPONENT_PWM0] = MT2712_MUTEX_MOD_DISP_PWM0,
	[DDP_COMPONENT_PWM1] = MT2712_MUTEX_MOD_DISP_PWM1,
	[DDP_COMPONENT_PWM2] = MT2712_MUTEX_MOD_DISP_PWM2,
	[DDP_COMPONENT_RDMA0] = MT2712_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT2712_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_RDMA2] = MT2712_MUTEX_MOD_DISP_RDMA2,
	[DDP_COMPONENT_UFOE] = MT2712_MUTEX_MOD_DISP_UFOE,
	[DDP_COMPONENT_WDMA0] = MT2712_MUTEX_MOD_DISP_WDMA0,
	[DDP_COMPONENT_WDMA1] = MT2712_MUTEX_MOD_DISP_WDMA1,
};

static const unsigned int mt8167_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_AAL0] = MT8167_MUTEX_MOD_DISP_AAL,
	[DDP_COMPONENT_CCORR] = MT8167_MUTEX_MOD_DISP_CCORR,
	[DDP_COMPONENT_COLOR0] = MT8167_MUTEX_MOD_DISP_COLOR,
	[DDP_COMPONENT_DITHER] = MT8167_MUTEX_MOD_DISP_DITHER,
	[DDP_COMPONENT_GAMMA] = MT8167_MUTEX_MOD_DISP_GAMMA,
	[DDP_COMPONENT_OVL0] = MT8167_MUTEX_MOD_DISP_OVL0,
	[DDP_COMPONENT_OVL1] = MT8167_MUTEX_MOD_DISP_OVL1,
	[DDP_COMPONENT_PWM0] = MT8167_MUTEX_MOD_DISP_PWM,
	[DDP_COMPONENT_RDMA0] = MT8167_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT8167_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_UFOE] = MT8167_MUTEX_MOD_DISP_UFOE,
	[DDP_COMPONENT_WDMA0] = MT8167_MUTEX_MOD_DISP_WDMA0,
};

static const unsigned int mt8173_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_AAL0] = MT8173_MUTEX_MOD_DISP_AAL,
	[DDP_COMPONENT_COLOR0] = MT8173_MUTEX_MOD_DISP_COLOR0,
	[DDP_COMPONENT_COLOR1] = MT8173_MUTEX_MOD_DISP_COLOR1,
	[DDP_COMPONENT_GAMMA] = MT8173_MUTEX_MOD_DISP_GAMMA,
	[DDP_COMPONENT_OD0] = MT8173_MUTEX_MOD_DISP_OD,
	[DDP_COMPONENT_OVL0] = MT8173_MUTEX_MOD_DISP_OVL0,
	[DDP_COMPONENT_OVL1] = MT8173_MUTEX_MOD_DISP_OVL1,
	[DDP_COMPONENT_PWM0] = MT8173_MUTEX_MOD_DISP_PWM0,
	[DDP_COMPONENT_PWM1] = MT8173_MUTEX_MOD_DISP_PWM1,
	[DDP_COMPONENT_RDMA0] = MT8173_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT8173_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_RDMA2] = MT8173_MUTEX_MOD_DISP_RDMA2,
	[DDP_COMPONENT_UFOE] = MT8173_MUTEX_MOD_DISP_UFOE,
	[DDP_COMPONENT_WDMA0] = MT8173_MUTEX_MOD_DISP_WDMA0,
	[DDP_COMPONENT_WDMA1] = MT8173_MUTEX_MOD_DISP_WDMA1,
};

static const unsigned int mt8183_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_AAL0] = MT8183_MUTEX_MOD_DISP_AAL0,
	[DDP_COMPONENT_CCORR] = MT8183_MUTEX_MOD_DISP_CCORR0,
	[DDP_COMPONENT_COLOR0] = MT8183_MUTEX_MOD_DISP_COLOR0,
	[DDP_COMPONENT_DITHER] = MT8183_MUTEX_MOD_DISP_DITHER0,
	[DDP_COMPONENT_GAMMA] = MT8183_MUTEX_MOD_DISP_GAMMA0,
	[DDP_COMPONENT_OVL0] = MT8183_MUTEX_MOD_DISP_OVL0,
	[DDP_COMPONENT_OVL_2L0] = MT8183_MUTEX_MOD_DISP_OVL0_2L,
	[DDP_COMPONENT_OVL_2L1] = MT8183_MUTEX_MOD_DISP_OVL1_2L,
	[DDP_COMPONENT_RDMA0] = MT8183_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT8183_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_WDMA0] = MT8183_MUTEX_MOD_DISP_WDMA0,
};

static const unsigned int mt8365_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_AAL0] = MT8365_MUTEX_MOD_DISP_AAL,
	[DDP_COMPONENT_COLOR0] = MT8365_MUTEX_MOD_DISP_COLOR0,
	[DDP_COMPONENT_CCORR] = MT8365_MUTEX_MOD_DISP_CCORR,
	[DDP_COMPONENT_GAMMA] = MT8365_MUTEX_MOD_DISP_GAMMA,
	[DDP_COMPONENT_DITHER] = MT8365_MUTEX_MOD_DISP_DITHER,
	[DDP_COMPONENT_OVL0] = MT8365_MUTEX_MOD_DISP_OVL0,
	[DDP_COMPONENT_OVL_2L0] = MT8365_MUTEX_MOD_DISP_OVL0_2L,
	[DDP_COMPONENT_PWM0] = MT8365_MUTEX_MOD_DISP_PWM0,
	[DDP_COMPONENT_RDMA0] = MT8365_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT8365_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_WDMA0] = MT8365_MUTEX_MOD_DISP_WDMA0,
	[DDP_COMPONENT_DPI0] = MT8365_MUTEX_MOD_DISP_DPI0,
};

static const unsigned int mt2712_mutex_sof[DDP_MUTEX_SOF_DSI3 + 1] = {
	[DDP_MUTEX_SOF_SINGLE_MODE] = MUTEX_SOF_SINGLE_MODE,
	[DDP_MUTEX_SOF_DSI0] = MUTEX_SOF_DSI0,
	[DDP_MUTEX_SOF_DSI1] = MUTEX_SOF_DSI1,
	[DDP_MUTEX_SOF_DPI0] = MUTEX_SOF_DPI0,
	[DDP_MUTEX_SOF_DPI1] = MUTEX_SOF_DPI1,
	[DDP_MUTEX_SOF_DSI2] = MUTEX_SOF_DSI2,
	[DDP_MUTEX_SOF_DSI3] = MUTEX_SOF_DSI3,
};

static const unsigned int mt8167_mutex_sof[DDP_MUTEX_SOF_DSI3 + 1] = {
	[DDP_MUTEX_SOF_SINGLE_MODE] = MUTEX_SOF_SINGLE_MODE,
	[DDP_MUTEX_SOF_DSI0] = MUTEX_SOF_DSI0,
	[DDP_MUTEX_SOF_DSI1] = MUTEX_SOF_DSI1,
	[DDP_MUTEX_SOF_DPI0] = MT8167_MUTEX_SOF_DPI0,
	[DDP_MUTEX_SOF_DPI1] = MT8167_MUTEX_SOF_DPI1,
	[DDP_MUTEX_SOF_DSI2] = MUTEX_SOF_DSI2,
	[DDP_MUTEX_SOF_DSI3] = MUTEX_SOF_DSI3,
};

static const unsigned int mt8183_mutex_sof[DDP_MUTEX_SOF_DSI3 + 1] = {
	[DDP_MUTEX_SOF_SINGLE_MODE] = MUTEX_SOF_SINGLE_MODE,
	[DDP_MUTEX_SOF_DSI0] = MUTEX_SOF_DSI0 | MT8183_MUTEX_EOF_DSI0,
	[DDP_MUTEX_SOF_DPI0] = MT8183_MUTEX_SOF_DPI0 | MT8183_MUTEX_EOF_DPI0,
};

static const unsigned int mt8365_mutex_sof[MUTEX_SOF_DSI3 + 1] = {
	[DDP_MUTEX_SOF_SINGLE_MODE] = MUTEX_SOF_SINGLE_MODE,
	[DDP_MUTEX_SOF_DSI0] = MUTEX_SOF_DSI0 | MUTEX_EOF_DSI0,
	[DDP_MUTEX_SOF_DPI0] = MT8183_MUTEX_SOF_DPI0 | MT8183_MUTEX_EOF_DPI0,
};

static const struct mtk_ddp_data mt2701_ddp_driver_data = {
	.mutex_mod = mt2701_mutex_mod,
	.mutex_sof = mt2712_mutex_sof,
	.mutex_mod_reg = MT2701_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT2701_DISP_MUTEX0_SOF0,
};

static const struct mtk_ddp_data mt2712_ddp_driver_data = {
	.mutex_mod = mt2712_mutex_mod,
	.mutex_sof = mt2712_mutex_sof,
	.mutex_mod_reg = MT2701_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT2701_DISP_MUTEX0_SOF0,
};

static const struct mtk_ddp_data mt8167_ddp_driver_data = {
	.mutex_mod = mt8167_mutex_mod,
	.mutex_sof = mt8167_mutex_sof,
	.mutex_mod_reg = MT2701_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT2701_DISP_MUTEX0_SOF0,
};

static const struct mtk_ddp_data mt8173_ddp_driver_data = {
	.mutex_mod = mt8173_mutex_mod,
	.mutex_sof = mt2712_mutex_sof,
	.mutex_mod_reg = MT2701_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT2701_DISP_MUTEX0_SOF0,
};

static const struct mtk_ddp_data mt8183_ddp_driver_data = {
	.mutex_mod = mt8183_mutex_mod,
	.mutex_sof = mt8183_mutex_sof,
	.mutex_mod_reg = MT8183_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT8183_DISP_MUTEX0_SOF0,
};

static const struct mtk_ddp_data mt8365_ddp_driver_data = {
	.mutex_mod = mt8365_mutex_mod,
	.mutex_sof = mt8365_mutex_sof,
	.mutex_mod_reg = MT8365_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT8365_DISP_MUTEX0_SOF0,
	.clk_gating_config = true,
};

const struct mtk_mmsys_reg_data mt2701_mmsys_reg_data = {
	.ovl0_mout_en = DISP_REG_CONFIG_DISP_OVL0_MOUT_EN,
	.dsi0_sel_in = DISP_REG_CONFIG_DSI_SEL,
	.dsi0_sel_in_rdma1 = DSI_SEL_IN_RDMA,
	.rdma1_sout_dpi1 = RDMA1_SOUT_DPI1,
	.dpi1_sel_in = DISP_REG_CONFIG_DPI_SEL_IN,
	.color0_sel_in = DISP_REG_CONFIG_DISP_COLOR0_SEL_IN,
};

const struct mtk_mmsys_reg_data mt8167_mmsys_reg_data = {
	.rdma1_sout_sel_in = 0x70,
	.rdma1_sout_dpi0 = 0x2,
	.rdma1_sout_dpi1 = 0x2,
	.dpi1_sel_in = 0x74,
	.dpi1_sel_in_rdma1 = 0x2,

	.dsi0_sel_in = 0x64,
	.ovl0_mout_en = 0x30,
	.color0_sel_in = 0x58,
};

const struct mtk_mmsys_reg_data mt8173_mmsys_reg_data = {
	.ovl0_mout_en = DISP_REG_CONFIG_DISP_OVL0_MOUT_EN,
	.rdma1_sout_sel_in = DISP_REG_CONFIG_DISP_RDMA1_SOUT_EN,
	.rdma1_sout_dpi0 = RDMA1_SOUT_DPI0,
	.dpi0_sel_in = DISP_REG_CONFIG_DPI_SEL_IN,
	.dpi0_sel_in_rdma1 = DPI0_SEL_IN_RDMA1,
	.dsi0_sel_in = DISP_REG_CONFIG_DSIE_SEL_IN,
	.dsi0_sel_in_rdma1 = DSI0_SEL_IN_RDMA1,
	.rdma1_sout_dpi1 = RDMA1_SOUT_DPI1,
	.dpi1_sel_in = DISP_REG_CONFIG_DPI_SEL_IN,
	.color0_sel_in = DISP_REG_CONFIG_DISP_COLOR0_SEL_IN,
};

const struct mtk_mmsys_reg_data mt8183_mmsys_reg_data = {
	.ovl0_mout_en = MT8183_DISP_OVL0_MOUT_EN,
	.rdma0_sout_sel_in = MT8183_DISP_RDMA0_SOUT_SEL_IN,
	.rdma0_sout_color0 = MT8183_RDMA0_SOUT_COLOR0,
	.rdma1_sout_sel_in = MT8183_DISP_RDMA1_SOUT_SEL_IN,
	.rdma1_sout_dsi0 = MT8183_RDMA1_SOUT_DSI0,
	.dpi0_sel_in = MT8183_DISP_DPI0_SEL_IN,
	.dpi0_sel_in_rdma1 = MT8183_DPI0_SEL_IN_RDMA1,
	.dsi0_sel_in = MT8183_DISP_DSI0_SEL_IN,
	.dsi0_sel_in_rdma1 = MT8183_DSI0_SEL_IN_RDMA1,
	.rdma1_sout_dpi1 = RDMA1_SOUT_DPI1,
	.dpi1_sel_in = DISP_REG_CONFIG_DPI_SEL_IN,
	.color0_sel_in = DISP_REG_CONFIG_DISP_COLOR0_SEL_IN,
	.dither0_mout_en = MT8183_DISP_DITHER0_MOUT_EN,
};

const struct mtk_mmsys_reg_data mt8365_mmsys_reg_data = {
	.ovl0_mout_en = 0xf3c,
	.dither0_mout_en = 0xf50,
	.rdma0_sout_sel_in = 0xf4c,
	.rdma0_sout_color0 = 0x1,
	.dsi0_sel_in = 0xf68,
	.dsi0_sel_in_dither = 0x1,
	.dpi0_sel_in = 0xfd8,
	.dpi0_sel_in_rdma1 = 0x0,
	.rdma1_sout_sel_in = 0xfd0,
	.rdma1_sout_dpi0 = 0x1,
	.lvds_sys_cfg_00 = 0xfdc,
	.lvds_sys_cfg_00_lvds_pxl_clk = 0x1,
};

static unsigned int mtk_ddp_mout_en(const struct mtk_mmsys_reg_data *data,
				    enum mtk_ddp_comp_id cur,
				    enum mtk_ddp_comp_id next,
				    unsigned int *addr)
{
	unsigned int value;

	if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_COLOR0) {
		*addr = data->ovl0_mout_en;
		value = OVL0_MOUT_EN_COLOR0;
	} else if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_RDMA0) {
		*addr = data->ovl0_mout_en;
		value = OVL_MOUT_EN_RDMA;
	} else if (cur == DDP_COMPONENT_DITHER && next == DDP_COMPONENT_RDMA0) {
		*addr = DISP_REG_CONFIG_DISP_DITHER_MOUT_EN;
		value = DITHER_MOUT_EN_RDMA;
	} else if (cur == DDP_COMPONENT_OD0 && next == DDP_COMPONENT_RDMA0) {
		*addr = DISP_REG_CONFIG_DISP_OD_MOUT_EN;
		value = OD_MOUT_EN_RDMA0;
	} else if (cur == DDP_COMPONENT_UFOE && next == DDP_COMPONENT_DSI0) {
		*addr = DISP_REG_CONFIG_DISP_UFOE_MOUT_EN;
		value = UFOE_MOUT_EN_DSI0;
	} else if (cur == DDP_COMPONENT_OVL1 && next == DDP_COMPONENT_COLOR1) {
		*addr = DISP_REG_CONFIG_DISP_OVL1_MOUT_EN;
		value = OVL1_MOUT_EN_COLOR1;
	} else if (cur == DDP_COMPONENT_GAMMA && next == DDP_COMPONENT_RDMA1) {
		*addr = DISP_REG_CONFIG_DISP_GAMMA_MOUT_EN;
		value = GAMMA_MOUT_EN_RDMA1;
	} else if (cur == DDP_COMPONENT_OD1 && next == DDP_COMPONENT_RDMA1) {
		*addr = DISP_REG_CONFIG_DISP_OD_MOUT_EN;
		value = OD1_MOUT_EN_RDMA1;
	} else if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_OVL_2L0) {
		*addr = data->ovl0_mout_en;
		value = OVL0_MOUT_EN_OVL0_2L;
	} else if (cur == DDP_COMPONENT_OVL_2L0 &&
		   next == DDP_COMPONENT_RDMA0) {
		*addr = MT8183_DISP_OVL0_2L_MOUT_EN;
		value = OVL0_2L_MOUT_EN_DISP_PATH0;
	} else if (cur == DDP_COMPONENT_OVL_2L1 &&
		   next == DDP_COMPONENT_RDMA1) {
		*addr = MT8183_DISP_OVL1_2L_MOUT_EN;
		value = OVL1_2L_MOUT_EN_RDMA1;
	} else if (cur == DDP_COMPONENT_DITHER && next == DDP_COMPONENT_DSI0) {
		*addr = data->dither0_mout_en;
		value = DITHER0_MOUT_IN_DSI0;
	} else {
		value = 0;
	}

	return value;
}

static unsigned int mtk_ddp_sel_in(const struct mtk_mmsys_reg_data *data,
				   enum mtk_ddp_comp_id cur,
				   enum mtk_ddp_comp_id next,
				   unsigned int *addr)
{
	unsigned int value;

	if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_COLOR0) {
		*addr = data->color0_sel_in;
		value = COLOR0_SEL_IN_OVL0;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DPI0) {
		*addr = data->dpi0_sel_in;
		value = data->dpi0_sel_in_rdma1;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DPI1) {
		*addr = data->dpi1_sel_in;
		value = data->dpi1_sel_in_rdma1;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DSI0) {
		*addr = data->dsi0_sel_in;
		value = data->dsi0_sel_in_rdma1;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DSI1) {
		*addr = DISP_REG_CONFIG_DSIO_SEL_IN;
		value = DSI1_SEL_IN_RDMA1;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DSI2) {
		*addr = DISP_REG_CONFIG_DSIE_SEL_IN;
		value = DSI2_SEL_IN_RDMA1;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DSI3) {
		*addr = DISP_REG_CONFIG_DSIO_SEL_IN;
		value = DSI3_SEL_IN_RDMA1;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DPI0) {
		*addr = DISP_REG_CONFIG_DPI_SEL_IN;
		value = DPI0_SEL_IN_RDMA2;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DPI1) {
		*addr = DISP_REG_CONFIG_DPI_SEL_IN;
		value = DPI1_SEL_IN_RDMA2;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DSI0) {
		*addr = DISP_REG_CONFIG_DSIE_SEL_IN;
		value = DSI0_SEL_IN_RDMA2;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DSI1) {
		*addr = DISP_REG_CONFIG_DSIO_SEL_IN;
		value = DSI1_SEL_IN_RDMA2;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DSI2) {
		*addr = DISP_REG_CONFIG_DSIE_SEL_IN;
		value = DSI2_SEL_IN_RDMA2;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DSI3) {
		*addr = DISP_REG_CONFIG_DSIE_SEL_IN;
		value = DSI3_SEL_IN_RDMA2;
	} else if (cur == DDP_COMPONENT_OVL1 && next == DDP_COMPONENT_COLOR1) {
		*addr = DISP_REG_CONFIG_DISP_COLOR1_SEL_IN;
		value = COLOR1_SEL_IN_OVL1;
	} else if (cur == DDP_COMPONENT_BLS && next == DDP_COMPONENT_DSI0) {
		*addr = DISP_REG_CONFIG_DSI_SEL;
		value = DSI_SEL_IN_BLS;
	} else if (cur == DDP_COMPONENT_OVL_2L0 &&
		   next == DDP_COMPONENT_RDMA0) {
		*addr = MT8183_DISP_PATH0_SEL_IN;
		value = DISP_PATH0_SEL_IN_OVL0_2L;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DSI0) {
		*addr = data->dsi0_sel_in;
		value = DSI0_SEL_IN_RDMA0;
	} else if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_RDMA0) {
		*addr = 0xf54;
		value = 0;
	} else if (cur == DDP_COMPONENT_COLOR0 && next == DDP_COMPONENT_CCORR) {
		*addr = 0xf64;
		value = 0;
	} else if (cur == DDP_COMPONENT_DITHER && next == DDP_COMPONENT_DSI0) {
		*addr = data->dsi0_sel_in;
		value = data->dsi0_sel_in_dither;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_COLOR0) {
		*addr = 0xf60;
		value = 0;
	} else {
		value = 0;
	}

	return value;
}

static unsigned int mtk_ddp_sout_sel(const struct mtk_mmsys_reg_data *data,
				     enum mtk_ddp_comp_id cur,
				     enum mtk_ddp_comp_id next,
				     unsigned int *addr)
{
	unsigned int value;

	if (cur == DDP_COMPONENT_BLS && next == DDP_COMPONENT_DSI0) {
		*addr = DISP_REG_CONFIG_OUT_SEL;
		value = BLS_TO_DSI_RDMA1_TO_DPI1;
	} else if (cur == DDP_COMPONENT_BLS && next == DDP_COMPONENT_DPI0) {
		*addr = DISP_REG_CONFIG_OUT_SEL;
		value = BLS_TO_DPI_RDMA1_TO_DSI;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DSI0) {
		*addr = DISP_REG_CONFIG_DISP_RDMA0_SOUT_SEL_IN;
		value = RDMA0_SOUT_SEL_IN_DSI0;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DPI0) {
		*addr = DISP_REG_CONFIG_DISP_RDMA0_SOUT_EN;
		value = RDMA0_SOUT_DPI0;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DPI1) {
		*addr = DISP_REG_CONFIG_DISP_RDMA0_SOUT_EN;
		value = RDMA0_SOUT_DPI1;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DSI1) {
		*addr = DISP_REG_CONFIG_DISP_RDMA0_SOUT_EN;
		value = RDMA0_SOUT_DSI1;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DSI2) {
		*addr = DISP_REG_CONFIG_DISP_RDMA0_SOUT_EN;
		value = RDMA0_SOUT_DSI2;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DSI3) {
		*addr = DISP_REG_CONFIG_DISP_RDMA0_SOUT_EN;
		value = RDMA0_SOUT_DSI3;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DSI1) {
		*addr = DISP_REG_CONFIG_DISP_RDMA1_SOUT_EN;
		value = RDMA1_SOUT_DSI1;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DSI2) {
		*addr = DISP_REG_CONFIG_DISP_RDMA1_SOUT_EN;
		value = RDMA1_SOUT_DSI2;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DSI3) {
		*addr = DISP_REG_CONFIG_DISP_RDMA1_SOUT_EN;
		value = RDMA1_SOUT_DSI3;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DPI0) {
		*addr = data->rdma1_sout_sel_in;
		value = data->rdma1_sout_dpi0;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DPI1) {
		*addr = data->rdma1_sout_sel_in;
		value = data->rdma1_sout_dpi1;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DPI0) {
		*addr = DISP_REG_CONFIG_DISP_RDMA2_SOUT;
		value = RDMA2_SOUT_DPI0;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DPI1) {
		*addr = DISP_REG_CONFIG_DISP_RDMA2_SOUT;
		value = RDMA2_SOUT_DPI1;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DSI1) {
		*addr = DISP_REG_CONFIG_DISP_RDMA2_SOUT;
		value = RDMA2_SOUT_DSI1;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DSI2) {
		*addr = DISP_REG_CONFIG_DISP_RDMA2_SOUT;
		value = RDMA2_SOUT_DSI2;
	} else if (cur == DDP_COMPONENT_RDMA2 && next == DDP_COMPONENT_DSI3) {
		*addr = DISP_REG_CONFIG_DISP_RDMA2_SOUT;
		value = RDMA2_SOUT_DSI3;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_COLOR0) {
		*addr = data->rdma0_sout_sel_in;
		value = data->rdma0_sout_color0;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DSI0) {
		*addr = data->rdma1_sout_sel_in;
		value = data->rdma1_sout_dsi0;
	} else {
		value = 0;
	}

	return value;
}

void mtk_ddp_lvds_sys_cfg_lvds(void __iomem *config_regs,
			       const struct mtk_mmsys_reg_data *reg_data)
{
	if (reg_data->lvds_sys_cfg_00)
		writel_relaxed(LVDS_SYS_CFG_00_LVDS_PXL_CLK,
			       config_regs + reg_data->lvds_sys_cfg_00);
}

void mtk_ddp_add_comp_to_path(void __iomem *config_regs,
			      const struct mtk_mmsys_reg_data *reg_data,
			      enum mtk_ddp_comp_id cur,
			      enum mtk_ddp_comp_id next)
{
	unsigned int addr, value, reg;

	if (reg_data->lvds_sys_cfg_00)
		writel_relaxed(reg_data->lvds_sys_cfg_00_lvds_pxl_clk,
			       config_regs + reg_data->lvds_sys_cfg_00);

	value = mtk_ddp_mout_en(reg_data, cur, next, &addr);
	if (value) {
		reg = readl_relaxed(config_regs + addr) | value;
		writel_relaxed(reg, config_regs + addr);
	}

	value = mtk_ddp_sout_sel(reg_data, cur, next, &addr);
	if (value)
		writel_relaxed(value, config_regs + addr);

	value = mtk_ddp_sel_in(reg_data, cur, next, &addr);
	if (value) {
		reg = readl_relaxed(config_regs + addr) | value;
		writel_relaxed(reg, config_regs + addr);
	}
}

void mtk_ddp_remove_comp_from_path(void __iomem *config_regs,
				   const struct mtk_mmsys_reg_data *reg_data,
				   enum mtk_ddp_comp_id cur,
				   enum mtk_ddp_comp_id next)
{
	unsigned int addr, value, reg;

	value = mtk_ddp_mout_en(reg_data, cur, next, &addr);
	if (value) {
		reg = readl_relaxed(config_regs + addr) & ~value;
		writel_relaxed(reg, config_regs + addr);
	}

	value = mtk_ddp_sel_in(reg_data, cur, next, &addr);
	if (value) {
		reg = readl_relaxed(config_regs + addr) & ~value;
		writel_relaxed(reg, config_regs + addr);
	}
}

struct mtk_disp_mutex *mtk_disp_mutex_get(struct device *dev, unsigned int id)
{
	struct mtk_ddp *ddp = dev_get_drvdata(dev);

	if (id >= 10)
		return ERR_PTR(-EINVAL);
	if (ddp->mutex[id].claimed)
		return ERR_PTR(-EBUSY);

	ddp->mutex[id].claimed = true;

	return &ddp->mutex[id];
}

void mtk_disp_mutex_put(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	mutex->claimed = false;
}

int mtk_disp_mutex_prepare(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	if (ddp->data->clk_gating_config)
		writel_relaxed(MUTEX_DISABLE_CLK_GATING,
			       ddp->regs + MT8168_DISP_MUTEX_CFG);

	return clk_prepare_enable(ddp->clk);
}

void mtk_disp_mutex_unprepare(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	if (ddp->data->clk_gating_config)
		writel_relaxed(MUTEX_ENABLE_CLK_GATING,
			       ddp->regs + MT8168_DISP_MUTEX_CFG);
	clk_disable_unprepare(ddp->clk);
}

void mtk_disp_mutex_add_comp(struct mtk_disp_mutex *mutex,
			     enum mtk_ddp_comp_id id)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);
	unsigned int reg;
	unsigned int sof_id;
	unsigned int offset;

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	switch (id) {
	case DDP_COMPONENT_DSI0:
		sof_id = DDP_MUTEX_SOF_DSI0;
		break;
	case DDP_COMPONENT_DSI1:
		sof_id = DDP_MUTEX_SOF_DSI0;
		break;
	case DDP_COMPONENT_DSI2:
		sof_id = DDP_MUTEX_SOF_DSI2;
		break;
	case DDP_COMPONENT_DSI3:
		sof_id = DDP_MUTEX_SOF_DSI3;
		break;
	case DDP_COMPONENT_DPI0:
		sof_id = DDP_MUTEX_SOF_DPI0;
		break;
	case DDP_COMPONENT_DPI1:
		sof_id = DDP_MUTEX_SOF_DPI1;
		break;
	case DDP_COMPONENT_LVDS:
		return;
	default:
		if (ddp->data->mutex_mod[id] < 32) {
			offset = DISP_REG_MUTEX_MOD(ddp->data->mutex_mod_reg,
						    mutex->id);
			reg = readl_relaxed(ddp->regs + offset);
			reg |= 1 << ddp->data->mutex_mod[id];
			writel_relaxed(reg, ddp->regs + offset);
		} else {
			offset = DISP_REG_MUTEX_MOD2(mutex->id);
			reg = readl_relaxed(ddp->regs + offset);
			reg |= 1 << (ddp->data->mutex_mod[id] - 32);
			writel_relaxed(reg, ddp->regs + offset);
		}
		return;
	}

	writel_relaxed(ddp->data->mutex_sof[sof_id],
		       ddp->regs +
		       DISP_REG_MUTEX_SOF(ddp->data->mutex_sof_reg, mutex->id));
}

void mtk_disp_mutex_remove_comp(struct mtk_disp_mutex *mutex,
				enum mtk_ddp_comp_id id)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);
	unsigned int reg;
	unsigned int offset;

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	switch (id) {
	case DDP_COMPONENT_DSI0:
	case DDP_COMPONENT_DSI1:
	case DDP_COMPONENT_DSI2:
	case DDP_COMPONENT_DSI3:
	case DDP_COMPONENT_DPI0:
	case DDP_COMPONENT_DPI1:
		writel_relaxed(MUTEX_SOF_SINGLE_MODE,
			       ddp->regs +
			       DISP_REG_MUTEX_SOF(ddp->data->mutex_sof_reg,
						  mutex->id));
		break;
	default:
		if (ddp->data->mutex_mod[id] < 32) {
			offset = DISP_REG_MUTEX_MOD(ddp->data->mutex_mod_reg,
						    mutex->id);
			reg = readl_relaxed(ddp->regs + offset);
			reg &= ~(1 << ddp->data->mutex_mod[id]);
			writel_relaxed(reg, ddp->regs + offset);
		} else {
			offset = DISP_REG_MUTEX_MOD2(mutex->id);
			reg = readl_relaxed(ddp->regs + offset);
			reg &= ~(1 << (ddp->data->mutex_mod[id] - 32));
			writel_relaxed(reg, ddp->regs + offset);
		}
		break;
	}
}

void mtk_disp_mutex_enable(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	writel(1, ddp->regs + DISP_REG_MUTEX_EN(mutex->id));
}

void mtk_disp_mutex_disable(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	writel(0, ddp->regs + DISP_REG_MUTEX_EN(mutex->id));
}

void mtk_disp_mutex_acquire(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);
	u32 tmp;

	writel(1, ddp->regs + DISP_REG_MUTEX_EN(mutex->id));
	writel(1, ddp->regs + DISP_REG_MUTEX(mutex->id));
	if (readl_poll_timeout_atomic(ddp->regs + DISP_REG_MUTEX(mutex->id),
				      tmp, tmp & INT_MUTEX, 1, 10000))
		pr_err("could not acquire mutex %d\n", mutex->id);
}

void mtk_disp_mutex_release(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	writel(0, ddp->regs + DISP_REG_MUTEX(mutex->id));
}

static int mtk_ddp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_ddp *ddp;
	struct resource *regs;
	int i;

	ddp = devm_kzalloc(dev, sizeof(*ddp), GFP_KERNEL);
	if (!ddp)
		return -ENOMEM;

	for (i = 0; i < 10; i++)
		ddp->mutex[i].id = i;

	if (of_find_property(dev->of_node, "clocks", &i)) {
		ddp->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(ddp->clk)) {
			dev_err(dev, "Failed to get clock\n");
			return PTR_ERR(ddp->clk);
		}
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ddp->regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ddp->regs)) {
		dev_err(dev, "Failed to map mutex registers\n");
		return PTR_ERR(ddp->regs);
	}

	ddp->data = of_device_get_match_data(dev);

	platform_set_drvdata(pdev, ddp);

	return 0;
}

static int mtk_ddp_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ddp_driver_dt_match[] = {
	{ .compatible = "mediatek,mt2701-disp-mutex",
	  .data = &mt2701_ddp_driver_data},
	{ .compatible = "mediatek,mt2712-disp-mutex",
	  .data = &mt2712_ddp_driver_data},
	{ .compatible = "mediatek,mt8167-disp-mutex",
	  .data = &mt8167_ddp_driver_data},
	{ .compatible = "mediatek,mt8173-disp-mutex",
	  .data = &mt8173_ddp_driver_data},
	{ .compatible = "mediatek,mt8183-disp-mutex",
	  .data = &mt8183_ddp_driver_data},
	{ .compatible = "mediatek,mt8365-disp-mutex",
	  .data = &mt8365_ddp_driver_data},
	{},
};
MODULE_DEVICE_TABLE(of, ddp_driver_dt_match);

struct platform_driver mtk_ddp_driver = {
	.probe		= mtk_ddp_probe,
	.remove		= mtk_ddp_remove,
	.driver		= {
		.name	= "mediatek-ddp",
		.owner	= THIS_MODULE,
		.of_match_table = ddp_driver_dt_match,
	},
};
