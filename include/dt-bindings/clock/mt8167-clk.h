/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: James Liao <jamesjj.liao@mediatek.com>
 */

#ifndef _DT_BINDINGS_CLK_MT8167_H
#define _DT_BINDINGS_CLK_MT8167_H

#include <dt-bindings/clock/mt8516-clk.h>

/* APMIXEDSYS */

#undef CLK_APMIXED_NR_CLK
#define CLK_APMIXED_TVDPLL		6
#define CLK_APMIXED_LVDSPLL		7
#define CLK_APMIXED_HDMI_REF	8
#define CLK_APMIXED_NR_CLK		9

/* TOPCKGEN */

#undef CLK_TOP_NR_CLK
#define CLK_TOP_DSI0_LNTC_DSICK		177
#define CLK_TOP_VPLL_DPIX		178
#define CLK_TOP_LVDSTX_CLKDIG_CTS	179
#define CLK_TOP_HDMTX_CLKDIG_CTS	180
#define CLK_TOP_LVDSPLL			181
#define CLK_TOP_LVDSPLL_D2		182
#define CLK_TOP_LVDSPLL_D4		183
#define CLK_TOP_LVDSPLL_D8		184
#define CLK_TOP_MIPI_26M		185
#define CLK_TOP_TVDPLL			186
#define CLK_TOP_TVDPLL_D2		187
#define CLK_TOP_TVDPLL_D4		188
#define CLK_TOP_TVDPLL_D8		189
#define CLK_TOP_TVDPLL_D16		190
#define CLK_TOP_PWM_MM			191
#define CLK_TOP_CAM_MM			192
#define CLK_TOP_MFG_MM			193
#define CLK_TOP_SPM_52M			194
#define CLK_TOP_MIPI_26M_DBG		195
#define CLK_TOP_SCAM_MM			196
#define CLK_TOP_SMI_MM			197
//#define CLK_TOP_GCE			198
#define CLK_TOP_26M_HDMI_SIFM		199
#define CLK_TOP_26M_CEC			200
#define CLK_TOP_32K_CEC			201
#define CLK_TOP_GCPU_B			202
#define CLK_TOP_RG_VDEC			203
#define CLK_TOP_RG_FDPI0		204
#define CLK_TOP_RG_FDPI1		205
#define CLK_TOP_RG_AXI_MFG		206
#define CLK_TOP_RG_SLOW_MFG		207
#define CLK_TOP_GFMUX_EMI1X_SEL		209
#define CLK_TOP_CSW_MUX_MFG_SEL		212
#define CLK_TOP_CAMTG_MM_SEL		214
#define CLK_TOP_PWM_MM_SEL		215
#define CLK_TOP_SPM_52M_SEL		216
#define CLK_TOP_MFG_MM_SEL		217
#define CLK_TOP_SMI_MM_SEL		218
#define CLK_TOP_SCAM_MM_SEL		219
#define CLK_TOP_VDEC_MM_SEL		220
#define CLK_TOP_DPI0_MM_SEL		221
#define CLK_TOP_DPI1_MM_SEL		222
#define CLK_TOP_AXI_MFG_IN_SEL		223
#define CLK_TOP_SLOW_MFG_SEL		224
#define CLK_TOP_NR_CLK			225

/* MFGCFG */

#define CLK_MFG_BAXI			0
#define CLK_MFG_BMEM			1
#define CLK_MFG_BG3D			2
#define CLK_MFG_B26M			3
#define CLK_MFG_NR_CLK			4

/* MMSYS */

#define CLK_MM_SMI_COMMON		0
#define CLK_MM_SMI_LARB0		1
#define CLK_MM_CAM_MDP			2
#define CLK_MM_MDP_RDMA			3
#define CLK_MM_MDP_RSZ0			4
#define CLK_MM_MDP_RSZ1			5
#define CLK_MM_MDP_TDSHP		6
#define CLK_MM_MDP_WDMA			7
#define CLK_MM_MDP_WROT			8
#define CLK_MM_FAKE_ENG			9
#define CLK_MM_DISP_OVL0		10
#define CLK_MM_DISP_RDMA0		11
#define CLK_MM_DISP_RDMA1		12
#define CLK_MM_DISP_WDMA		13
#define CLK_MM_DISP_COLOR		14
#define CLK_MM_DISP_CCORR		15
#define CLK_MM_DISP_AAL			16
#define CLK_MM_DISP_GAMMA		17
#define CLK_MM_DISP_DITHER		18
#define CLK_MM_DISP_UFOE		19
#define CLK_MM_DISP_PWM_MM		20
#define CLK_MM_DISP_PWM_26M		21
#define CLK_MM_DSI_ENGINE		22
#define CLK_MM_DSI_DIGITAL		23
#define CLK_MM_DPI0_ENGINE		24
#define CLK_MM_DPI0_PXL			25
#define CLK_MM_LVDS_PXL			26
#define CLK_MM_LVDS_CTS			27
#define CLK_MM_DPI1_ENGINE		28
#define CLK_MM_DPI1_PXL			29
#define CLK_MM_HDMI_PXL			30
#define CLK_MM_HDMI_SPDIF		31
#define CLK_MM_HDMI_ADSP_BCK		32
#define CLK_MM_HDMI_PLL			33
#define CLK_MM_NR_CLK			34

/* IMGSYS */

#define CLK_IMG_LARB1_SMI		0
#define CLK_IMG_CAM_SMI			1
#define CLK_IMG_CAM_CAM			2
#define CLK_IMG_SEN_TG			3
#define CLK_IMG_SEN_CAM			4
#define CLK_IMG_VENC			5
#define CLK_IMG_NR_CLK			6

/* VDECSYS */

#define CLK_VDEC_CKEN			0
#define CLK_VDEC_LARB1_CKEN		1
#define CLK_VDEC_NR_CLK			2

#endif /* _DT_BINDINGS_CLK_MT8167_H */
