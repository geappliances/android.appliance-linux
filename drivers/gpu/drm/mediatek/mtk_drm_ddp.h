/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef MTK_DRM_DDP_H
#define MTK_DRM_DDP_H

#include "mtk_drm_ddp_comp.h"

struct regmap;
struct device;
struct mtk_disp_mutex;
struct mtk_mmsys_reg_data;

extern const struct mtk_mmsys_reg_data mt2701_mmsys_reg_data;
extern const struct mtk_mmsys_reg_data mt8167_mmsys_reg_data;
extern const struct mtk_mmsys_reg_data mt8173_mmsys_reg_data;
extern const struct mtk_mmsys_reg_data mt8183_mmsys_reg_data;
extern const struct mtk_mmsys_reg_data mt8365_mmsys_reg_data;
void mtk_ddp_lvds_sys_cfg_lvds(void __iomem *config_regs,
			      const struct mtk_mmsys_reg_data *reg_data);
void mtk_ddp_add_comp_to_path(void __iomem *config_regs,
			      const struct mtk_mmsys_reg_data *reg_data,
			      enum mtk_ddp_comp_id cur,
			      enum mtk_ddp_comp_id next);
void mtk_ddp_remove_comp_from_path(void __iomem *config_regs,
				   const struct mtk_mmsys_reg_data *reg_data,
				   enum mtk_ddp_comp_id cur,
				   enum mtk_ddp_comp_id next);

struct mtk_disp_mutex *mtk_disp_mutex_get(struct device *dev, unsigned int id);
int mtk_disp_mutex_prepare(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_add_comp(struct mtk_disp_mutex *mutex,
			     enum mtk_ddp_comp_id id);
void mtk_disp_mutex_enable(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_disable(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_remove_comp(struct mtk_disp_mutex *mutex,
				enum mtk_ddp_comp_id id);
void mtk_disp_mutex_unprepare(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_put(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_acquire(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_release(struct mtk_disp_mutex *mutex);

#endif /* MTK_DRM_DDP_H */
