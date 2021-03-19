/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 BayLibre, SAS
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#ifndef _MT8516_AFE_COMMON_H_
#define _MT8516_AFE_COMMON_H_

#include "../common/mtk-base-afe.h"

enum {
	MT8516_AFE_BE_ADDA,
};

int mt8516_dai_adda_register(struct mtk_base_afe *afe);

#endif
