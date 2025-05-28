/*
 * Copyright (C) 2018 MediaTek Inc.
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

/**************************************************
 * Shader Present Setting
 **************************************************/
#define T0C0  (BIT(0))
#define T1C0  (BIT(1))
#define T2C0  (BIT(2))
#define T3C0  (BIT(3))
#define T0C1  (BIT(4))
#define T1C1  (BIT(5))
#define T2C1  (BIT(6))
#define T3C1  (BIT(7))
#define T0C2  (BIT(8))
#define T1C2  (BIT(9))
#define T2C2  (BIT(10))
#define T3C2  (BIT(11))
#define T0C3  (BIT(12))
#define T1C3  (BIT(13))
#define T2C3  (BIT(14))
#define T3C3  (BIT(15))
#define T4C0  (BIT(16))
#define T5C0  (BIT(17))
#define T6C0  (BIT(18))
#define T7C0  (BIT(19))
#define T4C1  (BIT(20))
#define T5C1  (BIT(21))
#define T6C1  (BIT(22))
#define T7C1  (BIT(23))


struct mfg_timestamp_register_info {
	unsigned int reg;
	unsigned int mask;
	unsigned int value;
};

struct mtk_platform_context {
	unsigned int shader_present;
	int num_cores;
	void *mfg_base;
};

/**
 * CPU_SPEED_FUNC - A pointer to a function that calculates the CPU clock
 *
 * CPU clock speed of the platform is in MHz - see kbase_cpu_clk_speed_func
 * for the function prototype.
 *
 * Attached value: A kbase_cpu_clk_speed_func.
 * Default Value:  NA
 */
#define CPU_SPEED_FUNC (NULL)

/**
 * GPU_SPEED_FUNC - A pointer to a function that calculates the GPU clock
 *
 * GPU clock speed of the platform in MHz - see kbase_gpu_clk_speed_func
 * for the function prototype.
 *
 * Attached value: A kbase_gpu_clk_speed_func.
 * Default Value:  NA
 */
#define GPU_SPEED_FUNC (NULL)

/**
 * Power management configuration
 *
 * Attached value: pointer to @ref kbase_pm_callback_conf
 * Default value: See @ref kbase_pm_callback_conf
 */
#define POWER_MANAGEMENT_CALLBACKS (&pm_callbacks)

/**
 * Platform specific configuration functions
 *
 * Attached value: pointer to @ref kbase_platform_funcs_conf
 * Default value: See @ref kbase_platform_funcs_conf
 */
#define PLATFORM_FUNCS (&platform_funcs)

extern struct kbase_pm_callback_conf pm_callbacks;
extern struct kbase_platform_funcs_conf platform_funcs;

extern struct kbase_pm_callback_conf mt8183_pm_callbacks;
extern struct kbase_platform_funcs_conf mt8183_platform_funcs;

extern struct kbase_pm_callback_conf mt8195_pm_callbacks;
extern struct kbase_platform_funcs_conf mt8195_platform_funcs;

extern struct kbase_pm_callback_conf mt8188_pm_callbacks;
extern struct kbase_platform_funcs_conf mt8188_platform_funcs;
