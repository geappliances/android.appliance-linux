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

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <mali_kbase.h>
#include "mali_kbase_config_platform.h"
#include <mali_kbase_defs.h>

static struct platform_device *probe_gpu_core1_dev;
static struct platform_device *probe_gpu_core2_dev;

static const struct of_device_id mtk_gpu_corex_of_ids[] = {
	{ .compatible = "mediatek,gpu_core1", .data = "1" },
	{ .compatible = "mediatek,gpu_core2", .data = "2" },
	{}
};

static const struct  mfg_timestamp_register_info mt8183_mfg_timestamp_register_info = {
	.reg = 0x130, .mask = 0b00000011, .value = 0xFF };
static const struct  mfg_timestamp_register_info mt8365_mfg_timestamp_register_info = {
	.reg = 0x130, .mask = 0b00000011, .value = 0xFF };

static const struct of_device_id mtk_mfgcfg_comp_dt_ids[] = {
	{ .compatible = "mediatek,mt8183-mfgcfg",
	  .data = &mt8183_mfg_timestamp_register_info },
	{ .compatible = "mediatek,mt8365-mfgcfg",
	  .data = &mt8365_mfg_timestamp_register_info }
};

static int mtk_gpu_corex_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	const char *tmp;

	match = of_match_device(mtk_gpu_corex_of_ids, dev);
	if (!match)
		return -ENODEV;
	tmp = match->data;
	if (*tmp == '1')
		probe_gpu_core1_dev = pdev;
	else
		probe_gpu_core2_dev = pdev;

	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int mtk_gpu_corex_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static struct platform_driver mtk_gpu_corex_driver = {
	.probe  = mtk_gpu_corex_probe,
	.remove = mtk_gpu_corex_remove,
	.driver = {
		.name = "gpu_corex",
		.of_match_table = mtk_gpu_corex_of_ids,
	}
};

static int pm_callback_power_on(struct kbase_device *kbdev)
{
	int error;
	struct mfg_base *mfg = kbdev->platform_context;

	if (mfg->is_powered) {
		dev_dbg(kbdev->dev, "mali_device is already powered\n");
		return 0;
	}

	error = pm_runtime_get_sync(kbdev->dev);
	if (error < 0) {
		dev_err(kbdev->dev,
			"Power on core 0 failed (err: %d)\n", error);
		return error;
	}

	error = pm_runtime_get_sync(&mfg->gpu_core1_dev->dev);
	if (error < 0) {
		dev_err(kbdev->dev,
			"Power on core 1 failed (err: %d)\n", error);
		return error;
	}

	error = pm_runtime_get_sync(&mfg->gpu_core2_dev->dev);
	if (error < 0) {
		dev_err(kbdev->dev,
			"Power on core 2 failed (err: %d)\n", error);
		return error;
	}

	error = clk_enable(mfg->clk_main_parent);
	if (error < 0) {
		dev_err(kbdev->dev,
			"clk_main_parent clock enable failed (err: %d)\n",
			error);
		return error;
	}

	error = clk_enable(mfg->clk_mux);
	if (error < 0) {
		dev_err(kbdev->dev,
			"clk_mux clock enable failed (err: %d)\n", error);
		return error;
	}

	error = clk_enable(mfg->subsys_mfg_cg);
	if (error < 0) {
		dev_err(kbdev->dev,
			"subsys_mfg_cg clock enable failed (err: %d)\n", error);
		return error;
	}

	/* Write 1 into 0x13000130 bit 0 & bit 1 to enable timestamp register.*/
	regmap_update_bits(mfg->mfgcfg, mfg->timestamp_register_info->reg,
			   mfg->timestamp_register_info->mask,
			   mfg->timestamp_register_info->value);

	mfg->is_powered = true;

	return 1;
}

static void pm_callback_power_off(struct kbase_device *kbdev)
{
	struct mfg_base *mfg = kbdev->platform_context;
	int error;

	if (!mfg->is_powered) {
		dev_dbg(kbdev->dev, "mali_device is already powered off\n");
		return;
	}

	mfg->is_powered = false;

	clk_disable(mfg->subsys_mfg_cg);

	clk_disable(mfg->clk_mux);

	clk_disable(mfg->clk_main_parent);

	pm_runtime_mark_last_busy(&mfg->gpu_core2_dev->dev);
	error = pm_runtime_put_autosuspend(&mfg->gpu_core2_dev->dev);
	if (error < 0)
		dev_err(kbdev->dev,
		"Power off core 2 failed (err: %d)\n", error);

	pm_runtime_mark_last_busy(&mfg->gpu_core1_dev->dev);
	error = pm_runtime_put_autosuspend(&mfg->gpu_core1_dev->dev);
	if (error < 0)
		dev_err(kbdev->dev,
		"Power off core 1 failed (err: %d)\n", error);

	pm_runtime_mark_last_busy(kbdev->dev);
	error = pm_runtime_put_autosuspend(kbdev->dev);
	if (error < 0)
		dev_err(kbdev->dev,
		"Power off core 0 failed (err: %d)\n", error);
}

static int kbase_device_runtime_init(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "%s\n", __func__);

	return 0;
}

static void kbase_device_runtime_disable(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "%s\n", __func__);
}

static int pm_callback_runtime_on(struct kbase_device *kbdev)
{
	struct mfg_base *mfg = kbdev->platform_context;
	int error, i;

	if (mfg->reg_is_powered) {
		dev_dbg(kbdev->dev, "GPU regulators are already power on\n");
		return 0;
	}

	for (i = 0; i < kbdev->nr_regulators; i++) {
		error = regulator_enable(kbdev->regulators[i]);
		if (error < 0) {
			dev_err(kbdev->dev,
				"Power on reg %d failed error = %d\n",
				i, error);
			return error;
		}
	}

	error = clk_prepare(mfg->clk_main_parent);
	if (error < 0) {
		dev_err(kbdev->dev,
			"clk_main_parent clock prepare failed (err: %d)\n",
			error);
		return error;
	}

	error = clk_prepare(mfg->clk_mux);
	if (error < 0) {
		dev_err(kbdev->dev,
			"clk_mux clock prepare failed (err: %d)\n", error);
		return error;
	}

	error = clk_prepare(mfg->subsys_mfg_cg);
	if (error < 0) {
		dev_err(kbdev->dev,
			"subsys_mfg_cg clock prepare failed (err: %d)\n",
			error);
		return error;
	}

	mfg->reg_is_powered = true;

	return 0;
}

static void pm_callback_runtime_off(struct kbase_device *kbdev)
{
	struct mfg_base *mfg = kbdev->platform_context;
	int error, i;

	if (!mfg->reg_is_powered) {
		dev_dbg(kbdev->dev, "GPU regulators are already power off\n");
		return;
	}

	clk_unprepare(mfg->subsys_mfg_cg);

	clk_unprepare(mfg->clk_mux);

	clk_unprepare(mfg->clk_main_parent);

	for (i = 0; i < kbdev->nr_regulators; i++) {
		error = regulator_disable(kbdev->regulators[i]);
		if (error < 0) {
			dev_err(kbdev->dev,
				"Power off reg %d failed error = %d\n",
				i, error);
		}
	}

	mfg->reg_is_powered = false;
}

static void pm_callback_resume(struct kbase_device *kbdev)
{
	pm_callback_runtime_on(kbdev);
	pm_callback_power_on(kbdev);
}

static void pm_callback_suspend(struct kbase_device *kbdev)
{
	pm_callback_power_off(kbdev);
	pm_callback_runtime_off(kbdev);
}

struct kbase_pm_callback_conf pm_callbacks = {
	.power_on_callback = pm_callback_power_on,
	.power_off_callback = pm_callback_power_off,
	.power_suspend_callback = pm_callback_suspend,
	.power_resume_callback = pm_callback_resume,
#ifdef KBASE_PM_RUNTIME
	.power_runtime_init_callback = kbase_device_runtime_init,
	.power_runtime_term_callback = kbase_device_runtime_disable,
	.power_runtime_on_callback = pm_callback_runtime_on,
	.power_runtime_off_callback = pm_callback_runtime_off,
#else				/* KBASE_PM_RUNTIME */
	.power_runtime_init_callback = NULL,
	.power_runtime_term_callback = NULL,
	.power_runtime_on_callback = NULL,
	.power_runtime_off_callback = NULL,
#endif				/* KBASE_PM_RUNTIME */
};

int find_match_mfgcfg(struct mfg_base *mfg)
{
	int i;
	struct device_node *node;

	for (i = 0 ; i < ARRAY_SIZE(mtk_mfgcfg_comp_dt_ids); i++) {
		node = of_find_compatible_node(NULL, NULL, mtk_mfgcfg_comp_dt_ids[i].compatible);
		if (node) {
			mfg->timestamp_register_info =
				(struct mfg_timestamp_register_info*)mtk_mfgcfg_comp_dt_ids[i].data;
			break;
		}
	}

	if (IS_ERR(node))
		return PTR_ERR(node);

	mfg->mfgcfg = syscon_node_to_regmap(node);
	if (IS_ERR(mfg->mfgcfg))
		return PTR_ERR(mfg->mfgcfg);

	return 0;
}

int mali_mfgsys_init(struct kbase_device *kbdev, struct mfg_base *mfg)
{
	int err = 0, i;
	unsigned long volt;

	if (!probe_gpu_core1_dev || !probe_gpu_core2_dev)
		return -EPROBE_DEFER;

	for (i = 0; i < kbdev->nr_regulators; i++)
		if (kbdev->regulators[i] == NULL)
			return -EINVAL;

	mfg->gpu_core1_dev = probe_gpu_core1_dev;
	mfg->gpu_core2_dev = probe_gpu_core2_dev;

	mfg->clk_main_parent = devm_clk_get(kbdev->dev, "clk_main_parent");
	if (IS_ERR(mfg->clk_main_parent)) {
		err = PTR_ERR(mfg->clk_main_parent);
		dev_err(kbdev->dev, "devm_clk_get clk_main_parent failed\n");
		return err;
	}

	mfg->clk_sub_parent = devm_clk_get(kbdev->dev, "clk_sub_parent");
	if (IS_ERR(mfg->clk_sub_parent)) {
		err = PTR_ERR(mfg->clk_sub_parent);
		dev_err(kbdev->dev, "devm_clk_get clk_sub_parent failed\n");
		return err;
	}

	mfg->clk_mux = devm_clk_get(kbdev->dev, "clk_mux");
	if (IS_ERR(mfg->clk_mux)) {
		err = PTR_ERR(mfg->clk_mux);
		dev_err(kbdev->dev, "devm_clk_get clk_mux failed\n");
		return err;
	}

	mfg->subsys_mfg_cg = devm_clk_get(kbdev->dev, "subsys_mfg_cg");
	if (IS_ERR(mfg->subsys_mfg_cg)) {
		err = PTR_ERR(mfg->subsys_mfg_cg);
		dev_err(kbdev->dev, "devm_clk_get subsys_mfg_cg failed\n");
		return err;
	}

	for (i = 0; i < kbdev->nr_regulators; i++) {
		volt = (i == 0) ? VGPU_MAX_VOLT : VSRAM_GPU_MAX_VOLT;
		err = regulator_set_voltage(kbdev->regulators[i],
			volt, volt + VOLT_TOL);
		if (err < 0) {
			dev_err(kbdev->dev,
				"Regulator %d set voltage failed: %d\n",
				i, err);
			return err;
		}
		kbdev->current_voltages[i] = volt;
	}

	err = find_match_mfgcfg(mfg);
	if (err) {
		dev_err(kbdev->dev, "Failed to lookup matched mfgcfg\n");
		return err;
	}

	mfg->is_powered = false;
	mfg->reg_is_powered = false;

	return 0;
}

static int platform_init(struct kbase_device *kbdev)
{
	int err;
	struct mfg_base *mfg;

	mfg = kzalloc(sizeof(*mfg), GFP_KERNEL);
	if (!mfg)
		return -ENOMEM;

	err = mali_mfgsys_init(kbdev, mfg);
	if (err)
		goto platform_init_err;

	kbdev->platform_context = mfg;
	pm_runtime_set_autosuspend_delay(kbdev->dev, 50);
	pm_runtime_use_autosuspend(kbdev->dev);
	pm_runtime_enable(kbdev->dev);

	err = clk_set_parent(mfg->clk_mux, mfg->clk_sub_parent);
	if (err) {
		dev_err(kbdev->dev, "Failed to select sub clock src\n");
		goto platform_init_err;
	}

	err = clk_set_rate(mfg->clk_main_parent, GPU_FREQ_KHZ_MAX * 1000);
	if (err) {
		dev_err(kbdev->dev, "Failed to set clock %d kHz\n",
				GPU_FREQ_KHZ_MAX);
		goto platform_init_err;
	}

	err = clk_set_parent(mfg->clk_mux, mfg->clk_main_parent);
	if (err) {
		dev_err(kbdev->dev, "Failed to select main clock src\n");
		goto platform_init_err;
	}

	return 0;

platform_init_err:
	kfree(mfg);
	return err;
}

static void platform_term(struct kbase_device *kbdev)
{
	struct mfg_base *mfg = kbdev->platform_context;

	kfree(mfg);
	kbdev->platform_context = NULL;
	pm_runtime_disable(kbdev->dev);
}

struct kbase_platform_funcs_conf platform_funcs = {
	.platform_init_func = platform_init,
	.platform_term_func = platform_term
};

static int __init mtk_mfg_corex(void)
{
	int ret;

	ret = platform_driver_register(&mtk_gpu_corex_driver);
	if (ret != 0)
		pr_debug("%s: Failed to register GPU core driver", __func__);

	return ret;
}

subsys_initcall(mtk_mfg_corex);
