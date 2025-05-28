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

//#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <mali_kbase.h>
#include "mali_kbase_config_platform.h"

 
static struct platform_device *probe_gpu_core1_dev;
static struct platform_device *probe_gpu_core2_dev;

static const struct of_device_id mtk_gpu_corex_of_ids[] = {
	{ .compatible = "mediatek,gpu_core1", .data = "1" },
	{ .compatible = "mediatek,gpu_core2", .data = "2" },
	{}
};

MODULE_DEVICE_TABLE(of, mtk_gpu_corex_of_ids);

static int mtk_gpu_corex_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	const char *tmp;

	match = of_match_device(mtk_gpu_corex_of_ids, dev);
	if (!match)
		return -ENODEV;
	tmp = match->data;
	if (*tmp == '1') {
		dev_info(dev, "gpu core1 prob\n");
		probe_gpu_core1_dev = pdev;
	} else if (*tmp == '2') {
		dev_info(dev, "gpu core2 prob\n");
		probe_gpu_core2_dev = pdev;
	} else {
		dev_warn(dev, "Unexpected, gpu core\n");
	}


	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	//dev_info(dev, "gpu core%s prob, %s\n", *tmp);
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

static int __init mtk_mfg_corex_init(void)
{
	int ret;

	ret = platform_driver_register(&mtk_gpu_corex_driver);
	if (ret != 0)
		pr_debug("%s: Failed to register GPU core driver", __func__);

	return ret;
}

static void __exit mtk_mfg_corex_exit(void)
{
	platform_driver_unregister(&mtk_gpu_corex_driver);
}

module_init(mtk_mfg_corex_init);
module_exit(mtk_mfg_corex_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mediatek gpu core driver");
