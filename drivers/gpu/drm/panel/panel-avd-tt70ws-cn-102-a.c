// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 BayLibre
 * Author: Amjad Ouled-Ameur <aouledameur@baylibre.com>
 */

#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <video/videomode.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>

// Default Video Mode configuration
#define DCLK (53560)
#define HDISPLAY (1024)
#define VDISPLAY (600)
#define HSYNC_FRONT_PORCH (160)
#define HSYNC_BACK_PORCH (160)
#define HSYNC_PULSE_WIDTH (40)
#define HSYNC_START (HDISPLAY + HSYNC_FRONT_PORCH)
#define HSYNC_END (HSYNC_START + HSYNC_PULSE_WIDTH)
#define HSYNC_TOTAL (HSYNC_END + HSYNC_BACK_PORCH)
#define VSYNC_FRONT_PORCH (12)
#define VSYNC_BACK_PORCH (23)
#define VSYNC_PULSE_WIDTH (10)
#define VSYNC_START (VDISPLAY + VSYNC_FRONT_PORCH)
#define VSYNC_END (VSYNC_START + VSYNC_PULSE_WIDTH)
#define VSYNC_TOTAL (VSYNC_END + VSYNC_BACK_PORCH)
#define VREFRESH (60)
#define VIDEOMODE_FLAGS (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC)

struct lvds_panel {
	struct drm_panel panel;
	struct device *dev;

	unsigned int width;
	unsigned int height;
	struct videomode video_mode;
	unsigned int bus_format;
	bool data_mirror;

	struct gpio_desc *lcd_stb_gpio;
	struct gpio_desc *lcd_en_gpio;
	struct gpio_desc *lcd_rst_gpio;

	bool prepared;
	bool enabled;

	struct backlight_device *backlight;
};

static const struct drm_display_mode default_mode = {
	.clock = DCLK,
	.hdisplay = HDISPLAY,
	.vdisplay = VDISPLAY,
	.hsync_start = HSYNC_START,
	.hsync_end = HSYNC_END,
	.htotal = HSYNC_TOTAL,
	.vsync_start = VSYNC_START,
	.vsync_end = VSYNC_END,
	.vtotal = VSYNC_TOTAL,
	.vrefresh = VREFRESH,
	.flags = VIDEOMODE_FLAGS,
};

static inline struct lvds_panel *to_lvds_panel(struct drm_panel *panel)
{
	return container_of(panel, struct lvds_panel, panel);
}

static int lvds_panel_disable(struct drm_panel *panel)
{
	struct lvds_panel *lvds = to_lvds_panel(panel);

	if (!lvds->enabled)
		return 0;

	backlight_disable(lvds->backlight);
	lvds->enabled = false;

	return 0;
}

static int lvds_panel_unprepare(struct drm_panel *panel)
{
	struct lvds_panel *lvds = to_lvds_panel(panel);

	if (!lvds->prepared)
		return 0;

	gpiod_set_value(lvds->lcd_stb_gpio, 0);
	mdelay(100);
	gpiod_set_value(lvds->lcd_en_gpio, 0);
	gpiod_set_value(lvds->lcd_rst_gpio, 0);

	lvds->prepared = false;

	return 0;
}

static int lvds_panel_prepare(struct drm_panel *panel)
{
	struct lvds_panel *lvds = to_lvds_panel(panel);

	if (lvds->prepared)
		return 0;

	gpiod_set_value(lvds->lcd_stb_gpio, 0);
	mdelay(50);
	gpiod_set_value(lvds->lcd_en_gpio, 0);
	gpiod_set_value(lvds->lcd_rst_gpio, 0);
	mdelay(200);

	gpiod_set_value(lvds->lcd_en_gpio, 1);
	mdelay(12);

	gpiod_set_value(lvds->lcd_rst_gpio, 1);
	mdelay(14);

	gpiod_set_value(lvds->lcd_stb_gpio, 1);
	mdelay(10);

	lvds->prepared = true;

	return 0;
}

static int lvds_panel_enable(struct drm_panel *panel)
{
	struct lvds_panel *lvds = to_lvds_panel(panel);

	if (lvds->enabled)
		return 0;

	backlight_enable(lvds->backlight);
	lvds->enabled = true;

	return 0;
}

static int lvds_panel_get_modes(struct drm_panel *panel)
{
	struct lvds_panel *lvds = to_lvds_panel(panel);
	struct drm_connector *connector = lvds->panel.connector;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%ux@%u\n",
				default_mode.hdisplay, default_mode.vdisplay,
				drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = lvds->width;
	connector->display_info.height_mm = lvds->height;
	drm_display_info_set_bus_formats(&connector->display_info,
					 &lvds->bus_format, 1);
	connector->display_info.bus_flags = lvds->data_mirror
					  ? DRM_BUS_FLAG_DATA_LSB_TO_MSB
					  : DRM_BUS_FLAG_DATA_MSB_TO_LSB;

	return 1;
}

static const struct drm_panel_funcs lvds_panel_funcs = {
	.disable = lvds_panel_disable,
	.unprepare = lvds_panel_unprepare,
	.prepare = lvds_panel_prepare,
	.enable = lvds_panel_enable,
	.get_modes = lvds_panel_get_modes,
};

static int avd_lvds_panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct lvds_panel *lvds;
	int ret;

	lvds = devm_kzalloc(dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	lvds->dev = dev;

	lvds->lcd_stb_gpio = devm_gpiod_get(dev, "lcd_stb", GPIOD_OUT_LOW);
	if (IS_ERR(lvds->lcd_stb_gpio)) {
		ret = PTR_ERR(lvds->lcd_stb_gpio);
		dev_err(dev, "failed to request %s GPIO: %d\n",
			"lcd_stb", ret);
		return ret;
	}

	lvds->lcd_en_gpio = devm_gpiod_get(dev, "lcd_en", GPIOD_OUT_LOW);
	if (IS_ERR(lvds->lcd_en_gpio)) {
		ret = PTR_ERR(lvds->lcd_en_gpio);
		dev_err(dev, "failed to request %s GPIO: %d\n",
			"enable", ret);
		return ret;
	}

	lvds->lcd_rst_gpio = devm_gpiod_get(dev, "lcd_rst", GPIOD_OUT_LOW);
	if (IS_ERR(lvds->lcd_rst_gpio)) {
		ret = PTR_ERR(lvds->lcd_rst_gpio);
		dev_err(dev, "failed to request %s GPIO: %d\n",
			"reset", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "width-mm", &lvds->width);
	if (ret < 0) {
		dev_err(lvds->dev, "%pOF: invalid or missing %s DT property\n",
			np, "width-mm");
		return -ENODEV;
	}
	ret = of_property_read_u32(np, "height-mm", &lvds->height);
	if (ret < 0) {
		dev_err(lvds->dev, "%pOF: invalid or missing %s DT property\n",
			np, "height-mm");
		return -ENODEV;
	}

	lvds->data_mirror = of_property_read_bool(np, "data-mirror");

	lvds->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(lvds->backlight)) {
		dev_err(dev, "failed to get backlight\n");
		return PTR_ERR(lvds->backlight);
	}

	// VESA 8-bit format
	lvds->bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG;

	drm_panel_init(&lvds->panel);
	lvds->panel.dev = dev;
	lvds->panel.funcs = &lvds_panel_funcs;

	ret = drm_panel_add(&lvds->panel);
	if (ret)
		return ret;

	dev_set_drvdata(dev, lvds);

	return 0;
}

static int avd_lvds_panel_remove(struct platform_device *pdev)
{
	struct lvds_panel *lvds = dev_get_drvdata(&pdev->dev);

	drm_panel_remove(&lvds->panel);

	return 0;
}

static const struct of_device_id avd_lvds_of_match[] = {
	{ .compatible = "mediatek,panel-avd-tt70ws-cn-102-a", },
	{ }
};
MODULE_DEVICE_TABLE(of, avd_lvds_of_match);

static struct platform_driver avd_lvds_panel_driver = {
	.probe = avd_lvds_panel_probe,
	.remove = avd_lvds_panel_remove,
	.driver = {
		.name = "panel-avd-tt70ws-cn-102-a",
		.of_match_table = avd_lvds_of_match,
	},
};
module_platform_driver(avd_lvds_panel_driver);

MODULE_AUTHOR("Amjad Ouled-Ameur <aouledameur@baylibre.com>");
MODULE_DESCRIPTION("AVD TT70WS-CN-102-A LVDS Panel Driver");
MODULE_LICENSE("GPL v2");
