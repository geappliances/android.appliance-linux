// SPDX-License-Identifier: GPL-2.0

/*
 * rpi_backlight.c - Backlight controller through i2c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <drm/drm_panel.h>

/* I2C register of the Atmel microcontroller that controls brightness */
#define REG_PWM 0x86

#define MIN_BRIGHTNESS		0
#define MAX_BRIGHTNESS		255
#define DEFAULT_BRIGHTNESS	150

struct rpi_backlight_t {
	struct i2c_client *i2c;
	struct backlight_device *bl;
	struct device *dev;
};

static int rpi_backlight_update_status(struct backlight_device *bl)
{
	struct rpi_backlight_t *rpi_backlight = bl_get_data(bl);
	int brightness = bl->props.brightness;

	if (!rpi_backlight)
		return -ENOMEM;

	return i2c_smbus_write_byte_data(rpi_backlight->i2c, REG_PWM,
					 brightness);
}

static const struct backlight_ops rpi_backlight_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= rpi_backlight_update_status,
};

static int rpi_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	struct rpi_backlight_t *rpi_backlight;
	struct device_node *np;
	struct i2c_client *i2c;

	rpi_backlight =
		devm_kzalloc(&pdev->dev, sizeof(*rpi_backlight), GFP_KERNEL);

	if (!rpi_backlight)
		return -ENOMEM;

	np = of_find_compatible_node(NULL, NULL,
				     "pumpkin,7inch-touchscreen-panel");
	if (!np)
		return -ENOENT;

	i2c = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!i2c)
		return -EPROBE_DEFER;


	rpi_backlight->i2c = i2c;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;

	bl = devm_backlight_device_register(&pdev->dev, dev_name(&pdev->dev),
					    &pdev->dev, rpi_backlight,
					    &rpi_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		put_device(&i2c->dev);
		return PTR_ERR(bl);
	}
	rpi_backlight->bl = bl;


	bl->props.brightness = DEFAULT_BRIGHTNESS;
	backlight_update_status(rpi_backlight->bl);
	platform_set_drvdata(pdev, rpi_backlight->bl);

	return 0;
}

static int rpi_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct rpi_backlight_t *rpi_backlight = bl_get_data(bl);

	put_device(&rpi_backlight->i2c->dev);
}

static const struct of_device_id rpi_backlight_of_match[] = {
	{ .compatible = "raspberrypi,rpi-backlight" },
};
MODULE_DEVICE_TABLE(of, rpi_backlight_of_match);

static struct platform_driver rpi_backlight_driver = {
	.driver		= {
		.name		= "rpi-backlight",
		.of_match_table = of_match_ptr(rpi_backlight_of_match),
	},
	.probe		= rpi_backlight_probe,
	.remove		= rpi_backlight_remove,
};

module_platform_driver(rpi_backlight_driver);

MODULE_AUTHOR("Taha HAMDI <thamdi@baylibre.com>");
MODULE_DESCRIPTION("Raspberry Pi Backlight Driver");
MODULE_LICENSE("GPL");
