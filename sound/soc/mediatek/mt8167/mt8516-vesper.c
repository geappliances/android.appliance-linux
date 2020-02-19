// SPDX-License-Identifier: GPL-2.0
/*
 * mt8516-vesper.c  --  MT8516-Vesper ALSA SoC machine driver
 *
 * Copyright (c) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <sound/soc.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

struct adc_config {
	char *name;
	unsigned int tdm_mask;
};

struct vesper_config {
	char *dai_link_name;
	struct adc_config *tdm_cfg;
	int num_cfg;
};

static struct adc_config vesper_cfg_3_1[] = {
	{.name = "pcm186x.1-004a", .tdm_mask = 0x0f},
	{.name = "tlv320adc3101.1-0018", .tdm_mask = 0x30},
	{.name = "pcm186x.1-004b", .tdm_mask = 0xc0},
};

static struct adc_config vesper_cfg_6_1[] = {
	{.name = "pcm186x.1-004a", .tdm_mask = 0x0f},
	{.name = "pcm186x.1-004b", .tdm_mask = 0xf0},
};

static struct vesper_config vesper_cfg[] = {
	{.dai_link_name = "TDM Capture",
	 .tdm_cfg = vesper_cfg_3_1,
	 .num_cfg = ARRAY_SIZE(vesper_cfg_3_1)},
	{.dai_link_name = "TDM Capture 6.1",
	 .tdm_cfg = vesper_cfg_6_1,
	 .num_cfg = ARRAY_SIZE(vesper_cfg_6_1)},
};

static int tdmin_capture_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int i, j, k;

	for (i = 0; i < rtd->num_codecs; i++)
		if (rtd->codec_dais[i]->active)
			return -EBUSY;

	for (k = 0; k < ARRAY_SIZE(vesper_cfg); k++)
		if (!strcmp(rtd->dai_link->name, vesper_cfg[k].dai_link_name))
			break;

	if (k == ARRAY_SIZE(vesper_cfg))
		return 0;

	for (i = 0; i < rtd->num_codecs; i++) {
		for (j = 0; j < vesper_cfg[k].num_cfg; j++) {
			if (!strcmp(rtd->codec_dais[i]->component->name,
				    vesper_cfg[k].tdm_cfg[j].name)) {
				snd_soc_dai_set_tdm_slot(rtd->codec_dais[i],
				vesper_cfg[k].tdm_cfg[j].tdm_mask, 0, 8, 32);
				break;
			}
		}
	}

	return 0;
}

static int tdmin_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int rate = params_rate(params);
	unsigned int mclk_rate = rate * 256;
	int i;

	/* codec mclk */
	for (i = 0; i < rtd->num_codecs; i++)
		snd_soc_dai_set_sysclk(rtd->codec_dais[i], 0, mclk_rate,
				       SND_SOC_CLOCK_IN);

	return 0;
}

static struct snd_soc_ops tdmin_capture_ops = {
	   .startup = tdmin_capture_startup,
	   .hw_params = tdmin_hw_params,
};

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm_capture_6_1,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN")),
	DAILINK_COMP_ARRAY(
		COMP_CODEC("pcm186x.1-004a", "pcm1865-aif"),
		COMP_CODEC("pcm186x.1-004b", "pcm1865-aif")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN")),
	DAILINK_COMP_ARRAY(
		COMP_CODEC("pcm186x.1-004a", "pcm1865-aif"),
		COMP_CODEC("tlv320adc3101.1-0018", "tlv320adc3101-aif"),
		COMP_CODEC("pcm186x.1-004b", "pcm1865-aif")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE */
SND_SOC_DAILINK_DEFS(tdm_in_io,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN_IO")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S")),
	DAILINK_COMP_ARRAY(COMP_CODEC("pcm512x.1-004c", "pcm512x-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link mt8516_vesper_dais[] = {
	/* Front End DAI links */
	{
		.name = "TDM Capture",
		.stream_name = "TDM_Capture",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		.ops = &tdmin_capture_ops,
		SND_SOC_DAILINK_REG(tdm_capture),
	},
	{
		.name = "TDM Capture 6.1",
		.stream_name = "TDM_Capture_6_1",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		.ops = &tdmin_capture_ops,
		SND_SOC_DAILINK_REG(tdm_capture_6_1),
	},
	{
		.name = "DL1 Playback",
		.stream_name = "DL1_Playback",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(playback1),
	},

	/* Backend End DAI links */
	{
		.name = "TDM IN BE",
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(tdm_in_io),
	},
	{
		.name = "I2S BE",
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(i2s),
	},
};

static struct snd_soc_card mt8516_vesper_card = {
	.name = "mt-snd-card",
	.owner = THIS_MODULE,
	.dai_link = mt8516_vesper_dais,
	.num_links = ARRAY_SIZE(mt8516_vesper_dais),
};

static int mt8516_vesper_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8516_vesper_card;
	struct device_node *platform_node;
	struct device_node *codec_node;
	int ret, i;

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}
	codec_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,audio-codec", 0);
	if (!codec_node) {
		dev_err(&pdev->dev, "Property 'audio-codec' missing or invalid\n");
		return -EINVAL;
	}

	for (i = 0; i < card->num_links; i++) {
		if (mt8516_vesper_dais[i].platforms->name)
			continue;
		mt8516_vesper_dais[i].platforms->of_node = platform_node;
	}

	card->dev = &pdev->dev;

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
		__func__, ret);
		return ret;
	}

	return ret;
}

static const struct of_device_id mt8516_vesper_dt_match[] = {
	{ .compatible = "mediatek,mt8516-soc-vesper", },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8516_vesper_dt_match);

static struct platform_driver mt8516_vesper_mach_driver = {
	.driver = {
		   .name = "mt8516-soc-vesper",
		   .of_match_table = mt8516_vesper_dt_match,
#ifdef CONFIG_PM
		   .pm = &snd_soc_pm_ops,
#endif
	},
	.probe = mt8516_vesper_dev_probe,
};

module_platform_driver(mt8516_vesper_mach_driver);

/* Module information */
MODULE_DESCRIPTION("MT8516-Vesper ALSA SoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mt8516-vesper");
