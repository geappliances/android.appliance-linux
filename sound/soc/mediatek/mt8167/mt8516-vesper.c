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
	struct device_node *component_of_node;
	unsigned int tdm_mask;
};

struct vesper_config {
	const char *dai_link_name;
	struct adc_config *tdm_cfg;
	int num_cfg;
};

static struct vesper_config *vesper_cfg;
static int num_vesper_cfg;

static int tdmin_capture_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int i, j, k;

	for (i = 0; i < rtd->num_codecs; i++)
		if (rtd->codec_dais[i]->active)
			return -EBUSY;

	for (k = 0; k < num_vesper_cfg; k++)
		if (!strcmp(rtd->dai_link->name, vesper_cfg[k].dai_link_name))
			break;

	if (k == num_vesper_cfg)
		return 0;

	for (i = 0; i < rtd->num_codecs; i++) {
		for (j = 0; j < vesper_cfg[k].num_cfg; j++) {
			if (rtd->codec_dais[i]->dev->of_node ==
			    vesper_cfg[k].tdm_cfg[j].component_of_node) {
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

/* No codec declared by default in the dai links. They are added
 * dynamically depending on the dt
 */

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm_capture_6_1,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s_8ch_playback,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE */
SND_SOC_DAILINK_DEFS(tdm_in_io,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN_IO")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hdmi,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMIO")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
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
	{
		.name = "I2S 8CH Playback",
		.stream_name = "I2S8CH Playback",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(i2s_8ch_playback),
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
	{
		.name = "HDMI BE",
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(hdmi),
	},
};

static struct snd_soc_card mt8516_vesper_card = {
	.name = "mt-snd-card",
	.owner = THIS_MODULE,
	.dai_link = mt8516_vesper_dais,
	.num_links = ARRAY_SIZE(mt8516_vesper_dais),
};

static int set_card_codec_info(struct snd_soc_card *card)
{
	struct snd_soc_dai_link_component *dai_link_codecs, *dlc;
	struct device_node *dl_node, *c_node;
	struct device *dev = card->dev;
	struct of_phandle_args args;
	struct adc_config *cur_cfg;
	const char *dai_link_name;
	const char *dai_format;
	int cfg_idx, link_idx;
	bool is_tdm_format;
	int num_codecs;
	u32 tdm_mask;
	int ret, i;

	/* Figure out how many vesper tdm configs are needed */
	num_vesper_cfg = 0;
	for_each_child_of_node(dev->of_node, dl_node) {
		if (!of_property_read_string(dl_node, "dai-format",
					     &dai_format)) {
			if (!strcmp(dai_format, "tdm"))
				num_vesper_cfg++;
		}
	}
	/* Allocate the number of vesper tdm configs that are needed */
	vesper_cfg = devm_kcalloc(dev, num_vesper_cfg,
				  sizeof(*vesper_cfg), GFP_KERNEL);
	if (!vesper_cfg)
		return -ENOMEM;

	cfg_idx = 0;
	/* Loop over all the dai link sub nodes*/
	for_each_child_of_node(dev->of_node, dl_node) {
		if (of_property_read_string(dl_node, "dai-link-name",
					    &dai_link_name))
			return -EINVAL;

		/* Check wether the used format is tdm. If this is the case,
		 * the tdm mask information is stored to be used when the tdm
		 * is started and the tdm slots must be set.
		 */
		is_tdm_format = false;
		if (!of_property_read_string(dl_node, "dai-format",
					     &dai_format)) {
			if (!strcmp(dai_format, "tdm"))
				is_tdm_format = true;
		}

		num_codecs = of_get_child_count(dl_node);
		/* Allocate the snd_soc_dai_link_component array that will be
		 * used to dynamically add the list of codecs to the static
		 * snd_soc_dai_link array.
		 */
		dlc = dai_link_codecs = devm_kcalloc(dev, num_codecs,
					       sizeof(*dai_link_codecs),
					       GFP_KERNEL);
		if (!dai_link_codecs)
			return -ENOMEM;

		if (is_tdm_format) {
			/* Fill the vesper_cfg structure and allocate a number
			 * of tdm_cfg corresponding to the number of codecs.
			 */
			vesper_cfg[cfg_idx].num_cfg = num_codecs;
			vesper_cfg[cfg_idx].dai_link_name = dai_link_name;
			vesper_cfg[cfg_idx].tdm_cfg = devm_kcalloc(dev,
				num_codecs, sizeof(struct adc_config),
				GFP_KERNEL);
			if (!vesper_cfg[cfg_idx].tdm_cfg)
				return -ENOMEM;
		}

		link_idx = 0;
		cur_cfg = vesper_cfg[cfg_idx].tdm_cfg;
		/* Loop over all the codec sub nodes for this dai link */
		for_each_child_of_node(dl_node, c_node) {
			/* Retrieve the node and the dai_name that are used
			 * by the soundcard.
			 */
			ret = of_parse_phandle_with_args(c_node, "sound-dai",
							 "#sound-dai-cells", 0,
							 &args);
			if (ret) {
				if (ret != -EPROBE_DEFER)
					dev_err(dev,
						"can't parse dai %d\n", ret);
				return ret;
			}
			dlc->of_node = args.np;
			ret =  snd_soc_get_dai_name(&args, &dlc->dai_name);
			if (ret) {
				of_node_put(c_node);
				return ret;
			}

			if (is_tdm_format) {
				/* Fill the tdm cfg for this codec */
				if (of_property_read_u32(c_node, "tdm-mask",
							 &tdm_mask))
					return -EINVAL;
				cur_cfg->component_of_node = dlc->of_node;
				cur_cfg->tdm_mask = tdm_mask;
				cur_cfg++;
			}
			dlc++;
			link_idx++;
		}

		/* Update the snd_soc_dai_link static array with the codecs
		 * we have just found.
		 */
		for (i = 0; i < card->num_links; i++) {
			if (!strcmp(dai_link_name, card->dai_link[i].name)) {
				card->dai_link[i].num_codecs = link_idx;
				card->dai_link[i].codecs = dai_link_codecs;
				break;
			}
		}

		if (is_tdm_format)
			cfg_idx++;
	}

	return 0;
}

static int mt8516_vesper_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8516_vesper_card;
	struct device_node *platform_node;
	struct device_node *codec_node;
	int ret, i;

	card->dev = &pdev->dev;
	ret = set_card_codec_info(card);
	if (ret) {
		dev_err(&pdev->dev, "%s set_card_codec_info failed %d\n",
		__func__, ret);
		return ret;
	}

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
