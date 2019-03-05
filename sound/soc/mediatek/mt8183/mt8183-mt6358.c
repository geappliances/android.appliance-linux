// SPDX-License-Identifier: GPL-2.0
//
// mt8183-mt6358.c
//	--  MT8183-MT6358 ALSA SoC machine driver
//
// Copyright (c) 2018 MediaTek Inc.
// Author: KaiChieh Chuang <kaichieh.chuang@mediatek.com>

#include <linux/module.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/pinctrl/consumer.h>

#include "mt8183-afe-common.h"

static int mt8183_mt6358_i2s_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int rate = params_rate(params);
	unsigned int mclk_fs_ratio = 128;
	unsigned int mclk_fs = rate * mclk_fs_ratio;

	return snd_soc_dai_set_sysclk(rtd->cpu_dai,
				      0, mclk_fs, SND_SOC_CLOCK_OUT);
}

static const struct snd_soc_ops mt8183_mt6358_i2s_ops = {
	.hw_params = mt8183_mt6358_i2s_hw_params,
};

static struct snd_soc_dai_link mt8183_mt6358_dai_links[] = {
	/* FE */
	{
		.name = "Playback_1",
		.stream_name = "Playback_1",
		.cpu_dai_name = "DL1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_2",
		.stream_name = "Playback_2",
		.cpu_dai_name = "DL2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_3",
		.stream_name = "Playback_3",
		.cpu_dai_name = "DL3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Capture_1",
		.stream_name = "Capture_1",
		.cpu_dai_name = "UL1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_2",
		.stream_name = "Capture_2",
		.cpu_dai_name = "UL2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_3",
		.stream_name = "Capture_3",
		.cpu_dai_name = "UL3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_Mono_1",
		.stream_name = "Capture_Mono_1",
		.cpu_dai_name = "UL_MONO_1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	/* BE */
	{
		.name = "Primary Codec",
		.cpu_dai_name = "ADDA",
		.codec_dai_name = "mt6358-snd-codec-aif1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
};

static struct snd_soc_card mt8183_mt6358_card = {
	.name = "mt8183-mt6358",
	.owner = THIS_MODULE,
	.dai_link = mt8183_mt6358_dai_links,
	.num_links = ARRAY_SIZE(mt8183_mt6358_dai_links),
};

static int mt8183_mt6358_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8183_mt6358_card;
	struct device_node *platform_node;
	struct snd_soc_dai_link *dai_link;
	struct pinctrl *default_pins;
	int ret, i;

	card->dev = &pdev->dev;

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}

	for_each_card_prelinks(card, i, dai_link) {
		dai_link->platform = NULL;

		if (dai_link->platform_name)
			continue;
		dai_link->platform_of_node = platform_node;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);
		return ret;
	}

	default_pins =
		devm_pinctrl_get_select(&pdev->dev, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(default_pins)) {
		dev_err(&pdev->dev, "%s set pins failed\n",
			__func__);
		return PTR_ERR(default_pins);
	}

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id mt8183_mt6358_dt_match[] = {
	{.compatible = "mediatek,mt8183-mt6358-sound",},
	{}
};
#endif

static struct platform_driver mt8183_mt6358_driver = {
	.driver = {
		.name = "mt8183-mt6358",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt8183_mt6358_dt_match,
#endif
	},
	.probe = mt8183_mt6358_dev_probe,
};

module_platform_driver(mt8183_mt6358_driver);

/* Module information */
MODULE_DESCRIPTION("MT8183-MT6358 ALSA SoC machine driver");
MODULE_AUTHOR("KaiChieh Chuang <kaichieh.chuang@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("mt8183_mt6358 sound card");

