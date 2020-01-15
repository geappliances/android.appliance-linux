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
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#define ENUM_TO_STR(enum) #enum

enum PINCTRL_PIN_STATE {
	PIN_STATE_DEFAULT = 0,
	PIN_STATE_MAX
};

enum mtkfile_pcm_state {
	MTKFILE_PCM_STATE_UNKNOWN = 0,
	MTKFILE_PCM_STATE_OPEN,
	MTKFILE_PCM_STATE_HW_PARAMS,
	MTKFILE_PCM_STATE_PREPARE,
	MTKFILE_PCM_STATE_START,
	MTKFILE_PCM_STATE_PAUSE,
	MTKFILE_PCM_STATE_RESUME,
	MTKFILE_PCM_STATE_DRAIN,
	MTKFILE_PCM_STATE_STOP,
	MTKFILE_PCM_STATE_HW_FREE,
	MTKFILE_PCM_STATE_CLOSE,
	MTKFILE_PCM_STATE_NUM,
};

static const char *const pcm_state_func[] = {
	ENUM_TO_STR(MTKFILE_PCM_STATE_UNKNOWN),
	ENUM_TO_STR(MTKFILE_PCM_STATE_OPEN),
	ENUM_TO_STR(MTKFILE_PCM_STATE_HW_PARAMS),
	ENUM_TO_STR(MTKFILE_PCM_STATE_PREPARE),
	ENUM_TO_STR(MTKFILE_PCM_STATE_START),
	ENUM_TO_STR(MTKFILE_PCM_STATE_PAUSE),
	ENUM_TO_STR(MTKFILE_PCM_STATE_RESUME),
	ENUM_TO_STR(MTKFILE_PCM_STATE_DRAIN),
	ENUM_TO_STR(MTKFILE_PCM_STATE_STOP),
	ENUM_TO_STR(MTKFILE_PCM_STATE_HW_FREE),
	ENUM_TO_STR(MTKFILE_PCM_STATE_CLOSE),
};

static const char * const nfy_ctl_names[] = {
	"Master Volume",
	"Master Volume X",
	"Master Switch",
	"Master Switch X",
	"PCM State",
	"PCM State X",
};

enum {
	MASTER_VOLUME_ID = 0,
	MASTER_VOLUMEX_ID,
	MASTER_SWITCH_ID,
	MASTER_SWITCHX_ID,
	PCM_STATE_ID,
	PCM_STATEX_ID,
	CTRL_NOTIFY_NUM,
	CTRL_NOTIFY_INVAL = 0xFFFF,
};
struct soc_ctlx_res {
	int master_volume;
	int master_switch;
	int pcm_state;
	struct snd_ctl_elem_id nfy_ids[CTRL_NOTIFY_NUM];
	struct mutex res_mutex;
	spinlock_t res_lock;
};

struct mt8516_vesper_priv {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_states[PIN_STATE_MAX];
	struct soc_ctlx_res ctlx_res;
};

struct adc_config {
	char *name;
	unsigned int tdm_mask;
};


static const char * const mt8516_vesper_pinctrl_pin_str[PIN_STATE_MAX] = {
	"default",
};

static SOC_ENUM_SINGLE_EXT_DECL(pcm_state_enums, pcm_state_func);

/* ctrl resource manager */
static inline int soc_ctlx_init(struct soc_ctlx_res *ctlx_res,
				struct snd_soc_card *soc_card)
{
	int i;
	struct snd_card *card = soc_card->snd_card;
	struct snd_kcontrol *control;

	ctlx_res->master_volume = 100;
	ctlx_res->master_switch = 1;
	ctlx_res->pcm_state = MTKFILE_PCM_STATE_UNKNOWN;
	mutex_init(&ctlx_res->res_mutex);
	spin_lock_init(&ctlx_res->res_lock);

	for (i = 0; i < CTRL_NOTIFY_NUM; i++) {
		list_for_each_entry(control, &card->controls, list) {
			if (strncmp(control->id.name, nfy_ctl_names[i],
				    sizeof(control->id.name)))
				continue;
			ctlx_res->nfy_ids[i] = control->id;
		}
	}

	return 0;
}

static int soc_ctlx_get(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kctl);
	struct mt8516_vesper_priv *card_data = snd_soc_card_get_drvdata(card);
	struct soc_ctlx_res *res_mgr = &card_data->ctlx_res;
	int type;

	for (type = 0; type < CTRL_NOTIFY_NUM; type++) {
		if (kctl->id.numid == res_mgr->nfy_ids[type].numid)
			break;
	}
	if (type == CTRL_NOTIFY_NUM) {
		pr_err("invalid mixer control(numid:%d)\n", kctl->id.numid);
		return -EINVAL;
	}

	mutex_lock(&res_mgr->res_mutex);
	switch (type) {
	case MASTER_VOLUME_ID:
	case MASTER_VOLUMEX_ID:
		ucontrol->value.integer.value[0] = res_mgr->master_volume;
		break;
	case MASTER_SWITCH_ID:
	case MASTER_SWITCHX_ID:
		ucontrol->value.integer.value[0] = res_mgr->master_switch;
		break;
	default:
		break;
	}
	mutex_unlock(&res_mgr->res_mutex);

	return 0;
}

static int soc_ctlx_put(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kctl);
	struct mt8516_vesper_priv *card_data = snd_soc_card_get_drvdata(card);
	struct soc_ctlx_res *res_mgr = &card_data->ctlx_res;
	int type;
	int nfy_type;
	int need_notify_self = 0;
	int *value = NULL;

	for (type = 0; type < CTRL_NOTIFY_NUM; type++) {
		if (kctl->id.numid == res_mgr->nfy_ids[type].numid)
			break;
	}
	if (type == CTRL_NOTIFY_NUM) {
		pr_err("invalid mixer control(numid:%d)\n", kctl->id.numid);
		return -EINVAL;
	}

	mutex_lock(&res_mgr->res_mutex);
	switch (type) {
	case MASTER_VOLUME_ID:
		if ((res_mgr->master_switch == 1) ||
			(ucontrol->value.integer.value[0] != 0)) {
			nfy_type = MASTER_VOLUMEX_ID;
			value = &res_mgr->master_volume;
			need_notify_self = 1;
		}
		break;
	case MASTER_VOLUMEX_ID:
		nfy_type = MASTER_VOLUME_ID;
		value = &res_mgr->master_volume;
		break;
	case MASTER_SWITCH_ID:
		nfy_type = MASTER_SWITCHX_ID;
		value = &res_mgr->master_switch;
		need_notify_self = 1;
		break;
	case MASTER_SWITCHX_ID:
		nfy_type = MASTER_SWITCH_ID;
		value = &res_mgr->master_switch;
		break;
	default:
		break;
	}
	if (value != NULL) {
		*value = ucontrol->value.integer.value[0];
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &(res_mgr->nfy_ids[nfy_type]));
	} else {
		nfy_type = CTRL_NOTIFY_INVAL;
	}
	if (need_notify_self) {
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &(kctl->id));
	}
	mutex_unlock(&res_mgr->res_mutex);

	return 0;
}

static int soc_pcm_state_get(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kctl);
	struct mt8516_vesper_priv *card_data = snd_soc_card_get_drvdata(card);
	struct soc_ctlx_res *res_mgr = &card_data->ctlx_res;
	unsigned long flags;

	spin_lock_irqsave(&res_mgr->res_lock, flags);
	ucontrol->value.integer.value[0] = res_mgr->pcm_state;
	spin_unlock_irqrestore(&res_mgr->res_lock, flags);

	return 0;
}

static int soc_pcm_state_put(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kctl);
	struct mt8516_vesper_priv *card_data = snd_soc_card_get_drvdata(card);
	struct soc_ctlx_res *res_mgr = &card_data->ctlx_res;
	unsigned long flags;

	spin_lock_irqsave(&res_mgr->res_lock, flags);
	if (ucontrol->value.integer.value[0] != res_mgr->pcm_state) {
		res_mgr->pcm_state = ucontrol->value.integer.value[0];
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &(res_mgr->nfy_ids[PCM_STATEX_ID]));
	}
	spin_unlock_irqrestore(&res_mgr->res_lock, flags);

	return 0;
}

static const struct snd_kcontrol_new mt8516_vesper_soc_controls[] = {
	/* for third party app use */
	SOC_SINGLE_EXT("Master Volume",
			    0,
			    0,
			    100,
			    0,
			    soc_ctlx_get,
			    soc_ctlx_put),
	SOC_SINGLE_EXT("Master Volume X",
			    0,
			    0,
			    100,
			    0,
			    soc_ctlx_get,
			    soc_ctlx_put),
	SOC_SINGLE_BOOL_EXT("Master Switch",
			    0,
			    soc_ctlx_get,
			    soc_ctlx_put),
	SOC_SINGLE_BOOL_EXT("Master Switch X",
			    0,
			    soc_ctlx_get,
			    soc_ctlx_put),
	SOC_ENUM_EXT("PCM State",
		     pcm_state_enums,
		     soc_pcm_state_get,
		     soc_pcm_state_put),
	SOC_ENUM_EXT("PCM State X",
		     pcm_state_enums,
		     soc_pcm_state_get,
		     0),
};

static struct adc_config vesper_config[] = {
	{.name = "pcm186x.1-004a", .tdm_mask = 0x0f},
	{.name = "tlv320adc3101.1-0018", .tdm_mask = 0x30},
};

static int tdmin_capture_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int i, j;

	for (i = 0; i < rtd->num_codecs; i++) {
		for (j = 0; j < ARRAY_SIZE(vesper_config); j++) {
			if (!strcmp(rtd->codec_dais[i]->component->name,
			vesper_config[j].name)) {
				snd_soc_dai_set_tdm_slot(rtd->codec_dais[i],
				vesper_config[j].tdm_mask, 0, 8, 32);
				break;
			}
		}
	}

	return 0;
}

static void tdmin_capture_shutdown(struct snd_pcm_substream *substream)
{
	pr_notice("%s\n", __func__);
}

static int tdmin_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int rate = params_rate(params);
	unsigned int mclk_rate = rate * 256;
	int i;

	pr_notice("%s setting mclk to %u Hz\n", __func__, mclk_rate);

	/* codec mclk */
	for (i = 0; i < rtd->num_codecs; i++)
		snd_soc_dai_set_sysclk(rtd->codec_dais[i], 0, mclk_rate,
				       SND_SOC_CLOCK_IN);

	return 0;
}

static struct snd_soc_ops tdmin_capture_ops = {
	   .startup = tdmin_capture_startup,
	   .shutdown = tdmin_capture_shutdown,
	   .hw_params = tdmin_hw_params,
};

static struct snd_soc_dai_link_component tdm_in_codecs[] = {
	{.name = "pcm186x.1-004a", .dai_name = "pcm1865-aif" },
	{.name = "tlv320adc3101.1-0018", .dai_name = "tlv320adc3101-aif" },
};

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN")),
	DAILINK_COMP_ARRAY(
		COMP_CODEC("pcm186x.1-004a", "pcm1865-aif"),
		COMP_CODEC("tlv320adc3101.1-0018", "tlv320adc3101-aif")),
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
		.codecs = tdm_in_codecs,
		.num_codecs = 2,
		.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_NB_NF |
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
	.controls = mt8516_vesper_soc_controls,
	.num_controls = ARRAY_SIZE(mt8516_vesper_soc_controls),
};

static int mt8516_vesper_gpio_probe(struct snd_soc_card *card)
{
	struct mt8516_vesper_priv *card_data;
	int ret = 0;
	int i;

	card_data = snd_soc_card_get_drvdata(card);

	card_data->pinctrl = devm_pinctrl_get(card->dev);
	if (IS_ERR(card_data->pinctrl)) {
		ret = PTR_ERR(card_data->pinctrl);
		dev_err(card->dev, "%s pinctrl_get failed %d\n",
			__func__, ret);
		goto exit;
	}

	for (i = 0 ; i < PIN_STATE_MAX ; i++) {
		card_data->pin_states[i] =
			pinctrl_lookup_state(card_data->pinctrl,
				mt8516_vesper_pinctrl_pin_str[i]);
		if (IS_ERR(card_data->pin_states[i])) {
			ret = PTR_ERR(card_data->pin_states[i]);
			dev_warn(card->dev, "%s Can't find pinctrl state %s %d\n",
				__func__,
				mt8516_vesper_pinctrl_pin_str[i], ret);
		}
	}

exit:
	return ret;
}

static int mt8516_vesper_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8516_vesper_card;
	struct device_node *platform_node;
	struct device_node *codec_node;
	int ret, i;
	struct mt8516_vesper_priv *card_data;

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

	card_data = devm_kzalloc(&pdev->dev,
		sizeof(struct mt8516_vesper_priv), GFP_KERNEL);
	if (!card_data) {
		ret = -ENOMEM;
		dev_err(&pdev->dev,
			"%s allocate card private data fail %d\n",
			__func__, ret);
		return ret;
	}

	snd_soc_card_set_drvdata(card, card_data);

	mt8516_vesper_gpio_probe(card);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
		__func__, ret);
		return ret;
	}
	soc_ctlx_init(&card_data->ctlx_res, card);

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
