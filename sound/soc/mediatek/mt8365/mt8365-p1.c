/*
 * mt8365-p1.c  --  MT8365 machine driver
 *
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Jia Zeng <jia.zeng@mediatek.com>
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
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "mt8365-afe-common.h"

#define PREFIX	"mediatek,"

#define TEST_BACKEND_WITH_ENDPOINT

enum PINCTRL_PIN_STATE {
	PIN_STATE_MOSI_ON = 0,
	PIN_STATE_MOSI_OFF,
	PIN_STATE_MISO_ON,
	PIN_STATE_MISO_OFF,
	PIN_STATE_DEFAULT,
	PIN_STATE_DMIC,
	PIN_STATE_PCM,
	PIN_STATE_I2S0,
	PIN_STATE_I2S1,
	PIN_STATE_I2S2,
	PIN_STATE_I2S3,
	PIN_STATE_TDM_OUT,
	PIN_STATE_TDM_IN,
	PIN_STATE_MAX
};

static const char * const mt8365_p1_pin_str[PIN_STATE_MAX] = {
	"aud_mosi_on",
	"aud_mosi_off",
	"aud_miso_on",
	"aud_miso_off",
	"default",
	"aud_dmic",
	"aud_pcm",
	"aud_i2s0",
	"aud_i2s1",
	"aud_i2s2",
	"aud_i2s3",
	"aud_tdm_out",
	"aud_tdm_in",
};

struct mt8365_p1_dmic_ctrl_data {
	unsigned int fix_rate;
	unsigned int fix_channels;
	unsigned int fix_bit_width;
};

struct mt8365_p1_tdm_ctrl_data {
	unsigned int tdmin_fix_rate;
	unsigned int tdmin_fix_bit_width;
	unsigned int tdmin_fix_channels;
};

struct mt8365_p1_priv {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_states[PIN_STATE_MAX];
	int ext_amp1_gpio;
	int ext_amp2_gpio;
	unsigned int ext_spk_amp_warmup_time_us;
	unsigned int ext_spk_amp_shutdown_time_us;
	struct mt8365_p1_dmic_ctrl_data dmic_data;
	struct mt8365_p1_tdm_ctrl_data tdm_data;
};

struct mt8365_dai_link_prop {
	char *name;
	unsigned int link_id;
};

enum {
	/* FE */
	DAI_LINK_DL1_PLAYBACK = 0,
	DAI_LINK_DL2_PLAYBACK,
	DAI_LINK_AWB_CAPTURE,
	DAI_LINK_VUL_CAPTURE,
	DAI_LINK_VUL2_CAPTURE,
	DAI_LINK_VUL3_CAPTURE,
	DAI_LINK_TDM_OUT,
	DAI_LINK_TDM_IN,
#ifdef CONFIG_SND_SOC_MTK_BTCVSD
	DAI_LINK_BTCVSD_RX,
	DAI_LINK_BTCVSD_TX,
#endif
	DAI_LINK_FE_HOSTLESS_FM,
	/* BE */
	DAI_LINK_I2S_INTF,
	DAI_LINK_2ND_I2S_INTF,
	DAI_LINK_PCM1_INTF,
	DAI_LINK_VIRTUAL_DL_SOURCE,
	DAI_LINK_VIRTUAL_TDM_OUT_SOURCE,
	DAI_LINK_DMIC,
	DAI_LINK_INT_ADDA,
	DAI_LINK_TDM_OUT_IO,
	DAI_LINK_TDM_IN_IO,
	DAI_LINK_GASRC1,
	DAI_LINK_GASRC2,
	DAI_LINK_TDM_ASRC,
	DAI_LINK_HW_GAIN1,
	DAI_LINK_NUM
};

static void mt8365_p1_ext_hp_amp1_turn_on(struct snd_soc_card *card)
{
	struct mt8365_p1_priv *card_data = snd_soc_card_get_drvdata(card);

	/* enable spk amp1 */
	if (gpio_is_valid(card_data->ext_amp1_gpio))
		gpio_set_value(card_data->ext_amp1_gpio, 1);

	if (card_data->ext_spk_amp_warmup_time_us > 0)
		usleep_range(card_data->ext_spk_amp_warmup_time_us,
			card_data->ext_spk_amp_warmup_time_us + 1);
}

static void mt8365_p1_ext_hp_amp1_turn_off(struct snd_soc_card *card)
{
	struct mt8365_p1_priv *card_data = snd_soc_card_get_drvdata(card);

	/* disable spk amp1 */
	if (gpio_is_valid(card_data->ext_amp1_gpio))
		gpio_set_value(card_data->ext_amp1_gpio, 0);

	if (card_data->ext_spk_amp_shutdown_time_us > 0)
		usleep_range(card_data->ext_spk_amp_shutdown_time_us,
			card_data->ext_spk_amp_shutdown_time_us + 1);
}

static void mt8365_p1_ext_hp_amp2_turn_on(struct snd_soc_card *card)
{
	struct mt8365_p1_priv *card_data = snd_soc_card_get_drvdata(card);

	/* enable spk amp2 */
	if (gpio_is_valid(card_data->ext_amp2_gpio))
		gpio_set_value(card_data->ext_amp2_gpio, 1);

	if (card_data->ext_spk_amp_warmup_time_us > 0)
		usleep_range(card_data->ext_spk_amp_warmup_time_us,
			card_data->ext_spk_amp_warmup_time_us + 1);
}

static void mt8365_p1_ext_hp_amp2_turn_off(struct snd_soc_card *card)
{
	struct mt8365_p1_priv *card_data = snd_soc_card_get_drvdata(card);

	/* disable spk amp2 */
	if (gpio_is_valid(card_data->ext_amp2_gpio))
		gpio_set_value(card_data->ext_amp2_gpio, 0);

	if (card_data->ext_spk_amp_shutdown_time_us > 0)
		usleep_range(card_data->ext_spk_amp_shutdown_time_us,
			card_data->ext_spk_amp_shutdown_time_us + 1);
}

/* HP Spk Amp1 */
static int mt8365_p1_hp_spk_amp1_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;

	dev_dbg(card->dev, "%s, event %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mt8365_p1_ext_hp_amp1_turn_on(card);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mt8365_p1_ext_hp_amp1_turn_off(card);
		break;
	default:
		break;
	}

	return 0;
}

/* HP Spk Amp2 */
static int mt8365_p1_hp_spk_amp2_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;

	dev_dbg(card->dev, "%s, event %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mt8365_p1_ext_hp_amp2_turn_on(card);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mt8365_p1_ext_hp_amp2_turn_off(card);
		break;
	default:
		break;
	}

	return 0;
}

/* LOL Spk Amp1, LOL share same enable pin with AU_HP_R */
static int mt8365_p1_lol_spk_amp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;

	dev_dbg(card->dev, "%s, event %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mt8365_p1_ext_hp_amp1_turn_on(card);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mt8365_p1_ext_hp_amp1_turn_off(card);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new mt8365_p1_hpl_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8365_p1_hpr_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8365_p1_lol_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8365_p1_hpl_output_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8365_p1_hpr_output_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8365_p1_hpl_spk_output_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new mt8365_p1_hpr_spk_output_switch_ctrl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);

static const struct snd_soc_dapm_widget mt8365_p1_widgets[] = {
	SND_SOC_DAPM_SWITCH("AU_HPL",
		SND_SOC_NOPM, 0, 0, &mt8365_p1_hpl_switch_ctrl),
	SND_SOC_DAPM_SWITCH("AU_HPR",
		SND_SOC_NOPM, 0, 0, &mt8365_p1_hpr_switch_ctrl),
	SND_SOC_DAPM_SWITCH("AU_LOL",
		SND_SOC_NOPM, 0, 0, &mt8365_p1_lol_switch_ctrl),
	SND_SOC_DAPM_SWITCH("HP_L Output",
		SND_SOC_NOPM, 0, 0, &mt8365_p1_hpl_output_switch_ctrl),
	SND_SOC_DAPM_SWITCH("HP_R Output",
		SND_SOC_NOPM, 0, 0, &mt8365_p1_hpr_output_switch_ctrl),
	SND_SOC_DAPM_SWITCH("HP_L Spk Output",
		SND_SOC_NOPM, 0, 0, &mt8365_p1_hpl_spk_output_switch_ctrl),
	SND_SOC_DAPM_SWITCH("HP_R Spk Output",
		SND_SOC_NOPM, 0, 0, &mt8365_p1_hpr_spk_output_switch_ctrl),

	SND_SOC_DAPM_SPK("HP Spk Amp1", mt8365_p1_hp_spk_amp1_event),
	SND_SOC_DAPM_SPK("HP Spk Amp2", mt8365_p1_hp_spk_amp2_event),
	SND_SOC_DAPM_SPK("LOL Spk Amp", mt8365_p1_lol_spk_amp_event),
	SND_SOC_DAPM_MIC("PMIC MIC", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
#ifdef TEST_BACKEND_WITH_ENDPOINT
	SND_SOC_DAPM_OUTPUT("TDM_OUT Out"),
#endif
};

static const struct snd_soc_dapm_route mt8365_p1_routes[] = {
	{"AU_HPL", "Switch", "MT6357 Playback"},
	{"AU_HPR", "Switch", "MT6357 Playback"},
	{"AU_LOL", "Switch", "MT6357 Playback"},

	{"HP_L Output", "Switch", "AU_HPL"},
	{"HP_R Output", "Switch", "AU_HPR"},
	{"HP_L Spk Output", "Switch", "AU_HPL"},
	{"HP_R Spk Output", "Switch", "AU_HPR"},

	{"Headphone", NULL, "HP_L Output"},
	{"Headphone", NULL, "HP_R Output"},

	{"HP Spk Amp1", NULL, "HP_R Spk Output"},
	{"HP Spk Amp2", NULL, "HP_L Spk Output"},
	{"LOL Spk Amp", NULL, "AU_LOL"},

	{"MT6357 Capture", NULL, "PMIC MIC"},

#ifdef TEST_BACKEND_WITH_ENDPOINT
	{"TDM_OUT Out", NULL, "TDM_OUT Playback"},
#endif
};

static int mt8365_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
	struct snd_pcm_hw_params *params)
{
	struct mt8365_p1_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	int id = rtd->dai_link->id;
	struct mt8365_p1_tdm_ctrl_data *tdm;
	struct mt8365_p1_dmic_ctrl_data *dmic;
	unsigned int fix_rate = 0;
	unsigned int fix_bit_width = 0;
	unsigned int fix_channels = 0;

	if (id == DAI_LINK_DMIC) {
		dmic = &priv->dmic_data;
		fix_rate = dmic->fix_rate;
		fix_bit_width = dmic->fix_bit_width;
		fix_channels = dmic->fix_channels;
	} else if (id == DAI_LINK_TDM_IN_IO) {
		tdm = &priv->tdm_data;
		fix_rate = tdm->tdmin_fix_rate;
		fix_bit_width = tdm->tdmin_fix_bit_width;
		fix_channels = tdm->tdmin_fix_channels;
	}

	if (fix_rate > 0) {
		struct snd_interval *rate =
			hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

		rate->max = rate->min = fix_rate;
	}

	if (fix_bit_width > 0) {
		struct snd_mask *mask =
			hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

		if (fix_bit_width == 32) {
			snd_mask_none(mask);
			snd_mask_set(mask, SNDRV_PCM_FORMAT_S32_LE);
		} else if (fix_bit_width == 16) {
			snd_mask_none(mask);
			snd_mask_set(mask, SNDRV_PCM_FORMAT_S16_LE);
		}
	}

	if (fix_channels > 0) {
		struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

		channels->min = channels->max = fix_channels;
	}

	return 0;
}

static int mt8365_p1_int_adda_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mt8365_p1_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (IS_ERR(priv->pin_states[PIN_STATE_MOSI_ON]))
			return ret;

		ret = pinctrl_select_state(priv->pinctrl,
					priv->pin_states[PIN_STATE_MOSI_ON]);
		if (ret)
			dev_err(rtd->card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (IS_ERR(priv->pin_states[PIN_STATE_MISO_ON]))
			return ret;

		ret = pinctrl_select_state(priv->pinctrl,
					priv->pin_states[PIN_STATE_MISO_ON]);
		if (ret)
			dev_err(rtd->card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

	return 0;
}

static void mt8365_p1_int_adda_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mt8365_p1_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (IS_ERR(priv->pin_states[PIN_STATE_MOSI_OFF]))
			return;

		ret = pinctrl_select_state(priv->pinctrl,
					priv->pin_states[PIN_STATE_MOSI_OFF]);
		if (ret)
			dev_err(rtd->card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (IS_ERR(priv->pin_states[PIN_STATE_MISO_OFF]))
			return;

		ret = pinctrl_select_state(priv->pinctrl,
					priv->pin_states[PIN_STATE_MISO_OFF]);
		if (ret)
			dev_err(rtd->card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

}

static struct snd_soc_ops mt8365_p1_int_adda_ops = {
	.startup = mt8365_p1_int_adda_startup,
	.shutdown = mt8365_p1_int_adda_shutdown,
};

SND_SOC_DAILINK_DEFS(playback1,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(playback2,
	DAILINK_COMP_ARRAY(COMP_CPU("DL2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(awb_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("AWB")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(vul,
	DAILINK_COMP_ARRAY(COMP_CPU("VUL")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(vul2,
	DAILINK_COMP_ARRAY(COMP_CPU("VUL2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(vul3,
	DAILINK_COMP_ARRAY(COMP_CPU("VUL3")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(tdm_out_fe,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_OUT")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(tdm_in_fe,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hostless_fm,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless FM DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s1,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(i2s2,
	DAILINK_COMP_ARRAY(COMP_CPU("2ND I2S")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(pcm1,
	DAILINK_COMP_ARRAY(COMP_CPU("PCM1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(virtual_dl,
	DAILINK_COMP_ARRAY(COMP_CPU("VIRTUAL_DL_SRC")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(tdm_out,
	DAILINK_COMP_ARRAY(COMP_CPU("VIRTUAL_TDM_OUT_SRC")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(dmic,
	DAILINK_COMP_ARRAY(COMP_CPU("DMIC")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(primary_codec,
	DAILINK_COMP_ARRAY(COMP_CPU("INT ADDA")),
	DAILINK_COMP_ARRAY(COMP_CODEC("mt-soc-codec", "mt6357-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(tdm_out_io,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_OUT_IO")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(tdm_in_io,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN_IO")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(gasrc1,
	DAILINK_COMP_ARRAY(COMP_CPU("GASRC1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(gasrc2,
	DAILINK_COMP_ARRAY(COMP_CPU("GASRC2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(tdm_asrc,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_ASRC")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(hw_gain1,
	DAILINK_COMP_ARRAY(COMP_CPU("HW_GAIN1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link mt8365_p1_dais[] = {
	/* Front End DAI links */
	[DAI_LINK_DL1_PLAYBACK] = {
		.name = "DL1_FE",
		.stream_name = "MultiMedia1_PLayback",
		.id = DAI_LINK_DL1_PLAYBACK,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(playback1),
	},
	[DAI_LINK_DL2_PLAYBACK] = {
		.name = "DL2_FE",
		.stream_name = "MultiMedia2_PLayback",
		.id = DAI_LINK_DL2_PLAYBACK,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(playback2),
	},
	[DAI_LINK_AWB_CAPTURE] = {
		.name = "AWB_FE",
		.stream_name = "DL1_AWB_Record",
		.id = DAI_LINK_AWB_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(awb_capture),
	},
	[DAI_LINK_VUL_CAPTURE] = {
		.name = "VUL_FE",
		.stream_name = "MultiMedia1_Capture",
		.id = DAI_LINK_VUL_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(vul),
	},
	[DAI_LINK_VUL2_CAPTURE] = {
		.name = "VUL2_FE",
		.stream_name = "VUL2_Capture",
		.id = DAI_LINK_VUL2_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(vul2),
	},
	[DAI_LINK_VUL3_CAPTURE] = {
		.name = "VUL3_FE",
		.stream_name = "VUL3_Capture",
		.id = DAI_LINK_VUL3_CAPTURE,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(vul3),
	},
	[DAI_LINK_TDM_OUT] = {
		.name = "TDM_OUT_FE",
		.stream_name = "TDM_Playback",
		.id = DAI_LINK_TDM_OUT,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(tdm_out_fe),
	},
	[DAI_LINK_TDM_IN] = {
		.name = "TDM_IN_FE",
		.stream_name = "TDM_Capture",
		.id = DAI_LINK_TDM_IN,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(tdm_in_fe),
	},
	[DAI_LINK_FE_HOSTLESS_FM] = {
		.name = "Hostless_FM",
		.stream_name = "Hostless_FM",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_fm),
	},
	/* Back End DAI links */
	[DAI_LINK_I2S_INTF] = {
		.name = "I2S BE",
		.no_pcm = 1,
		.id = DAI_LINK_I2S_INTF,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(i2s1),
	},
	[DAI_LINK_2ND_I2S_INTF] = {
		.name = "2ND_I2S BE",
		.no_pcm = 1,
		.id = DAI_LINK_2ND_I2S_INTF,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBM_CFM,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(i2s2),
	},
	[DAI_LINK_PCM1_INTF] = {
		.name = "PCM1 BE",
		.no_pcm = 1,
		.id = DAI_LINK_PCM1_INTF,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(pcm1),
	},
	[DAI_LINK_VIRTUAL_DL_SOURCE] = {
		.name = "VIRTUAL_DL_SRC BE",
		.no_pcm = 1,
		.id = DAI_LINK_VIRTUAL_DL_SOURCE,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(virtual_dl),
	},
	[DAI_LINK_VIRTUAL_TDM_OUT_SOURCE] = {
		.name = "VIRTUAL_TDM_OUT_SRC BE",
		.no_pcm = 1,
		.id = DAI_LINK_VIRTUAL_TDM_OUT_SOURCE,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(tdm_out),
	},
	[DAI_LINK_DMIC] = {
		.name = "DMIC BE",
		.no_pcm = 1,
		.id = DAI_LINK_DMIC,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dmic),
	},
	[DAI_LINK_INT_ADDA] = {
		.name = "MTK Codec",
		.no_pcm = 1,
		.id = DAI_LINK_INT_ADDA,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &mt8365_p1_int_adda_ops,
		SND_SOC_DAILINK_REG(primary_codec),
	},
	[DAI_LINK_TDM_OUT_IO] = {
		.name = "TDM_OUT BE",
		.no_pcm = 1,
		.id = DAI_LINK_TDM_OUT_IO,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(tdm_out_io),
	},
	[DAI_LINK_TDM_IN_IO] = {
		.name = "TDM_IN BE",
		.no_pcm = 1,
		.id = DAI_LINK_TDM_IN_IO,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_IB_IF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(tdm_in_io),
	},
	[DAI_LINK_GASRC1] = {
		.name = "GASRC1 BE",
		.no_pcm = 1,
		.id = DAI_LINK_GASRC1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(gasrc1),
	},
	[DAI_LINK_GASRC2] = {
		.name = "GASRC2 BE",
		.no_pcm = 1,
		.id = DAI_LINK_GASRC2,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(gasrc2),
	},
	[DAI_LINK_TDM_ASRC] = {
		.name = "TDM_ASRC BE",
		.no_pcm = 1,
		.id = DAI_LINK_TDM_ASRC,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(tdm_asrc),
	},
	[DAI_LINK_HW_GAIN1] = {
		.name = "HW_GAIN1 BE",
		.no_pcm = 1,
		.id = DAI_LINK_HW_GAIN1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(hw_gain1),
	},
};

static int mt8365_p1_gpio_probe(struct snd_soc_card *card)
{
	struct mt8365_p1_priv *priv = snd_soc_card_get_drvdata(card);
	struct device_node *np = card->dev->of_node;
	int ret = 0;
	int i;

	priv->pinctrl = devm_pinctrl_get(card->dev);
	if (IS_ERR(priv->pinctrl)) {
		ret = PTR_ERR(priv->pinctrl);
		dev_err(card->dev, "%s devm_pinctrl_get failed %d\n",
			__func__, ret);
		return ret;
	}

	for (i = 0 ; i < PIN_STATE_MAX ; i++) {
		priv->pin_states[i] = pinctrl_lookup_state(priv->pinctrl,
			mt8365_p1_pin_str[i]);
		if (IS_ERR(priv->pin_states[i])) {
			ret = PTR_ERR(priv->pin_states[i]);
			dev_info(card->dev, "%s Can't find pin state %s %d\n",
				 __func__, mt8365_p1_pin_str[i], ret);
		}
	}

	priv->ext_amp1_gpio = of_get_named_gpio(np, "ext-amp1-gpio", 0);
	if (!gpio_is_valid(priv->ext_amp1_gpio))
		dev_warn(card->dev, "%s get invalid ext-amp1-gpio %d\n",
				__func__, priv->ext_amp1_gpio);

	priv->ext_amp2_gpio = of_get_named_gpio(np, "ext-amp2-gpio", 0);
	if (!gpio_is_valid(priv->ext_amp2_gpio))
		dev_warn(card->dev, "%s get invalid ext-amp2-gpio %d\n",
				__func__, priv->ext_amp2_gpio);

	if (gpio_is_valid(priv->ext_amp1_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->ext_amp1_gpio,
			GPIOF_OUT_INIT_LOW, "ext amp1 enable");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init ext amp1 enable gpio %d\n",
				__func__, ret);
	}

	if (gpio_is_valid(priv->ext_amp2_gpio)) {
		ret = devm_gpio_request_one(card->dev, priv->ext_amp2_gpio,
			GPIOF_OUT_INIT_LOW, "ext amp2 enable");
		if (ret < 0)
			dev_err(card->dev, "%s failed to init ext amp2 enable gpio %d\n",
				__func__, ret);
	}

	for (i = PIN_STATE_DEFAULT ; i < PIN_STATE_MAX ; i++) {
		if (IS_ERR(priv->pin_states[i])) {
			dev_info(card->dev, "%s check pin %s state err\n",
				 __func__, mt8365_p1_pin_str[i]);
			continue;
		}

		/* default state */
		ret = pinctrl_select_state(priv->pinctrl,
				priv->pin_states[i]);
		if (ret)
			dev_info(card->dev, "%s failed to select state %d\n",
				__func__, ret);
	}

	/* turn off mosi pin if exist */
	if (!IS_ERR(priv->pin_states[PIN_STATE_MOSI_OFF])) {
		ret = pinctrl_select_state(priv->pinctrl,
				priv->pin_states[PIN_STATE_MOSI_OFF]);
		if (ret)
			dev_info(card->dev,
				"%s failed to select state %d\n",
				__func__, ret);
	}

	/* turn off miso pin if exist */
	if (!IS_ERR(priv->pin_states[PIN_STATE_MISO_OFF])) {
		ret = pinctrl_select_state(priv->pinctrl,
				priv->pin_states[PIN_STATE_MISO_OFF]);
		if (ret)
			dev_info(card->dev,
				"%s failed to select state %d\n",
				__func__, ret);
	}

	return ret;
}

static int link_to_dai(int link_id)
{
	switch (link_id) {
	case DAI_LINK_TDM_IN_IO:
		return MT8365_AFE_IO_TDM_IN;
	case DAI_LINK_DMIC:
		return MT8365_AFE_IO_DMIC;
	default:
		break;
	}
	return -1;
}

static void mt8365_p1_parse_of(struct snd_soc_card *card,
	struct device_node *np)
{
	struct mt8365_p1_priv *priv = snd_soc_card_get_drvdata(card);
	size_t i;
	int ret;
	char prop[128];
	unsigned int val;

	static const struct mt8365_dai_link_prop of_dai_links_io[] = {
		{ "dmic",	DAI_LINK_DMIC },
		{ "tdmin",	DAI_LINK_TDM_IN_IO },
	};

	for (i = 0; i < ARRAY_SIZE(of_dai_links_io); i++) {
		unsigned int link_id = of_dai_links_io[i].link_id;
		struct snd_soc_dai_link *dai_link = &mt8365_p1_dais[link_id];
		struct mt8365_p1_dmic_ctrl_data *dmic;
		struct mt8365_p1_tdm_ctrl_data *tdm;

		/* parse fix rate */
		snprintf(prop, sizeof(prop), PREFIX"%s-fix-rate",
			 of_dai_links_io[i].name);
		ret = of_property_read_u32(np, prop, &val);

		if (ret == 0 && mt8365_afe_rate_supported(val,
			link_to_dai(link_id))) {
			switch (link_id) {
			case DAI_LINK_DMIC:
				dmic = &priv->dmic_data;
				dmic->fix_rate = val;
				break;
			case DAI_LINK_TDM_IN_IO:
				tdm = &priv->tdm_data;
				tdm->tdmin_fix_rate = val;
				break;
			default:
				break;
			}

			dai_link->be_hw_params_fixup =
				mt8365_be_hw_params_fixup;
		}

		/* parse fix bit width */
		snprintf(prop, sizeof(prop), PREFIX"%s-fix-bit-width",
			 of_dai_links_io[i].name);
		ret = of_property_read_u32(np, prop, &val);
		if (ret == 0 && (val == 32 || val == 16)) {
			switch (link_id) {
			case DAI_LINK_DMIC:
				dmic = &priv->dmic_data;
				dmic->fix_bit_width = val;
				break;
			case DAI_LINK_TDM_IN_IO:
				tdm = &priv->tdm_data;
				tdm->tdmin_fix_bit_width = val;
				break;
			default:
				break;
			}

			dai_link->be_hw_params_fixup =
				mt8365_be_hw_params_fixup;
		}

		/* parse fix channels */
		snprintf(prop, sizeof(prop), PREFIX"%s-fix-channels",
			 of_dai_links_io[i].name);
		ret = of_property_read_u32(np, prop, &val);

		if (ret == 0 && mt8365_afe_channel_supported(val,
			link_to_dai(link_id))) {
			switch (link_id) {
			case DAI_LINK_DMIC:
				dmic = &priv->dmic_data;
				dmic->fix_channels = val;
				break;
			case DAI_LINK_TDM_IN_IO:
				tdm = &priv->tdm_data;
				tdm->tdmin_fix_channels = val;
				break;
			default:
				break;
			}

			dai_link->be_hw_params_fixup =
				mt8365_be_hw_params_fixup;
		}
	}

	of_property_read_u32(np,
		"mediatek,ext-spk-amp-warmup-time-us",
		&priv->ext_spk_amp_warmup_time_us);

	of_property_read_u32(np,
		"mediatek,ext-spk-amp-shutdown-time-us",
		&priv->ext_spk_amp_shutdown_time_us);
}

static struct snd_soc_card mt8365_p1_card = {
	.name = "mt-snd-card",
	.owner = THIS_MODULE,
	.dai_link = mt8365_p1_dais,
	.num_links = ARRAY_SIZE(mt8365_p1_dais),
	.dapm_widgets = mt8365_p1_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt8365_p1_widgets),
	.dapm_routes = mt8365_p1_routes,
	.num_dapm_routes = ARRAY_SIZE(mt8365_p1_routes),
};

static int mt8365_p1_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8365_p1_card;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *platform_node;
	struct mt8365_p1_priv *priv;
	int i, ret;

	platform_node = of_parse_phandle(dev->of_node, "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}

	for (i = 0; i < card->num_links; i++) {
		if (mt8365_p1_dais[i].platforms->name)
			continue;
		mt8365_p1_dais[i].platforms->of_node = platform_node;
	}


	card->dev = dev;

	priv = devm_kzalloc(dev, sizeof(struct mt8365_p1_priv),
			    GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		dev_err(dev, "%s allocate card private data fail %d\n",
			__func__, ret);
		return ret;
	}

	snd_soc_card_set_drvdata(card, priv);

	mt8365_p1_gpio_probe(card);

	mt8365_p1_parse_of(card, np);

	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		dev_err(dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);

	return ret;
}

static const struct of_device_id mt8365_p1_dt_match[] = {
	{ .compatible = "mediatek,mt8365-p1", },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8365_p1_dt_match);

static struct platform_driver mt8365_p1_driver = {
	.driver = {
		   .name = "mt8365-p1",
		   .of_match_table = mt8365_p1_dt_match,
#ifdef CONFIG_PM
		   .pm = &snd_soc_pm_ops,
#endif
	},
	.probe = mt8365_p1_dev_probe,
};

module_platform_driver(mt8365_p1_driver);

/* Module information */
MODULE_DESCRIPTION("MT8365 P1 SoC machine driver");
MODULE_AUTHOR("Jia Zeng <jia.zeng@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mt8365-p1");
