/*
 * mt8516_p1.c  --  MT8516P1 ALSA SoC machine driver
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

struct mt8516_pumpkin_priv {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_states[PIN_STATE_MAX];
	struct regulator *tdmadc_1p8_supply;
	struct regulator *tdmadc_3p3_supply;
	struct soc_ctlx_res ctlx_res;
};

static const char * const mt8516_pumpkin_pinctrl_pin_str[PIN_STATE_MAX] = {
	"default",
};

static SOC_ENUM_SINGLE_EXT_DECL(pcm_state_enums, pcm_state_func);

/* ctrl resource manager */
static inline int soc_ctlx_init(struct soc_ctlx_res *ctlx_res, struct snd_soc_card *soc_card)
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
			if (strncmp(control->id.name, nfy_ctl_names[i], sizeof(control->id.name)))
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
	struct mt8516_pumpkin_priv *card_data = snd_soc_card_get_drvdata(card);
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
//	pr_notice("get mixer control(%s) value is:%ld\n", kctl->id.name, ucontrol->value.integer.value[0]);

	return 0;
}

static int soc_ctlx_put(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kctl);
	struct mt8516_pumpkin_priv *card_data = snd_soc_card_get_drvdata(card);
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
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE, &(res_mgr->nfy_ids[nfy_type]));
	} else {
		nfy_type = CTRL_NOTIFY_INVAL;
	}
	if (need_notify_self) {
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE, &(kctl->id));
	}
	mutex_unlock(&res_mgr->res_mutex);
	pr_notice("set mixer control(%s) value is:%ld, notify id:%x, notify self:%d\n",
						kctl->id.name,
						ucontrol->value.integer.value[0],
						nfy_type,
						need_notify_self);

	return 0;
}

static int soc_pcm_state_get(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kctl);
	struct mt8516_pumpkin_priv *card_data = snd_soc_card_get_drvdata(card);
	struct soc_ctlx_res *res_mgr = &card_data->ctlx_res;
	unsigned long flags;

	spin_lock_irqsave(&res_mgr->res_lock, flags);
	ucontrol->value.integer.value[0] = res_mgr->pcm_state;
	spin_unlock_irqrestore(&res_mgr->res_lock, flags);
	pr_notice("get mixer control(%s) value is:%ld\n", kctl->id.name, ucontrol->value.integer.value[0]);

	return 0;
}

static int soc_pcm_state_put(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kctl);
	struct mt8516_pumpkin_priv *card_data = snd_soc_card_get_drvdata(card);
	struct soc_ctlx_res *res_mgr = &card_data->ctlx_res;
	unsigned long flags;

	spin_lock_irqsave(&res_mgr->res_lock, flags);
	if (ucontrol->value.integer.value[0] != res_mgr->pcm_state) {
		res_mgr->pcm_state = ucontrol->value.integer.value[0];
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE, &(res_mgr->nfy_ids[PCM_STATEX_ID]));
	}
	spin_unlock_irqrestore(&res_mgr->res_lock, flags);
	pr_notice("set mixer control(%s) value is:%ld\n",
						kctl->id.name,
						ucontrol->value.integer.value[0]);

	return 0;
}

static const struct snd_kcontrol_new mt8516_pumpkin_soc_controls[] = {
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

static int i2s_8ch_playback_state_set(struct snd_pcm_substream *substream, int state)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct mt8516_pumpkin_priv *card_data = snd_soc_card_get_drvdata(card);
	struct soc_ctlx_res *res_mgr = &card_data->ctlx_res;
	int nfy_type;
	unsigned long flags;

	nfy_type = PCM_STATEX_ID;
	spin_lock_irqsave(&res_mgr->res_lock, flags);
	if (res_mgr->pcm_state != state) {
		res_mgr->pcm_state = state;
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE, &(res_mgr->nfy_ids[nfy_type]));
	} else {
		nfy_type = CTRL_NOTIFY_INVAL;
	}
	spin_unlock_irqrestore(&res_mgr->res_lock, flags);

	return 0;
}

static int i2s_8ch_playback_startup(struct snd_pcm_substream *substream)
{
	i2s_8ch_playback_state_set(substream, MTKFILE_PCM_STATE_OPEN);
	return 0;
}

static void i2s_8ch_playback_shutdown(struct snd_pcm_substream *substream)
{
	i2s_8ch_playback_state_set(substream, MTKFILE_PCM_STATE_CLOSE);
}

static int i2s_8ch_playback_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	i2s_8ch_playback_state_set(substream, MTKFILE_PCM_STATE_HW_PARAMS);
	return 0;
}

static int i2s_8ch_playback_hw_free(struct snd_pcm_substream *substream)
{
	i2s_8ch_playback_state_set(substream, MTKFILE_PCM_STATE_HW_FREE);
	return 0;
}

static int i2s_8ch_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		i2s_8ch_playback_state_set(substream, MTKFILE_PCM_STATE_START);
		break;
	default:
		break;
	}

	return 0;
}

static struct snd_soc_ops i2s_8ch_playback_ops = {
	.startup = i2s_8ch_playback_startup,
	.shutdown = i2s_8ch_playback_shutdown,
	.hw_params = i2s_8ch_playback_hw_params,
	.hw_free = i2s_8ch_playback_hw_free,
	.trigger = i2s_8ch_playback_trigger,
};

static int pcm186x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	pr_notice("%s\n", __func__);
	return 0;
}

static struct snd_soc_ops pcm186x_machine_ops = {
	.hw_params = pcm186x_hw_params,
};

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback2,
	DAILINK_COMP_ARRAY(COMP_CPU("DL2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s_8ch_playback,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(dmic_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("VUL")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(awb_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("AWB")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(ref_in_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("AWB")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(dai_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE */
SND_SOC_DAILINK_DEFS(hdmi,
	DAILINK_COMP_ARRAY(COMP_CPU("HDMIO")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(second_ext_codec,
	DAILINK_COMP_ARRAY(COMP_CPU("2ND I2S")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(mtk_codec,
	DAILINK_COMP_ARRAY(COMP_CPU("INT ADDA")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(dmic,
	DAILINK_COMP_ARRAY(COMP_CPU("INT ADDA")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hw_gain1,
	DAILINK_COMP_ARRAY(COMP_CPU("HW_GAIN1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm_in,
	DAILINK_COMP_ARRAY(COMP_CPU("TDM_IN_IO")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(dl,
	DAILINK_COMP_ARRAY(COMP_CPU("DL Input")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(mrg_bt,
	DAILINK_COMP_ARRAY(COMP_CPU("MRG BT")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(intdir,
	DAILINK_COMP_ARRAY(COMP_CPU("INTDIR_IO")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link mt8516_pumpkin_dais[] = {
	/* Front End DAI links */
	{
		.name = "I2S 8CH Playback",
		.stream_name = "I2S8CH Playback",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_playback = 1,
		.ops = &i2s_8ch_playback_ops,
		SND_SOC_DAILINK_REG(i2s_8ch_playback),
	},
	{
		.name = "TDM Capture",
		.stream_name = "TDM_Capture",

		.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		.ops = &pcm186x_machine_ops,
		SND_SOC_DAILINK_REG(tdm_capture),
	},
	{
		.name = "DMIC Capture",
		.stream_name = "DMIC_Capture",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dmic_capture),
	},
	{
		.name = "AWB Capture",
		.stream_name = "AWB_Record",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(awb_capture),
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
		.name = "Ref In Capture",
		.stream_name = "DL1_AWB_Record",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ref_in_capture),
	},
	{
		.name = "DAI Capture",
		.stream_name = "VOIP_Call_BT_Capture",
		.trigger = {
			SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST
		},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dai_capture),
	},
	{
        	.name = "DL2 Playback",
	        .stream_name = "DL2_Playback",
	        .trigger = {
	                SND_SOC_DPCM_TRIGGER_POST,
	                SND_SOC_DPCM_TRIGGER_POST
        	},
	        .dynamic = 1,
	        .dpcm_playback = 1,
		SND_SOC_DAILINK_REG(playback2),
	},

	/* Backend End DAI links */
	{
		.name = "HDMI BE",
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(hdmi),
	},
	{
		.name = "2ND EXT Codec",
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
        	.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(second_ext_codec),
	},
	{
		.name = "MTK Codec",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(mtk_codec),
	},
	{
		.name = "DMIC BE",
		.no_pcm = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dmic),
	},
	{
		.name = "HW Gain1 BE",
		.no_pcm = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(hw_gain1),
	},
	{
		.name = "TDM IN BE",
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(tdm_in),
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
		.name = "DL BE",
		.no_pcm = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dl),
	},
	{
		.name = "MRG BT BE",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(mrg_bt),
	},
	{
		.name = "INTDIR BE",
		.no_pcm = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(intdir),
	},
};

static const struct snd_soc_dapm_widget mt8516_pumpkin_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("External Line In"),
	SND_SOC_DAPM_OUTPUT("External I2S out"),
	SND_SOC_DAPM_INPUT("External Line In2"),
	SND_SOC_DAPM_OUTPUT("External I2S out2"),
};

static const struct snd_soc_dapm_route mt8516_pumpkin_audio_map[] = {
	{"2ND I2S Capture", NULL, "External Line In"},
	{"I2S Capture", NULL, "External Line In2"},
	{"External I2S out", NULL, "I2S Playback"},
	{"External I2S out2", NULL, "2ND I2S Playback"},
};

static int mt8516_pumpkin_suspend_post(struct snd_soc_card *card)
{
	struct mt8516_pumpkin_priv *card_data;

	card_data = snd_soc_card_get_drvdata(card);

	if (!IS_ERR(card_data->tdmadc_1p8_supply))
		regulator_disable(card_data->tdmadc_1p8_supply);
	if (!IS_ERR(card_data->tdmadc_3p3_supply))
		regulator_disable(card_data->tdmadc_3p3_supply);
	return 0;
}

static int mt8516_pumpkin_resume_pre(struct snd_soc_card *card)
{
 	struct mt8516_pumpkin_priv *card_data;
	int ret;

	card_data = snd_soc_card_get_drvdata(card);

	/* tdm adc power down */
	if  (!IS_ERR(card_data->tdmadc_1p8_supply)) {
		ret = regulator_enable(card_data->tdmadc_1p8_supply);
		if (ret != 0)
			dev_err(card->dev, "%s failed to enable tdm 1p8 supply %d!\n", __func__, ret);
	}
	if (!IS_ERR(card_data->tdmadc_3p3_supply)) {
		ret = regulator_enable(card_data->tdmadc_3p3_supply);
		if (ret != 0)
			dev_err(card->dev, "%s failed to enable tdm 3p3 supply %d!\n", __func__, ret);
	}
	return 0;
}
static struct snd_soc_card mt8516_pumpkin_card = {
	.name = "mt-snd-card",
	.owner = THIS_MODULE,
	.dai_link = mt8516_pumpkin_dais,
	.num_links = ARRAY_SIZE(mt8516_pumpkin_dais),
	.controls = mt8516_pumpkin_soc_controls,
	.num_controls = ARRAY_SIZE(mt8516_pumpkin_soc_controls),
	.dapm_widgets = mt8516_pumpkin_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt8516_pumpkin_dapm_widgets),
	.dapm_routes = mt8516_pumpkin_audio_map,
	.num_dapm_routes = ARRAY_SIZE(mt8516_pumpkin_audio_map),
	.suspend_post = mt8516_pumpkin_suspend_post,
	.resume_pre = mt8516_pumpkin_resume_pre,
};

static int mt8516_pumpkin_gpio_probe(struct snd_soc_card *card)
{
	struct mt8516_pumpkin_priv *card_data;
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
				mt8516_pumpkin_pinctrl_pin_str[i]);
		if (IS_ERR(card_data->pin_states[i])) {
			ret = PTR_ERR(card_data->pin_states[i]);
			dev_warn(card->dev, "%s Can't find pinctrl state %s %d\n",
				__func__, mt8516_pumpkin_pinctrl_pin_str[i], ret);
		}
	}
	/* default state */
	if (!IS_ERR(card_data->pin_states[PIN_STATE_DEFAULT])) {
		ret = pinctrl_select_state(card_data->pinctrl,
				card_data->pin_states[PIN_STATE_DEFAULT]);
		if (ret) {
			dev_err(card->dev, "%s failed to select state %d\n",
				__func__, ret);
			goto exit;
		}
	}

exit:

	return ret;
}

static int set_card_codec_info(struct snd_soc_card *card)
{
	struct snd_soc_dai_link_component *dai_link_codecs, *dlc;
	struct device_node *dl_node, *c_node;
	struct device *dev = card->dev;
	struct of_phandle_args args;
	const char *dai_link_name;
	int num_codecs;
	int ret, i;

	/* Loop over all the dai link sub nodes*/
	for_each_child_of_node(dev->of_node, dl_node) {
		if (of_property_read_string(dl_node, "dai-link-name",
					    &dai_link_name))
			return -EINVAL;

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

			dlc++;
		}

		/* Update the snd_soc_dai_link static array with the codecs
		 * we have just found.
		 */
		for (i = 0; i < card->num_links; i++) {
			if (!strcmp(dai_link_name, card->dai_link[i].name)) {
				card->dai_link[i].num_codecs = num_codecs;
				card->dai_link[i].codecs = dai_link_codecs;
				break;
			}
		}
	}

	return 0;
}

static int mt8516_pumpkin_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8516_pumpkin_card;
	struct device_node *platform_node;
	int ret, i;
	struct mt8516_pumpkin_priv *card_data;

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

	for (i = 0; i < card->num_links; i++) {
		if (mt8516_pumpkin_dais[i].platforms->name)
			continue;
		mt8516_pumpkin_dais[i].platforms->of_node = platform_node;
	}

	card_data = devm_kzalloc(&pdev->dev,
		sizeof(struct mt8516_pumpkin_priv), GFP_KERNEL);
	if (!card_data) {
		ret = -ENOMEM;
		dev_err(&pdev->dev,
			"%s allocate card private data fail %d\n",
			__func__, ret);
		return ret;
	}

	snd_soc_card_set_drvdata(card, card_data);

	mt8516_pumpkin_gpio_probe(card);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
		__func__, ret);
		return ret;
	}
	soc_ctlx_init(&card_data->ctlx_res, card);

	return ret;
}

static const struct of_device_id mt8516_pumpkin_dt_match[] = {
	{ .compatible = "mediatek,mt8516-soc-pumpkin", },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8516_pumpkin_dt_match);

static struct platform_driver mt8516_pumpkin_mach_driver = {
	.driver = {
		   .name = "mt8516-soc-pumpkin",
		   .of_match_table = mt8516_pumpkin_dt_match,
#ifdef CONFIG_PM
		   .pm = &snd_soc_pm_ops,
#endif
	},
	.probe = mt8516_pumpkin_dev_probe,
};

module_platform_driver(mt8516_pumpkin_mach_driver);

/* Module information */
MODULE_DESCRIPTION("MT8516Pumpkin ALSA SoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mt8516-pumpkin");

