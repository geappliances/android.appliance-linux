// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 BayLibre, SAS
 * Copyright (c) 2019 MediaTek, Inc
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "mt8516-afe-common.h"
#include "mt8516-afe-regs.h"

enum {
	MTK_AFE_ADDA_DL_RATE_8K = 0,
	MTK_AFE_ADDA_DL_RATE_11K = 1,
	MTK_AFE_ADDA_DL_RATE_12K = 2,
	MTK_AFE_ADDA_DL_RATE_16K = 3,
	MTK_AFE_ADDA_DL_RATE_22K = 4,
	MTK_AFE_ADDA_DL_RATE_24K = 5,
	MTK_AFE_ADDA_DL_RATE_32K = 6,
	MTK_AFE_ADDA_DL_RATE_44K = 7,
	MTK_AFE_ADDA_DL_RATE_48K = 8,
};

enum {
	MTK_AFE_ADDA_UL_RATE_8K = 0,
	MTK_AFE_ADDA_UL_RATE_16K = 1,
	MTK_AFE_ADDA_UL_RATE_32K = 2,
	MTK_AFE_ADDA_UL_RATE_48K = 3,
};

static int mt8516_afe_setup_i2s(struct mtk_base_afe *afe,
				    struct snd_pcm_substream *substream,
				    unsigned int rate, int bit_width)
{
	int fs = afe->memif_fs(substream, rate);
	unsigned int val;

	if (bit_width > 16)
		val |= AFE_I2S_CON1_WLEN_32BIT;

	if (fs < 0)
		return -EINVAL;

	val = AFE_I2S_CON1_I2S2_TO_PAD |
	      AFE_I2S_CON1_LOW_JITTER_CLK |
	      AFE_I2S_CON1_RATE(fs) |
	      AFE_I2S_CON1_FORMAT_I2S |
	      AFE_I2S_CON1_EN;

	regmap_write(afe->regmap, AFE_I2S_CON1, val);

	return 0;
}

static int mt8516_afe_setup_adda_dl(struct mtk_base_afe *afe, unsigned int rate)
{
	unsigned int val = AFE_ADDA_DL_8X_UPSAMPLE |
			   AFE_ADDA_DL_MUTE_OFF |
			   AFE_ADDA_DL_DEGRADE_GAIN;

	if (rate == 8000 || rate == 16000)
		val |= AFE_ADDA_DL_VOICE_DATA;

	switch (rate) {
	case 8000:
		val |= MTK_AFE_ADDA_DL_RATE_8K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	case 11025:
		val |= MTK_AFE_ADDA_DL_RATE_11K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	case 12000:
		val |= MTK_AFE_ADDA_DL_RATE_12K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	case 16000:
		val |= MTK_AFE_ADDA_DL_RATE_16K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	case 22050:
		val |= MTK_AFE_ADDA_DL_RATE_22K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	case 24000:
		val |= MTK_AFE_ADDA_DL_RATE_24K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	case 32000:
		val |= MTK_AFE_ADDA_DL_RATE_32K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	case 44100:
		val |= MTK_AFE_ADDA_DL_RATE_44K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	case 48000:
		val |= MTK_AFE_ADDA_DL_RATE_48K << AFE_ADDA_DL_RATE_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	regmap_write(afe->regmap, AFE_ADDA_PREDIS_CON0, 0);
	regmap_write(afe->regmap, AFE_ADDA_PREDIS_CON1, 0);
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON0, val);
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON1, 0xf74f0000);

	return 0;
}

static int mt8516_afe_setup_adda_ul(struct mtk_base_afe *afe, unsigned int rate)
{
	unsigned int val = 0;
	unsigned int val2 = 0;

	switch (rate) {
	case 8000:
		val |= MTK_AFE_ADDA_UL_RATE_8K << AFE_ADDA_UL_RATE_CH1_SHIFT;
		val |= MTK_AFE_ADDA_UL_RATE_8K << AFE_ADDA_UL_RATE_CH2_SHIFT;
		val2 |= 1 << AFE_ADDA_NEWIF_ADC_VOICE_MODE_SHIFT;
		break;
	case 16000:
		val |= MTK_AFE_ADDA_UL_RATE_16K << AFE_ADDA_UL_RATE_CH1_SHIFT;
		val |= MTK_AFE_ADDA_UL_RATE_16K << AFE_ADDA_UL_RATE_CH2_SHIFT;
		val2 |= 1 << AFE_ADDA_NEWIF_ADC_VOICE_MODE_SHIFT;
		break;
	case 32000:
		val |= MTK_AFE_ADDA_UL_RATE_32K << AFE_ADDA_UL_RATE_CH1_SHIFT;
		val |= MTK_AFE_ADDA_UL_RATE_32K << AFE_ADDA_UL_RATE_CH2_SHIFT;
		val2 |= 1 << AFE_ADDA_NEWIF_ADC_VOICE_MODE_SHIFT;
		break;
	case 48000:
		val |= MTK_AFE_ADDA_UL_RATE_48K << AFE_ADDA_UL_RATE_CH1_SHIFT;
		val |= MTK_AFE_ADDA_UL_RATE_48K << AFE_ADDA_UL_RATE_CH2_SHIFT;
		val2 |= 3 << AFE_ADDA_NEWIF_ADC_VOICE_MODE_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(afe->regmap, AFE_ADDA_UL_SRC_CON0,
		(AFE_ADDA_UL_RATE_CH1_MASK << AFE_ADDA_UL_RATE_CH1_SHIFT) ||
		(AFE_ADDA_UL_RATE_CH2_MASK << AFE_ADDA_UL_RATE_CH2_MASK), val);
	regmap_update_bits(afe->regmap, AFE_ADDA_NEWIF_CFG1,
		AFE_ADDA_NEWIF_ADC_VOICE_MODE_CLR, val2);
	regmap_update_bits(afe->regmap, AFE_ADDA_TOP_CON0, 1, 0);

	return 0;
}

static void mt8516_afe_adda_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	unsigned int stream = substream->stream;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(afe->regmap, AFE_ADDA_DL_SRC2_CON0, 1, 0);
		regmap_update_bits(afe->regmap, AFE_I2S_CON1, 1, 0);
	} else {
		regmap_update_bits(afe->regmap, AFE_ADDA_UL_SRC_CON0, 1, 0);
	}

	regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0, 1, 0);
}

static int mt8516_afe_adda_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	unsigned int width_val = 0;

	if (params_width(params) > 16)
		width_val = AFE_CONN_24BIT_O03 | AFE_CONN_24BIT_O04;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		regmap_update_bits(afe->regmap, AFE_CONN_24BIT,
			   AFE_CONN_24BIT_O03 | AFE_CONN_24BIT_O04, width_val);

	return 0;
}

static int mt8516_afe_adda_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	const unsigned int rate = substream->runtime->rate;
	unsigned int stream = substream->stream;
	int bit_width = snd_pcm_format_width(substream->runtime->format);
	int ret;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = mt8516_afe_setup_adda_dl(afe, rate);
		if (ret)
			return ret;

		ret = mt8516_afe_setup_i2s(afe, substream, rate, bit_width);
		if (ret)
			return ret;

		regmap_update_bits(afe->regmap, AFE_ADDA_DL_SRC2_CON0, 1, 1);
	} else {
		ret = mt8516_afe_setup_adda_ul(afe, rate);
		if (ret)
			return ret;

		regmap_update_bits(afe->regmap, AFE_ADDA_UL_SRC_CON0, 1, 1);
	}

	regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0, 1, 1);

	return 0;
}

static const struct snd_soc_dai_ops mt8516_afe_adda_ops = {
	.shutdown	= mt8516_afe_adda_shutdown,
	.hw_params	= mt8516_afe_adda_hw_params,
	.prepare	= mt8516_afe_adda_prepare,
};

static const struct snd_kcontrol_new adda_o03_o04_enable_ctl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);

static const char * const ain_text[] = {
	"INT ADC", "EXT ADC"
};

static SOC_ENUM_SINGLE_DECL(ain_enum, AFE_ADDA_TOP_CON0, 0, ain_text);

static const struct snd_kcontrol_new ain_mux =
	SOC_DAPM_ENUM("AIN Source", ain_enum);

enum {
	SUPPLY_SEQ_ADDA_AFE_ON,
};

static const struct snd_soc_dapm_widget mtk_dai_adda_widgets[] = {
	SND_SOC_DAPM_MUX("AIN Mux", SND_SOC_NOPM, 0, 0, &ain_mux),

	SND_SOC_DAPM_SWITCH("ADDA O03_O04", SND_SOC_NOPM, 0, 0,
			    &adda_o03_o04_enable_ctl),


	SND_SOC_DAPM_SUPPLY_S("ADDA Enable", SUPPLY_SEQ_ADDA_AFE_ON,
			      AFE_DAC_CON0, 0, 0,
			      NULL, 0),

	/* Clocks */
	SND_SOC_DAPM_CLOCK_SUPPLY("top_pdn_audio"),
	SND_SOC_DAPM_CLOCK_SUPPLY("aud_dac_clk"),
	SND_SOC_DAPM_CLOCK_SUPPLY("aud_dac_predis_clk"),
	SND_SOC_DAPM_CLOCK_SUPPLY("aud_adc_clk"),
};

static const struct snd_soc_dapm_route mtk_dai_adda_routes[] = {
	/* playback */
	{"ADDA O03_O04", "Switch", "O03"},
	{"ADDA O03_O04", "Switch", "O04"},
	{"ADDA Playback", NULL, "ADDA O03_O04"},

	/* capture */
	{"AIN Mux", "INT ADC", "ADDA Capture"},

	/* enable */
	{"ADDA Playback", NULL, "ADDA Enable"},
	{"ADDA Capture", NULL, "ADDA Enable"},

	/* clock */
	{"ADDA Playback", NULL, "aud_dac_clk"},
	{"ADDA Playback", NULL, "aud_dac_predis_clk"},
	{"ADDA Playback", NULL, "top_pdn_audio"},

	{"ADDA Capture", NULL, "top_pdn_audio"},
	{"ADDA Capture", NULL, "aud_adc_clk"},
};

static struct snd_soc_dai_driver mtk_dai_adda_driver[] = {
	{
		.name = "ADDA",
		.id = MT8516_AFE_BE_ADDA,
		.playback = {
			.stream_name = "ADDA Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
		},
		.capture = {
			.stream_name = "ADDA Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 |
				 SNDRV_PCM_RATE_16000 |
				 SNDRV_PCM_RATE_32000 |
				 SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &mt8516_afe_adda_ops,
	},
};

int mt8516_dai_adda_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mtk_dai_adda_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_adda_driver);

	dai->dapm_widgets = mtk_dai_adda_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mtk_dai_adda_widgets);
	dai->dapm_routes = mtk_dai_adda_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_adda_routes);

	return 0;
}
