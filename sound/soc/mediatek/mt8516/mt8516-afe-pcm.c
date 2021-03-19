// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 BayLibre, SAS
 * Copyright (c) 2019 MediaTek, Inc
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/module.h>
#include <linux/of.h>

#include "mt8516-afe-common.h"
#include "mt8516-afe-regs.h"

#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-afe-fe-dai.h"
#include "../common/mtk-base-afe.h"

enum {
	MT8516_AFE_MEMIF_DL1,
	MT8516_AFE_MEMIF_DL2,
	MT8516_AFE_MEMIF_VUL,
	MT8516_AFE_MEMIF_DAI,
	MT8516_AFE_MEMIF_AWB,
	MT8516_AFE_MEMIF_MOD_DAI,
	MT8516_AFE_MEMIF_HDMI,
	MT8516_AFE_MEMIF_TDM_IN,
	MT8516_AFE_MEMIF_MULTILINE_IN,
	MT8516_AFE_MEMIF_NUM,
};

enum {
	MT8516_AFE_IRQ_1 = 0,
	MT8516_AFE_IRQ_2,
	MT8516_AFE_IRQ_5, /* dedicated for HDMI */
	MT8516_AFE_IRQ_7,
	MT8516_AFE_IRQ_10, /* dedicated for TDM IN */
	MT8516_AFE_IRQ_13, /* dedicated for ULM*/
	MT8516_AFE_IRQ_NUM
};

struct mt8516_afe_rate {
	unsigned int rate;
	unsigned int regvalue;
};

static const struct mt8516_afe_rate mt8516_afe_i2s_rates[] = {
	{ .rate = 8000, .regvalue = 0 },
	{ .rate = 11025, .regvalue = 1 },
	{ .rate = 12000, .regvalue = 2 },
	{ .rate = 16000, .regvalue = 4 },
	{ .rate = 22050, .regvalue = 5 },
	{ .rate = 24000, .regvalue = 6 },
	{ .rate = 32000, .regvalue = 8 },
	{ .rate = 44100, .regvalue = 9 },
	{ .rate = 48000, .regvalue = 10 },
	{ .rate = 88000, .regvalue = 11 },
	{ .rate = 96000, .regvalue = 12 },
	{ .rate = 176400, .regvalue = 13 },
	{ .rate = 192000, .regvalue = 14 },
};

static int mt8516_afe_i2s_fs(struct snd_pcm_substream *substream,
				unsigned int sample_rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mt8516_afe_i2s_rates); i++)
		if (mt8516_afe_i2s_rates[i].rate == sample_rate)
			return mt8516_afe_i2s_rates[i].regvalue;

	return -EINVAL;
}


static int mt8516_afe_irq_fs(struct snd_pcm_substream *substream,
		      unsigned int rate)
{
	return mt8516_afe_i2s_fs(substream, rate);
}

static const unsigned int mt8516_afe_backup_list[] = {
	AUDIO_TOP_CON0,
	AUDIO_TOP_CON1,
	AUDIO_TOP_CON3,
	AFE_CONN0,
	AFE_CONN1,
	AFE_CONN2,
	AFE_CONN3,
	AFE_CONN5,
	AFE_CONN_24BIT,
	AFE_I2S_CON,
	AFE_I2S_CON1,
	AFE_I2S_CON2,
	AFE_I2S_CON3,
	AFE_ADDA_PREDIS_CON0,
	AFE_ADDA_PREDIS_CON1,
	AFE_ADDA_DL_SRC2_CON0,
	AFE_ADDA_DL_SRC2_CON1,
	AFE_ADDA_UL_SRC_CON0,
	AFE_ADDA_UL_SRC_CON0,
	AFE_ADDA_NEWIF_CFG1,
	AFE_ADDA_TOP_CON0,
	AFE_ADDA_UL_DL_CON0,
	AFE_MEMIF_PBUF_SIZE,
	AFE_MEMIF_PBUF2_SIZE,
	AFE_DAC_CON0,
	AFE_DAC_CON1,
	AFE_DL1_BASE,
	AFE_DL1_END,
	AFE_DL2_BASE,
	AFE_DL2_END,
	AFE_VUL_BASE,
	AFE_VUL_END,
	AFE_AWB_BASE,
	AFE_AWB_END,
	AFE_DAI_BASE,
	AFE_DAI_END,
	AFE_HDMI_OUT_BASE,
	AFE_HDMI_OUT_END,
	AFE_HDMI_IN_2CH_BASE,
	AFE_HDMI_IN_2CH_END,
};

static const struct snd_pcm_hardware mt8516_afe_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_RESUME |
		SNDRV_PCM_INFO_MMAP_VALID,
	.buffer_bytes_max = 1024 * 1024,
	.period_bytes_min = 256,
	.period_bytes_max = 512 * 1024,
	.periods_min = 2,
	.periods_max = 256,
	.fifo_size = 0,
};

static const struct snd_kcontrol_new mt8516_afe_o03_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("I05 Switch", AFE_CONN1, 21, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I07 Switch", AFE_CONN1, 23, 1, 0),
};

static const struct snd_kcontrol_new mt8516_afe_o04_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("I06 Switch", AFE_CONN2, 6, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I08 Switch", AFE_CONN2, 8, 1, 0),
};

static const struct snd_kcontrol_new mt8516_afe_o09_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("I00 Switch", AFE_CONN5, 8, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I03 Switch", AFE_CONN3, 0, 1, 0),
};

static const struct snd_kcontrol_new mt8516_afe_o10_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("I01 Switch", AFE_CONN5, 13, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I04 Switch", AFE_CONN3, 3, 1, 0),
};

static const struct snd_soc_dapm_widget mt8516_memif_widgets[] = {
	/* inter-connections */
	SND_SOC_DAPM_MIXER("I03", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("I04", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("I05", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("I06", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("I07", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("I08", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("O03", SND_SOC_NOPM, 0, 0,
			   mt8516_afe_o03_mix, ARRAY_SIZE(mt8516_afe_o03_mix)),
	SND_SOC_DAPM_MIXER("O04", SND_SOC_NOPM, 0, 0,
			   mt8516_afe_o04_mix, ARRAY_SIZE(mt8516_afe_o04_mix)),
	SND_SOC_DAPM_MIXER("O09", SND_SOC_NOPM, 0, 0,
			   mt8516_afe_o09_mix, ARRAY_SIZE(mt8516_afe_o09_mix)),
	SND_SOC_DAPM_MIXER("O10", SND_SOC_NOPM, 0, 0,
			   mt8516_afe_o10_mix, ARRAY_SIZE(mt8516_afe_o10_mix)),
};

static const struct snd_soc_dapm_route mt8516_memif_routes[] = {
	/* downlink */
	{"I05", NULL, "DL1"},
	{"I06", NULL, "DL1"},
	{"I07", NULL, "DL2"},
	{"I08", NULL, "DL2"},
	{"O03", "I05 Switch", "I05"},
	{"O04", "I06 Switch", "I06"},
	{"O03", "I07 Switch", "I07"},
	{"O04", "I08 Switch", "I08"},

	/* uplink */
	{"I03", NULL, "AIN Mux"},
	{"I04", NULL, "AIN Mux"},

	{"O09", "I03 Switch", "I03"},
	{"O10", "I04 Switch", "I04"},
	{"VUL", NULL, "O09"},
	{"VUL", NULL, "O10"},
};

static struct mtk_base_irq_data mt8516_irq_data[MT8516_AFE_IRQ_NUM] = {
	[MT8516_AFE_IRQ_1] = {
		.id = MT8516_AFE_IRQ_1,
		.irq_cnt_reg = AFE_IRQ_CNT1,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON,
		.irq_fs_shift = 4,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON,
		.irq_en_shift = 0,
		.irq_clr_reg = AFE_IRQ_CLR,
		.irq_clr_shift = 0,
	},
	[MT8516_AFE_IRQ_2] = {
		.id = MT8516_AFE_IRQ_2,
		.irq_cnt_reg = AFE_IRQ_CNT2,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON,
		.irq_fs_shift = 8,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON,
		.irq_en_shift = 1,
		.irq_clr_reg = AFE_IRQ_CLR,
		.irq_clr_shift = 1,
	},
	[MT8516_AFE_IRQ_5] = {
		.id = MT8516_AFE_IRQ_5,
		.irq_cnt_reg = AFE_IRQ_CNT5,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = -1,
		.irq_fs_shift = -1,
		.irq_en_reg = AFE_IRQ_MCU_CON2,
		.irq_en_shift = 3,
		.irq_clr_reg = AFE_IRQ_CLR,
		.irq_clr_shift = 4,
	},
	[MT8516_AFE_IRQ_7] = {
		.id = MT8516_AFE_IRQ_7,
		.irq_cnt_reg = AFE_IRQ_CNT7,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON,
		.irq_fs_shift = 24,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON,
		.irq_en_shift = 14,
		.irq_clr_reg = AFE_IRQ_CLR,
		.irq_clr_shift = 6,
	},
	[MT8516_AFE_IRQ_10] = {
		.id = MT8516_AFE_IRQ_10,
		.irq_cnt_reg = AFE_IRQ_CNT10,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = -1,
		.irq_fs_shift = -1,
		.irq_en_reg = AFE_IRQ_MCU_CON2,
		.irq_en_shift = 4,
		.irq_clr_reg = AFE_IRQ_CLR,
		.irq_clr_shift = 9,
	},
	[MT8516_AFE_IRQ_13] = {
		.id = MT8516_AFE_IRQ_13,
		.irq_cnt_reg = AFE_IRQ_CNT13,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = -1,
		.irq_fs_shift = -1,
		.irq_en_reg = AFE_IRQ_MCU_CON2,
		.irq_en_shift = 7,
		.irq_clr_reg = AFE_IRQ_CLR,
		.irq_clr_shift = 12,
	},
};

static struct mtk_base_afe_irq mt8516_irqs[MT8516_AFE_IRQ_NUM] = {
	{ .irq_data = &mt8516_irq_data[MT8516_AFE_IRQ_1] },
	{ .irq_data = &mt8516_irq_data[MT8516_AFE_IRQ_2] },
	{ .irq_data = &mt8516_irq_data[MT8516_AFE_IRQ_5] },
	{ .irq_data = &mt8516_irq_data[MT8516_AFE_IRQ_7] },
	{ .irq_data = &mt8516_irq_data[MT8516_AFE_IRQ_10] },
	{ .irq_data = &mt8516_irq_data[MT8516_AFE_IRQ_13] },
};

static struct mtk_base_memif_data mt8516_memif_data[MT8516_AFE_MEMIF_NUM] = {
	[MT8516_AFE_MEMIF_DL1] = {
		.name = "DL1",
		.id = MT8516_AFE_MEMIF_DL1,
		.reg_ofs_base = AFE_DL1_BASE,
		.reg_ofs_cur = AFE_DL1_CUR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = 0,
		.fs_maskbit = 0xf,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = 21,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = 1,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
	[MT8516_AFE_MEMIF_DL2] = {
		.name = "DL2",
		.id = MT8516_AFE_MEMIF_DL2,
		.reg_ofs_base = AFE_DL2_BASE,
		.reg_ofs_cur = AFE_DL2_CUR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = 4,
		.fs_maskbit = 0xf,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = 22,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = 2,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
	[MT8516_AFE_MEMIF_VUL] = {
		.name = "VUL",
		.id = MT8516_AFE_MEMIF_VUL,
		.reg_ofs_base = AFE_VUL_BASE,
		.reg_ofs_cur = AFE_VUL_CUR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = 16,
		.fs_maskbit = 0xf,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = 27,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = 3,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
	[MT8516_AFE_MEMIF_DAI] = {
		.name = "DAI",
		.id = MT8516_AFE_MEMIF_DAI,
		.reg_ofs_base = AFE_DAI_BASE,
		.reg_ofs_cur = AFE_DAI_CUR,
		.fs_reg = AFE_DAC_CON0,
		.fs_shift = 24,
		.fs_maskbit = 0x3,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = -1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = 4,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
	[MT8516_AFE_MEMIF_AWB] = {
		.name = "AWB",
		.id = MT8516_AFE_MEMIF_AWB,
		.reg_ofs_base = AFE_AWB_BASE,
		.reg_ofs_cur = AFE_AWB_CUR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = 12,
		.fs_maskbit = 0xf,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = 24,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = 6,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
	[MT8516_AFE_MEMIF_MOD_DAI] = {
		.name = "MOD_DAI",
		.id = MT8516_AFE_MEMIF_MOD_DAI,
		.reg_ofs_base = AFE_MOD_PCM_BASE,
		.reg_ofs_cur = AFE_MOD_PCM_CUR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = 30,
		.fs_maskbit = 0x3,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = -1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = 7,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
	[MT8516_AFE_MEMIF_HDMI] = {
		.name = "HDMI",
		.id = MT8516_AFE_MEMIF_HDMI,
		.reg_ofs_base = AFE_HDMI_OUT_BASE,
		.reg_ofs_cur = AFE_HDMI_OUT_CUR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = -1,
		.fs_maskbit = 0xf,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = -1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = -1,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
	[MT8516_AFE_MEMIF_TDM_IN] = {
		.name = "TDM_IN",
		.id = MT8516_AFE_MEMIF_TDM_IN,
		.reg_ofs_base = AFE_HDMI_IN_2CH_BASE,
		.reg_ofs_cur = AFE_HDMI_IN_2CH_CUR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = -1,
		.fs_maskbit = 0xf,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = -1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = -1,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
	[MT8516_AFE_MEMIF_MULTILINE_IN] = {
		.name = "ULM",
		.id = MT8516_AFE_MEMIF_MULTILINE_IN,
		.reg_ofs_base = SPDIFIN_BASE_ADR,
		.reg_ofs_cur = SPDIFIN_CUR_ADR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = -1,
		.fs_maskbit = 0xf,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = -1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = -1,
		.hd_shift = -1,
		.msb_shift = -1,
		.agent_disable_shift = -1,
	},
};

struct mtk_base_afe_memif mt8516_memif[] = {
	[MT8516_AFE_MEMIF_DL1] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_DL1],
		.irq_usage = MT8516_AFE_IRQ_1,
		.const_irq = 1,
	},
	[MT8516_AFE_MEMIF_DL2] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_DL2],
		.irq_usage = MT8516_AFE_IRQ_7,
		.const_irq = 1,
	},
	[MT8516_AFE_MEMIF_VUL] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_VUL],
		.irq_usage = MT8516_AFE_IRQ_2,
		.const_irq = 1,
	},
	[MT8516_AFE_MEMIF_DAI] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_DAI],
		.irq_usage = MT8516_AFE_IRQ_2,
		.const_irq = 1,
	},
	[MT8516_AFE_MEMIF_AWB] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_AWB],
		.irq_usage = MT8516_AFE_IRQ_2,
		.const_irq = 1,
	},
	[MT8516_AFE_MEMIF_MOD_DAI] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_MOD_DAI],
		.irq_usage = MT8516_AFE_IRQ_2,
		.const_irq = 1,
	},
	[MT8516_AFE_MEMIF_HDMI] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_HDMI],
		.irq_usage = MT8516_AFE_IRQ_5,
		.const_irq = 1,
	},
	[MT8516_AFE_MEMIF_TDM_IN] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_TDM_IN],
		.irq_usage = MT8516_AFE_IRQ_10,
		.const_irq = 1,
	},
	[MT8516_AFE_MEMIF_MULTILINE_IN] = {
		.data = &mt8516_memif_data[MT8516_AFE_MEMIF_MULTILINE_IN],
		.data = &mt8516_memif_data[8],
		.irq_usage = MT8516_AFE_IRQ_13,
		.const_irq = 1,
	},
};

static const struct regmap_config mt8516_afe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = ABB_AFE_SDM_TEST,
	.cache_type = REGCACHE_NONE,
};

static irqreturn_t mt8516_afe_irq_handler(int irq, void *dev_id)
{
	struct mtk_base_afe *afe = dev_id;
	unsigned int reg_value;
	unsigned int memif_status;
	int i, ret;

	ret = regmap_read(afe->regmap, AFE_IRQ_STATUS, &reg_value);
	if (ret)
		goto exit_irq;

	ret = regmap_read(afe->regmap, AFE_DAC_CON0, &memif_status);
	if (ret)
		goto exit_irq;

	for (i = 0; i < MT8516_AFE_MEMIF_NUM; i++) {
		struct mtk_base_afe_memif *memif = &afe->memif[i];
		struct snd_pcm_substream *substream = memif->substream;
		unsigned int irq_clr_shift =
			afe->irqs[memif->irq_usage].irq_data->irq_clr_shift;
		unsigned int enable_shift = memif->data->enable_shift;

		if (!substream)
			continue;

		if (!(reg_value & (1 << irq_clr_shift)))
			continue;

		if (enable_shift >= 0 && !((1 << enable_shift) & memif_status))
			continue;

		snd_pcm_period_elapsed(substream);
	}

	regmap_write(afe->regmap, AFE_IRQ_CLR, reg_value & AFE_IRQ_STATUS_BITS);

	return IRQ_HANDLED;

exit_irq:
	return IRQ_NONE;
}

static struct snd_soc_dai_driver mt8516_memif_dai_driver[] = {
	/* FE DAIs: memory intefaces to CPU */
	{
		.name = "DL1",
		.id = MT8516_AFE_MEMIF_DL1,
		.suspend = mtk_afe_dai_suspend,
		.resume = mtk_afe_dai_resume,
		.playback = {
			.stream_name = "DL1",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "DL2",
		.id = MT8516_AFE_MEMIF_DL2,
		.suspend = mtk_afe_dai_suspend,
		.resume = mtk_afe_dai_resume,
		.playback = {
			.stream_name = "DL2",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "VUL",
		.id = MT8516_AFE_MEMIF_VUL,
		.suspend = mtk_afe_dai_suspend,
		.resume = mtk_afe_dai_resume,
		.capture = {
			.stream_name = "VUL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "DAI",
		.id = MT8516_AFE_MEMIF_DAI,
		.suspend = mtk_afe_dai_suspend,
		.resume = mtk_afe_dai_resume,
		.capture = {
			.stream_name = "DAI",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_8000 |
				 SNDRV_PCM_RATE_16000 |
				 SNDRV_PCM_RATE_32000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "HDMI",
		.id = MT8516_AFE_MEMIF_HDMI,
		.suspend = mtk_afe_dai_suspend,
		.resume = mtk_afe_dai_resume,
		.playback = {
			.stream_name = "HDMI",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "TDM_IN",
		.id = MT8516_AFE_MEMIF_TDM_IN,
		.suspend = mtk_afe_dai_suspend,
		.resume = mtk_afe_dai_resume,
		.capture = {
			.stream_name = "TDM_IN",
			.channels_min = 2,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "ULM",
		.id = MT8516_AFE_MEMIF_MULTILINE_IN,
		.suspend = mtk_afe_dai_suspend,
		.resume = mtk_afe_dai_resume,
		.capture = {
			.stream_name = "MULTILINE_IN",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_32000
				| SNDRV_PCM_RATE_44100
				| SNDRV_PCM_RATE_48000
				| SNDRV_PCM_RATE_88200
				| SNDRV_PCM_RATE_96000
				| SNDRV_PCM_RATE_176400
				| SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	},
};

static int mt8516_dai_memif_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mt8516_memif_dai_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mt8516_memif_dai_driver);

	dai->dapm_widgets = mt8516_memif_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mt8516_memif_widgets);
	dai->dapm_routes = mt8516_memif_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mt8516_memif_routes);

	return 0;
}

typedef int (*dai_register_cb)(struct mtk_base_afe *);
static const dai_register_cb dai_register_cbs[] = {
	mt8516_dai_adda_register,
	mt8516_dai_memif_register,
};

static int mt8516_afe_component_probe(struct snd_soc_component *component)
{
	return mtk_afe_add_sub_dai_control(component);
}

static const struct snd_soc_component_driver mt8516_afe_component = {
	.name = AFE_PCM_NAME,
	.ops = &mtk_afe_pcm_ops,
	.pcm_new = mtk_afe_pcm_new,
	.pcm_free = mtk_afe_pcm_free,
	.probe = mt8516_afe_component_probe,
};

static int mt8516_afe_pcm_dev_probe(struct platform_device *pdev)
{
	int ret, i;
	unsigned int irq_id;
	struct mtk_base_afe *afe;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;

	afe = devm_kzalloc(&pdev->dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;
	platform_set_drvdata(pdev, afe);

	afe->dev = &pdev->dev;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	afe->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(afe->base_addr))
		return PTR_ERR(afe->base_addr);

	afe->regmap = devm_regmap_init_mmio(&pdev->dev, afe->base_addr,
		&mt8516_afe_regmap_config);
	if (IS_ERR(afe->regmap))
		return PTR_ERR(afe->regmap);

	afe->reg_back_up_list = &mt8516_afe_backup_list[0];
	afe->reg_back_up_list_num = ARRAY_SIZE(mt8516_afe_backup_list);

	/* init sub_dais */
	INIT_LIST_HEAD(&afe->sub_dais);

	for (i = 0; i < ARRAY_SIZE(dai_register_cbs); i++) {
		ret = dai_register_cbs[i](afe);
		if (ret) {
			dev_warn(afe->dev,
				 "Failed to register dai register %d, ret %d\n",
				 i, ret);
			return ret;
		}
	}

	/* init dai_driver and component_driver */
	ret = mtk_afe_combine_sub_dai(afe);
	if (ret) {
		dev_warn(afe->dev, "Failed to combine sub-dais, ret %d\n", ret);
		return ret;
	}

	afe->mtk_afe_hardware = &mt8516_afe_hardware;

	afe->irqs = mt8516_irqs;
	afe->irq_fs = mt8516_afe_irq_fs;

	afe->memif = &mt8516_memif[0];
	afe->memif_size = ARRAY_SIZE(mt8516_memif);
	afe->memif_fs = mt8516_afe_i2s_fs;

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &mt8516_afe_component,
					      afe->dai_drivers,
					      afe->num_dai_drivers);
	if (ret)
		return ret;

	irq_id = platform_get_irq(pdev, 0);
	if (!irq_id) {
		dev_err(afe->dev, "np %s no irq\n", np->name);
		return -ENXIO;
	}

	ret = devm_request_irq(afe->dev, irq_id, mt8516_afe_irq_handler,
			       0, "Afe_ISR_Handle", (void *)afe);
	if (ret) {
		dev_err(afe->dev, "could not request_irq\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id mt8516_afe_pcm_dt_match[] = {
	{ .compatible = "mediatek,mt8516-audio", },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8516_afe_pcm_dt_match);

static struct platform_driver mt8516_afe_pcm_driver = {
	.driver = {
		   .name = "mtk-afe-pcm",
		   .of_match_table = mt8516_afe_pcm_dt_match,
	},
	.probe = mt8516_afe_pcm_dev_probe,
};

module_platform_driver(mt8516_afe_pcm_driver);

MODULE_DESCRIPTION("Mediatek ALSA SoC AFE platform driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Fabien Parent <fparent@baylibre.com>");
