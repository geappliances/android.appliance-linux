MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)

snd-soc-mt8167-pcm-objs := \
    mt8167-afe-pcm.o mt8167-afe-util.o mt8167-afe-controls.o mt8167-afe-debug.o

obj-$(CONFIG_SND_SOC_MT8167) += snd-soc-mt8167-pcm.o
obj-$(CONFIG_SND_SOC_MT8516_PUMPKIN_MACH) += mt8516-pumpkin.o
obj-$(CONFIG_SND_SOC_MT8516_VESPER) += mt8516-vesper.o
