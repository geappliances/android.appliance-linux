include drivers/misc/mediatek/gpu/gpu_rgx/m1.15_6052913/pvrsrvkm.mk

obj-$(CONFIG_RGX_M1_15_6052913) += pvrsrvkm.o

ccflags-y += \
 -include config_kernel.h \
 -I $(srctree)/include/drm \
 -I $(srctree)/$(src) \
 -I $(srctree)/$(src)/km \
 -I $(srctree)/$(src)/system \
 -D__linux__

