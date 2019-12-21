include drivers/misc/mediatek/gpu/gpu_rgx/m1.11_5516664/pvrsrvkm.mk

obj-y += pvrsrvkm.o

ccflags-y += \
 -include config_kernel.h \
 -I $(srctree)/include/drm \
 -I $(srctree)/$(src) \
 -I $(srctree)/$(src)/km \
 -I $(srctree)/$(src)/system \
 -D__linux__
