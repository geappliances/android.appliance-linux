
obj-y += pvrsrvkm.o

ccflags-y += \
	-include $(srctree)/$(src)/include/config_kernel.h \
	-I$(srctree)/$(src)/kernel/drivers/staging/imgtec \
	-I$(srctree)/$(src)/include \
	-I$(srctree)/$(src)/include/public \
	-I$(srctree)/$(src)/include/drm \
	-I$(srctree)/$(src)/include/public/powervr \
	-I$(srctree)/$(src)/include/rogue \
	-I$(srctree)/$(src)/services/server/env/linux \
	-I$(srctree)/$(src)/services/server/include \
	-I$(srctree)/$(src)/services/server/devices \
	-I$(srctree)/$(src)/services/server/devices/rogue \
	-I$(srctree)/$(src)/services/server/common \
	-I$(srctree)/$(src)/services/shared/include \
	-I$(srctree)/$(src)/services/shared/devices/rogue \
	-I$(srctree)/$(src)/services/shared/common \
	-I$(srctree)/$(src)/services/system/rogue/include \
	-I$(srctree)/$(src)/services/include \
	-I$(srctree)/$(src)/services/include/rogue \
	-I$(srctree)/$(src)/hwdefs/rogue/km \
	-I$(srctree)/$(src)/hwdefs/rogue \
	-I$(srctree)/$(src)/generated/rogue/htbuffer_bridge \
	-I$(srctree)/$(src)/generated/rogue/cmm_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxtq2_bridge \
	-I$(srctree)/$(src)/generated/rogue/pdumpctrl_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxtq_bridge \
	-I$(srctree)/$(src)/generated/rogue/mm_bridge \
	-I$(srctree)/$(src)/generated/rogue/pvrtl_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxfwdbg_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxbreakpoint_bridge \
	-I$(srctree)/$(src)/generated/rogue/pdumpmm_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxregconfig_bridge \
	-I$(srctree)/$(src)/generated/rogue/pdump_bridge \
	-I$(srctree)/$(src)/generated/rogue/sync_bridge \
	-I$(srctree)/$(src)/generated/rogue/cache_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxhwperf_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxsignals_bridge \
	-I$(srctree)/$(src)/generated/rogue/ri_bridge \
	-I$(srctree)/$(src)/generated/rogue/dmabuf_bridge \
	-I$(srctree)/$(src)/generated/rogue/devicememhistory_bridge \
	-I$(srctree)/$(src)/generated/rogue/synctracking_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxpdump_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxkicksync_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxcmp_bridge \
	-I$(srctree)/$(src)/generated/rogue/srvcore_bridge \
	-I$(srctree)/$(src)/generated/rogue/rgxta3d_bridge \
	-I$(srctree)/$(src)/services/system/rogue/rgx_mtk \
	-I$(srctree)/$(src)/services/system/rogue/rgx_mtk/mt8167

ccflags-y += \
	-Wno-missing-field-initializers \
	-Wdeclaration-after-statement \
	-Wno-format-zero-length \
	-Wmissing-prototypes \
	-Wstrict-prototypes \
	-Wno-unused-parameter \
	-Wno-sign-compare \
	-Wno-type-limits \
	-Wno-error

pvrsrvkm-y += \
	kernel/drivers/staging/imgtec/pvr_drm.o \
	kernel/drivers/staging/imgtec/pvr_sync_file.o \
	kernel/drivers/staging/imgtec/ion_fbcdc_clear.o \
	kernel/drivers/staging/imgtec/pvr_platform_drv.o \
	kernel/drivers/staging/imgtec/pvr_mtk_drv.o \
	kernel/drivers/staging/imgtec/pvr_sw_fence.o \
	kernel/drivers/staging/imgtec/pvr_fence.o \
	kernel/drivers/staging/imgtec/pvr_counting_timeline.o \
	services/server/env/linux/physmem_dmabuf.o \
	services/server/env/linux/osmmap_stub.o \
	services/server/env/linux/osconnection_server.o \
	services/server/env/linux/pvr_gputrace.o \
	services/server/env/linux/physmem_osmem_linux.o \
	services/server/env/linux/allocmem.o \
	services/server/env/linux/pvr_procfs.o \
	services/server/env/linux/pvr_bridge_k.o \
	services/server/env/linux/physmem_test.o \
	services/server/env/linux/fwload.o \
	services/server/env/linux/pmr_os.o \
	services/server/env/linux/pvr_debugfs.o \
	services/server/env/linux/event.o \
	services/server/env/linux/km_apphint.o \
	services/server/env/linux/pvr_debug.o \
	services/server/env/linux/module_common.o \
	services/server/env/linux/handle_idr.o \
	services/server/env/linux/osfunc.o \
	services/server/devices/rogue/rgxstartstop.o \
	services/server/devices/rogue/rgxmem.o \
	services/server/devices/rogue/rgxbvnc.o \
	services/server/devices/rogue/rgxta3d.o \
	services/server/devices/rogue/rgxmipsmmuinit.o \
	services/server/devices/rogue/rgxmulticore.o \
	services/server/devices/rogue/rgxlayer_impl.o \
	services/server/devices/rogue/rgxdebug.o \
	services/server/devices/rogue/rgxpdump.o \
	services/server/devices/rogue/rgxkicksync.o \
	services/server/devices/rogue/rgxtransfer.o \
	services/server/devices/rogue/rgxccb.o \
	services/server/devices/rogue/rgxsrvinit.o \
	services/server/devices/rogue/rgxhwperf.o \
	services/server/devices/rogue/rgxinit.o \
	services/server/devices/rogue/rgxutils.o \
	services/server/devices/rogue/rgxfwimageutils.o \
	services/server/devices/rogue/rgxcompute.o \
	services/server/devices/rogue/rgxpower.o \
	services/server/devices/rogue/rgxsyncutils.o \
	services/server/devices/rogue/rgxmmuinit.o \
	services/server/devices/rogue/rgxregconfig.o \
	services/server/devices/rogue/rgxbreakpoint.o \
	services/server/devices/rogue/rgxfwutils.o \
	services/server/devices/rogue/rgxtdmtransfer.o \
	services/server/devices/rogue/rgxsignals.o \
	services/server/devices/rgxfwdbg.o \
	services/server/devices/rgxshader.o \
	services/server/devices/rgxfwtrace_strings.o \
	services/server/devices/rgxtimecorr.o \
	services/server/common/lists.o \
	services/server/common/tlintern.o \
	services/server/common/power.o \
	services/server/common/physheap.o \
	services/server/common/pdump_server.o \
	services/server/common/pdump_physmem.o \
	services/server/common/pvrsrv.o \
	services/server/common/mmu_common.o \
	services/server/common/physmem_lma.o \
	services/server/common/pmr.o \
	services/server/common/cache_km.o \
	services/server/common/connection_server.o \
	services/server/common/vz_vmm_vm.o \
	services/server/common/process_stats.o \
	services/server/common/devicemem_history_server.o \
	services/server/common/physmem_hostmem.o \
	services/server/common/srvcore.o \
	services/server/common/tlserver.o \
	services/server/common/htb_debug.o \
	services/server/common/handle.o \
	services/server/common/physmem_tdfwmem.o \
	services/server/common/htbserver.o \
	services/server/common/devicemem_server.o \
	services/server/common/pvrsrv_pool.o \
	services/server/common/vmm_pvz_server.o \
	services/server/common/vmm_pvz_client.o \
	services/server/common/info_page_km.o \
	services/server/common/sync_checkpoint.o \
	services/server/common/pdump_mmu.o \
	services/server/common/pvr_notifier.o \
	services/server/common/physmem.o \
	services/server/common/sync_server.o \
	services/server/common/tlstream.o \
	services/server/common/pvrsrv_bridge_init.o \
	services/server/common/di_server.o \
	services/server/common/devicemem_heapcfg.o \
	services/server/common/vz_vmm_pvz.o \
	services/shared/devices/rogue/rgx_hwperf_table.o \
	services/shared/common/devicemem_utils.o \
	services/shared/common/ra.o \
	services/shared/common/sync.o \
	services/shared/common/htbuffer.o \
	services/shared/common/tlclient.o \
	services/shared/common/mem_utils.o \
	services/shared/common/devicememx_pdump.o \
	services/shared/common/devicemem.o \
	services/shared/common/pvrsrv_error.o \
	services/shared/common/hash.o \
	services/shared/common/uniq_key_splay_tree.o \
	services/shared/common/devicemem_pdump.o \
	services/system/rogue/common/env/linux/ion_support_generic.o \
	services/system/rogue/common/env/linux/interrupt_support.o \
	services/system/rogue/common/env/linux/dma_support.o \
	services/system/rogue/common/vmm_type_stub.o \
	generated/rogue/htbuffer_bridge/client_htbuffer_direct_bridge.o \
	generated/rogue/htbuffer_bridge/server_htbuffer_bridge.o \
	generated/rogue/cmm_bridge/server_cmm_bridge.o \
	generated/rogue/rgxtq2_bridge/server_rgxtq2_bridge.o \
	generated/rogue/pdumpctrl_bridge/server_pdumpctrl_bridge.o \
	generated/rogue/pdumpctrl_bridge/client_pdumpctrl_direct_bridge.o \
	generated/rogue/rgxtq_bridge/server_rgxtq_bridge.o \
	generated/rogue/mm_bridge/server_mm_bridge.o \
	generated/rogue/mm_bridge/client_mm_direct_bridge.o \
	generated/rogue/pvrtl_bridge/client_pvrtl_direct_bridge.o \
	generated/rogue/pvrtl_bridge/server_pvrtl_bridge.o \
	generated/rogue/rgxfwdbg_bridge/server_rgxfwdbg_bridge.o \
	generated/rogue/rgxbreakpoint_bridge/server_rgxbreakpoint_bridge.o \
	generated/rogue/pdumpmm_bridge/client_pdumpmm_direct_bridge.o \
	generated/rogue/pdumpmm_bridge/server_pdumpmm_bridge.o \
	generated/rogue/rgxregconfig_bridge/server_rgxregconfig_bridge.o \
	generated/rogue/sync_bridge/server_sync_bridge.o \
	generated/rogue/sync_bridge/client_sync_direct_bridge.o \
	generated/rogue/cache_bridge/client_cache_direct_bridge.o \
	generated/rogue/cache_bridge/server_cache_bridge.o \
	generated/rogue/rgxhwperf_bridge/server_rgxhwperf_bridge.o \
	generated/rogue/rgxsignals_bridge/server_rgxsignals_bridge.o \
	generated/rogue/dmabuf_bridge/server_dmabuf_bridge.o \
	generated/rogue/devicememhistory_bridge/client_devicememhistory_direct_bridge.o \
	generated/rogue/devicememhistory_bridge/server_devicememhistory_bridge.o \
	generated/rogue/synctracking_bridge/client_synctracking_direct_bridge.o \
	generated/rogue/synctracking_bridge/server_synctracking_bridge.o \
	generated/rogue/rgxkicksync_bridge/server_rgxkicksync_bridge.o \
	generated/rogue/rgxcmp_bridge/server_rgxcmp_bridge.o \
	generated/rogue/srvcore_bridge/server_srvcore_bridge.o \
	generated/rogue/rgxta3d_bridge/server_rgxta3d_bridge.o

pvrsrvkm-y += \
	services/system/rogue/rgx_mtk/mtk_pp.o \
	services/system/rogue/rgx_mtk/mt8167/mtk_mfgsys.o \
	services/system/rogue/rgx_mtk/sysconfig.o

pvrsrvkm-$(CONFIG_DRM_POWERVR_ROGUE_DEBUG) += \
	generated/rogue/ri_bridge/client_ri_direct_bridge.o \
	generated/rogue/ri_bridge/server_ri_bridge.o \
	services/server/common/ri_server.o
pvrsrvkm-$(CONFIG_ARM)   += services/server/env/linux/osfunc_arm.o
pvrsrvkm-$(CONFIG_ARM64) += services/server/env/linux/osfunc_arm64.o
pvrsrvkm-$(CONFIG_EVENT_TRACING) += services/server/env/linux/trace_events.o
pvrsrvkm-$(CONFIG_X86)   += services/server/env/linux/osfunc_x86.o
