
obj-y += pvrsrvkm.o

ccflags-y += \
	-include $(srctree)/$(src)/include/config_kernel.h \
	-I$(srctree)/$(src)/kernel/drivers/staging/imgtec \
	-I$(srctree)/$(src)/include \
	-I$(srctree)/$(src)/include/drm \
	-I$(srctree)/$(src)/include/public/powervr \
	-I$(srctree)/$(src)/include/public \
	-I$(srctree)/$(src)/services/server/env/linux \
	-I$(srctree)/$(src)/services/server/include \
	-I$(srctree)/$(src)/services/server/env/linux \
	-I$(srctree)/$(src)/services/server/devices/rgx \
	-I$(srctree)/$(src)/services/shared/include \
	-I$(srctree)/$(src)/services/shared/devices/rgx \
	-I$(srctree)/$(src)/services/shared/common \
	-I$(srctree)/$(src)/services/system/include \
	-I$(srctree)/$(src)/services/include \
	-I$(srctree)/$(src)/services/include/shared \
	-I$(srctree)/$(src)/hwdefs/km \
	-I$(srctree)/$(src)/hwdefs \
	-I$(srctree)/$(src)/generated/htbuffer_bridge \
	-I$(srctree)/$(src)/generated/cmm_bridge \
	-I$(srctree)/$(src)/generated/rgxtq2_bridge \
	-I$(srctree)/$(src)/generated/pdumpctrl_bridge \
	-I$(srctree)/$(src)/generated/rgxtq_bridge \
	-I$(srctree)/$(src)/generated/mm_bridge \
	-I$(srctree)/$(src)/generated/pvrtl_bridge \
	-I$(srctree)/$(src)/generated/rgxfwdbg_bridge \
	-I$(srctree)/$(src)/generated/rgxbreakpoint_bridge \
	-I$(srctree)/$(src)/generated/pdumpmm_bridge \
	-I$(srctree)/$(src)/generated/rgxregconfig_bridge \
	-I$(srctree)/$(src)/generated/pdump_bridge \
	-I$(srctree)/$(src)/generated/sync_bridge \
	-I$(srctree)/$(src)/generated/cache_bridge \
	-I$(srctree)/$(src)/generated/rgxhwperf_bridge \
	-I$(srctree)/$(src)/generated/rgxsignals_bridge \
	-I$(srctree)/$(src)/generated/ri_bridge \
	-I$(srctree)/$(src)/generated/dmabuf_bridge \
	-I$(srctree)/$(src)/generated/devicememhistory_bridge \
	-I$(srctree)/$(src)/generated/synctracking_bridge \
	-I$(srctree)/$(src)/generated/rgxpdump_bridge \
	-I$(srctree)/$(src)/generated/rgxkicksync_bridge \
	-I$(srctree)/$(src)/generated/rgxcmp_bridge \
	-I$(srctree)/$(src)/generated/srvcore_bridge \
	-I$(srctree)/$(src)/generated/rgxta3d_bridge \
	-I$(srctree)/$(src)/services/system/rgx_mtk \
	-I$(srctree)/$(src)/services/system/rgx_mtk/mt8167

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
	kernel/drivers/staging/imgtec/pvr_sw_fence.o \
	kernel/drivers/staging/imgtec/pvr_fence.o \
	kernel/drivers/staging/imgtec/pvr_counting_timeline.o \
	services/server/env/linux/physmem_dmabuf.o \
	services/server/env/linux/osmmap_stub.o \
	services/server/env/linux/osconnection_server.o \
	services/server/env/linux/pvr_gputrace.o \
	services/server/env/linux/physmem_osmem_linux.o \
	services/server/env/linux/allocmem.o \
	services/server/env/linux/pvr_bridge_k.o \
	services/server/env/linux/htb_debug.o \
	services/server/env/linux/physmem_test.o \
	services/server/env/linux/pmr_os.o \
	services/server/env/linux/pvr_debugfs.o \
	services/server/env/linux/event.o \
	services/server/env/linux/km_apphint.o \
	services/server/env/linux/pvr_debug.o \
	services/server/env/linux/module_common.o \
	services/server/env/linux/handle_idr.o \
	services/server/env/linux/osfunc.o \
	services/server/devices/rgx/rgxstartstop.o \
	services/server/devices/rgx/rgxmem.o \
	services/server/devices/rgx/rgxbvnc.o \
	services/server/devices/rgx/rgxmipsmmuinit.o \
	services/server/devices/rgx/rgxlayer_impl.o \
	services/server/devices/rgx/rgxdebug.o \
	services/server/devices/rgx/rgxpdump.o \
	services/server/devices/rgx/rgxkicksync.o \
	services/server/devices/rgx/rgxtransfer.o \
	services/server/devices/rgx/env/linux/km/rgxfwload.o \
	services/server/devices/rgx/rgxccb.o \
	services/server/devices/rgx/rgxsrvinit.o \
	services/server/devices/rgx/rgxhwperf.o \
	services/server/devices/rgx/rgxinit.o \
	services/server/devices/rgx/rgxfwdbg.o \
	services/server/devices/rgx/rgxutils.o \
	services/server/devices/rgx/rgxfwimageutils.o \
	services/server/devices/rgx/rgxcompute.o \
	services/server/devices/rgx/rgxpower.o \
	services/server/devices/rgx/rgxsyncutils.o \
	services/server/devices/rgx/rgxmmuinit.o \
	services/server/devices/rgx/rgxregconfig.o \
	services/server/devices/rgx/rgxbreakpoint.o \
	services/server/devices/rgx/rgxfwutils.o \
	services/server/devices/rgx/rgxtdmtransfer.o \
	services/server/devices/rgx/rgxsignals.o \
	services/server/devices/rgx/rgxtimecorr.o \
	services/server/devices/rgx/rgxta3d.o \
	services/server/common/lists.o \
	services/server/common/physmem_tdfwcode.o \
	services/server/common/tlintern.o \
	services/server/common/power.o \
	services/server/common/physheap.o \
	services/server/common/pdump_server.o \
	services/server/common/physmem_tdsecbuf.o \
	services/server/common/pdump_physmem.o \
	services/server/common/pvrsrv.o \
	services/server/common/mmu_common.o \
	services/server/common/physmem_lma.o \
	services/server/common/pmr.o \
	services/server/common/cache_km.o \
	services/server/common/connection_server.o \
	services/server/common/process_stats.o \
	services/server/common/devicemem_history_server.o \
	services/server/common/physmem_hostmem.o \
	services/server/common/srvcore.o \
	services/server/common/tlserver.o \
	services/server/common/handle.o \
	services/server/common/htbserver.o \
	services/server/common/devicemem_server.o \
	services/server/common/pvrsrv_pool.o \
	services/server/common/info_page_km.o \
	services/server/common/sync_checkpoint.o \
	services/server/common/pdump_mmu.o \
	services/server/common/pvr_notifier.o \
	services/server/common/physmem.o \
	services/server/common/sync_server.o \
	services/server/common/tlstream.o \
	services/server/common/pvrsrv_bridge_init.o \
	services/server/common/devicemem_heapcfg.o \
	services/shared/devices/rgx/rgx_hwperf_table.o \
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
	services/system/common/vz_physheap_generic.o \
	services/system/common/vz_vmm_vm.o \
	services/system/common/env/linux/ion_support_generic.o \
	services/system/common/env/linux/interrupt_support.o \
	services/system/common/env/linux/dma_support.o \
	services/system/common/vmm_pvz_server.o \
	services/system/common/vmm_pvz_client.o \
	services/system/common/vz_support.o \
	services/system/common/vz_physheap_common.o \
	services/system/common/vmm_type_stub.o \
	services/system/common/vz_vmm_pvz.o \
	generated/htbuffer_bridge/client_htbuffer_direct_bridge.o \
	generated/htbuffer_bridge/server_htbuffer_bridge.o \
	generated/cmm_bridge/server_cmm_bridge.o \
	generated/rgxtq2_bridge/server_rgxtq2_bridge.o \
	generated/pdumpctrl_bridge/server_pdumpctrl_bridge.o \
	generated/pdumpctrl_bridge/client_pdumpctrl_direct_bridge.o \
	generated/rgxtq_bridge/server_rgxtq_bridge.o \
	generated/mm_bridge/server_mm_bridge.o \
	generated/mm_bridge/client_mm_direct_bridge.o \
	generated/pvrtl_bridge/client_pvrtl_direct_bridge.o \
	generated/pvrtl_bridge/server_pvrtl_bridge.o \
	generated/rgxfwdbg_bridge/server_rgxfwdbg_bridge.o \
	generated/rgxbreakpoint_bridge/server_rgxbreakpoint_bridge.o \
	generated/pdumpmm_bridge/client_pdumpmm_direct_bridge.o \
	generated/pdumpmm_bridge/server_pdumpmm_bridge.o \
	generated/rgxregconfig_bridge/server_rgxregconfig_bridge.o \
	generated/pdump_bridge/client_pdump_direct_bridge.o \
	generated/pdump_bridge/server_pdump_bridge.o \
	generated/sync_bridge/server_sync_bridge.o \
	generated/sync_bridge/client_sync_direct_bridge.o \
	generated/cache_bridge/client_cache_direct_bridge.o \
	generated/cache_bridge/server_cache_bridge.o \
	generated/rgxhwperf_bridge/server_rgxhwperf_bridge.o \
	generated/rgxsignals_bridge/server_rgxsignals_bridge.o \
	generated/dmabuf_bridge/server_dmabuf_bridge.o \
	generated/devicememhistory_bridge/client_devicememhistory_direct_bridge.o \
	generated/devicememhistory_bridge/server_devicememhistory_bridge.o \
	generated/synctracking_bridge/client_synctracking_direct_bridge.o \
	generated/synctracking_bridge/server_synctracking_bridge.o \
	generated/rgxpdump_bridge/server_rgxpdump_bridge.o \
	generated/rgxpdump_bridge/client_rgxpdump_direct_bridge.o \
	generated/rgxkicksync_bridge/server_rgxkicksync_bridge.o \
	generated/rgxcmp_bridge/server_rgxcmp_bridge.o \
	generated/srvcore_bridge/server_srvcore_bridge.o \
	generated/rgxta3d_bridge/server_rgxta3d_bridge.o

pvrsrvkm-y += \
	services/system/rgx_mtk/mtk_pp.o \
	services/system/rgx_mtk/mt8167/mtk_mfgsys.o \
	services/system/rgx_mtk/sysconfig.o

pvrsrvkm-$(CONFIG_DRM_POWERVR_ROGUE_DEBUG) += \
	generated/ri_bridge/client_ri_direct_bridge.o \
	generated/ri_bridge/server_ri_bridge.o \
	services/server/common/ri_server.o
pvrsrvkm-$(CONFIG_ARM)   += services/server/env/linux/osfunc_arm.o
pvrsrvkm-$(CONFIG_ARM64) += services/server/env/linux/osfunc_arm64.o
pvrsrvkm-$(CONFIG_EVENT_TRACING) += services/server/env/linux/trace_events.o
pvrsrvkm-$(CONFIG_X86)   += services/server/env/linux/osfunc_x86.o

