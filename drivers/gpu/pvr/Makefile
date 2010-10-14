#
ccflags-y = -DLINUX -D__linux__ -Idrivers/gpu/pvr \
	-DANDROID \
	-DPVR_BUILD_DIR="\"omap_sgx_android\"" \
	-DSGX_DYNAMIC_TIMING_INFO \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DSERVICES4 \
	-DSUPPORT_SRVINIT \
	-DSUPPORT_PERCONTEXT_PB \
	-DTRANSFER_QUEUE \
	-DSYS_USING_INTERRUPTS \
	-DPVR_SECURE_HANDLES \
	-DPVR_SECURE_FD_EXPORT \
	-DLDM_PLATFORM \
	-DSUPPORT_SGX_HWPERF \
	-DSUPPORT_MEMINFO_IDS \
	-DDISPLAY_CONTROLLER=omaplfb \
	-DSYS_SGX_ACTIVE_POWER_LATENCY_MS=100 \
	-DPVR_BUILD_DATE="" \
	-DSUPPORT_SGX \
	-DCLIENT_DRIVER_DEFAULT_WAIT_RETRIES=50 \
	-DPVR_LINUX_TIMERS_USING_WORKQUEUES \
	-DFLIP_TECHNIQUE_OVERLAY \
	-DPVR_LINUX_USING_WORKQUEUES \
	-DSUPPORT_ANDROID_PLATFORM \
	-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DSUPPORT_SGX_NEW_STATUS_VALS \
        -DSYS_OMAP3430_PIN_MEMORY_BUS_CLOCK \
	-DPVRSRV_MODNAME="\"pvrsrvkm"\"

ccflags-$(CONFIG_SGX540) +=  -Idrivers/gpu/pvr/omap4  -Idrivers/gpu/pvr/sgx \
	-DNO_OMAP_TIMER \
	-DSGX_CLK_CORE_DIV5 \
	-DSGX540 -DSUPPORT_SGX540

ccflags-$(CONFIG_SGX_REV110) += -DSGX_CORE_REV=110
ccflags-$(CONFIG_SGX_REV120) += -DSGX_CORE_REV=120

ccflags-$(CONFIG_SGX530) +=  -Idrivers/gpu/pvr/omap3  -Idrivers/gpu/pvr/sgx \
	-DSGX530 -DSUPPORT_SGX530

ccflags-$(CONFIG_SGX_REV125) += -DSGX_CORE_REV=125
ccflags-$(CONFIG_SGX_REV121) += -DSGX_CORE_REV=121

ccflags-$(CONFIG_SGX_530_BUILD_RELEASE) += \
	-DPVR_BUILD_TYPE="\"release\"" \
	-DRELEASE \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DSGX_EARLYSUSPEND \
	-DSUPPORT_HW_RECOVERY \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING

ccflags-$(CONFIG_SGX_530_BUILD_DEBUG) += \
	-DPVR_BUILD_TYPE="\"debug\""  -DDEBUG \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_LINUX_MEM_AREAS \
	-DDEBUG_LINUX_MMAP_AREAS \
	-DDEBUG_BRIDGE_KM \
	-DPVRSRV_USSE_EDM_STATUS_DEBUG \
	-DPVRSRV_DUMP_MK_TRACE \
	-DDEBUG_LOG_PATH_TRUNCATE="\"eurasia_km\""

ccflags-$(CONFIG_SGX_540_BUILD_RELEASE) += \
	-DPVR_BUILD_TYPE="\"release\"" \
	-DRELEASE \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DPVR_NO_FULL_CACHE_OPS \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING

ccflags-$(CONFIG_SGX_540_BUILD_DEBUG) += \
	-DPVR_BUILD_TYPE="\"debug\""  -DDEBUG \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_LINUX_MEM_AREAS \
	-DDEBUG_LINUX_MMAP_AREAS \
	-DDEBUG_BRIDGE_KM \
	-DPVRSRV_USSE_EDM_STATUS_DEBUG \
	-DPVRSRV_DUMP_MK_TRACE \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DPVR_NO_FULL_CACHE_OPS \
	-DDEBUG_LOG_PATH_TRUNCATE="\"eurasia_km\""

pvr_common-y := \
	osfunc.o \
	mutils.o \
	mmap.o \
	module.o \
	pdump.o \
	proc.o \
	pvr_bridge_k.o \
	pvr_debug.o \
	mm.o \
	mutex.o \
	event.o \
	osperproc.o \
	buffer_manager.o \
	devicemem.o \
	deviceclass.o \
	handle.o \
	hash.o \
	metrics.o \
	pvrsrv.o \
	queue.o \
	ra.o \
	resman.o \
	power.o \
	mem.o \
	pdump_common.o \
	bridged_support.o \
	bridged_pvr_bridge.o \
	perproc.o \
	lists.o \
	mem_debug.o \
	osfunc_common.o

pvr540-$(CONFIG_SGX540) := \
	omap4/sysconfig.o \
	omap4/sysutils.o

pvr530-$(CONFIG_SGX530) := \
	omap3/sysconfig.o \
	omap3/sysutils.o

sgx-y :=  \
	sgx/bridged_sgx_bridge.o \
	sgx/sgxinit.o \
	sgx/sgxpower.o \
	sgx/sgxreset.o \
	sgx/sgxutils.o \
	sgx/sgxkick.o \
	sgx/sgxtransfer.o \
	sgx/mmu.o \
	sgx/pb.o

sgx_displayclass-y := \
	display/omap_sgx_displayclass.o \
	display/omap_display.o

omaplfb-y := \
	omaplfb/omaplfb_displayclass.o \
	omaplfb/omaplfb_linux.o

obj-$(CONFIG_SGX540) := pvr_common.o sgx.o omaplfb.o pvr540.o
obj-$(CONFIG_SGX530) := pvr_common.o sgx.o omaplfb.o pvr530.o
obj-$(CONFIG_VIRTUAL_DISPLAY_SUPPORT) := pvr_common.o sgx.o \
	sgx_displayclass.o pvr540.o

