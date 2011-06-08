obj-$(CONFIG_TI_TILER) += tcm/

obj-$(CONFIG_TI_TILER) += tiler.o
tiler-objs = tiler-geom.o tiler-main.o tiler-iface.o tiler-reserve.o tmm-pat.o

ifdef CONFIG_TILER_ENABLE_NV12
tiler-objs += tiler-nv12.o
endif

obj-$(CONFIG_TI_TILER) += tiler_dmm.o
tiler_dmm-objs = dmm.o
