#
# Makefile for HSI drivers
#
EXTRA_CFLAGS :=

omap_hsi-objs	:=	hsi_driver.o hsi_driver_dma.o hsi_driver_int.o \
			hsi_driver_if.o hsi_driver_bus.o hsi_driver_gpio.o \
			hsi_driver_fifo.o

ifeq ($(CONFIG_DEBUG_FS), y)
	omap_hsi-objs += hsi_driver_debugfs.o
endif

hsi_char-objs	:=	hsi-char.o hsi-if.o

hsi-protocol-objs    := hsi_protocol.o hsi_protocol_if.o \
                        hsi_protocol_cmd.o

obj-$(CONFIG_OMAP_HSI)		+= omap_hsi.o
obj-$(CONFIG_OMAP_HSI_CHAR)	+= hsi_char.o
obj-$(CONFIG_OMAP_HSI_PROTOCOL) += hsi-protocol.o
