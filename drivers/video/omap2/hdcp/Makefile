#
# Makefile for HDCP linux kernel module.
#

ccflags-$(CONFIG_OMAP4_HDCP_DEBUG) = -DDEBUG -DHDCP_DEBUG

obj-$(CONFIG_OMAP4_HDCP) += hdcp.o
hdcp-y := hdcp_top.o hdcp_lib.o hdcp_ddc.o
