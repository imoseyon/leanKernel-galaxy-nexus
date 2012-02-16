#
# Copyright (c) 2006-2010 Trusted Logic S.A.
# All Rights Reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

ifdef S_VERSION_BUILD
EXTRA_CFLAGS += -DS_VERSION_BUILD=$(S_VERSION_BUILD)
endif

EXTRA_CFLAGS += -Iarch/arm/mach-omap2
EXTRA_CFLAGS += -Iarch/arm/plat-omap/include/plat
EXTRA_CFLAGS += -DCONFIG_TF_TEEC
EXTRA_CFLAGS += -DCONFIG_TF_ION

tf_driver-objs += tf_util.o
tf_driver-objs += tf_conn.o
tf_driver-objs += tf_device.o
tf_driver-objs += tf_comm.o
tf_driver-objs += tf_crypto.o
tf_driver-objs += tf_crypto_digest.o
tf_driver-objs += tf_crypto_aes.o
tf_driver-objs += tf_crypto_des.o
tf_driver-objs += tf_dma.o
tf_driver-objs += tf_comm_mshield.o
tf_driver-objs += tf_device_mshield.o
tf_driver-objs += bridge_pub2sec.o
tf_driver-objs += tf_teec.o

obj-$(CONFIG_SECURITY_MIDDLEWARE_COMPONENT) += tf_driver.o
obj-$(CONFIG_SECURITY_MIDDLEWARE_COMPONENT) += rproc_drm.o

plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_bridge_pub2sec.o :=-Wa,-march=armv7-a$(plus_sec)
