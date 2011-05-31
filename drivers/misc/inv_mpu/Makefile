
# Kernel makefile for motions sensors
#
#

# MPU
obj-$(CONFIG_MPU_SENSORS_MPU3050)	+= mpu3050.o

mpu3050-objs += mpuirq.o
mpu3050-objs += slaveirq.o
mpu3050-objs += mpu-dev.o
mpu3050-objs += mlsl-kernel.o
mpu3050-objs += mldl_cfg.o

EXTRA_CFLAGS += -Idrivers/misc/inv_mpu

obj-$(CONFIG_MPU_SENSORS_TIMERIRQ)+= timerirq.o

obj-y			+= accel/
obj-y			+= compass/
obj-y			+= pressure/

