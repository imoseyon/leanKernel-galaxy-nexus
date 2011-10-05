/*
 * hsi_driver_debugfs.c
 *
 * Implements HSI debugfs.
 *
 * Copyright (C) 2007-2008 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Carlos Chinea <carlos.chinea@nokia.com>
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/err.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include "hsi_driver.h"

#define HSI_DIR_NAME_SIZE	64

static struct dentry *hsi_dir;

static int hsi_debug_show(struct seq_file *m, void *p)
{
	struct hsi_dev *hsi_ctrl = m->private;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);

	hsi_clocks_enable(hsi_ctrl->dev, __func__);

	seq_printf(m, "REVISION\t: 0x%08x\n",
		   hsi_inl(hsi_ctrl->base, HSI_SYS_REVISION_REG));
	if (hsi_driver_device_is_hsi(pdev))
		seq_printf(m, "HWINFO\t\t: 0x%08x\n",
			   hsi_inl(hsi_ctrl->base, HSI_SYS_HWINFO_REG));
	seq_printf(m, "SYSCONFIG\t: 0x%08x\n",
		   hsi_inl(hsi_ctrl->base, HSI_SYS_SYSCONFIG_REG));
	seq_printf(m, "SYSSTATUS\t: 0x%08x\n",
		   hsi_inl(hsi_ctrl->base, HSI_SYS_SYSSTATUS_REG));

	hsi_clocks_disable(hsi_ctrl->dev, __func__);

	return 0;
}

static int hsi_debug_port_show(struct seq_file *m, void *p)
{
	struct hsi_port *hsi_port = m->private;
	struct hsi_dev *hsi_ctrl = hsi_port->hsi_controller;
	void __iomem *base = hsi_ctrl->base;
	unsigned int port = hsi_port->port_number;
	int ch, fifo;
	long buff_offset;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);

	hsi_clocks_enable(hsi_ctrl->dev, __func__);

	if (hsi_port->cawake_gpio >= 0)
		seq_printf(m, "CAWAKE\t\t: %d\n", hsi_get_cawake(hsi_port));

	seq_printf(m, "WAKE\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_SYS_WAKE_REG(port)));
	seq_printf(m, "MPU_ENABLE_IRQ%d\t: 0x%08x\n", hsi_port->n_irq,
		   hsi_inl(base,
			   HSI_SYS_MPU_ENABLE_REG(port, hsi_port->n_irq)));
	seq_printf(m, "MPU_STATUS_IRQ%d\t: 0x%08x\n", hsi_port->n_irq,
		   hsi_inl(base,
			   HSI_SYS_MPU_STATUS_REG(port, hsi_port->n_irq)));
	if (hsi_driver_device_is_hsi(pdev)) {
		seq_printf(m, "MPU_U_ENABLE_IRQ%d\t: 0x%08x\n",
			   hsi_port->n_irq,
			   hsi_inl(base, HSI_SYS_MPU_U_ENABLE_REG(port,
							hsi_port->n_irq)));
		seq_printf(m, "MPU_U_STATUS_IRQ%d\t: 0x%08x\n", hsi_port->n_irq,
			   hsi_inl(base,
				   HSI_SYS_MPU_U_STATUS_REG(port,
							    hsi_port->n_irq)));
	}
	/* HST */
	seq_printf(m, "\nHST\n===\n");
	seq_printf(m, "MODE\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HST_MODE_REG(port)));
	seq_printf(m, "FRAMESIZE\t: 0x%08x\n",
		   hsi_inl(base, HSI_HST_FRAMESIZE_REG(port)));
	seq_printf(m, "DIVISOR\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HST_DIVISOR_REG(port)));
	seq_printf(m, "CHANNELS\t: 0x%08x\n",
		   hsi_inl(base, HSI_HST_CHANNELS_REG(port)));
	seq_printf(m, "ARBMODE\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HST_ARBMODE_REG(port)));
	seq_printf(m, "TXSTATE\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HST_TXSTATE_REG(port)));
	if (hsi_driver_device_is_hsi(pdev)) {
		seq_printf(m, "BUFSTATE P1\t: 0x%08x\n",
			   hsi_inl(base, HSI_HST_BUFSTATE_REG(1)));
		seq_printf(m, "BUFSTATE P2\t: 0x%08x\n",
			   hsi_inl(base, HSI_HST_BUFSTATE_REG(2)));
	} else {
		seq_printf(m, "BUFSTATE\t: 0x%08x\n",
			   hsi_inl(base, HSI_HST_BUFSTATE_REG(port)));
	}
	seq_printf(m, "BREAK\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HST_BREAK_REG(port)));
	for (ch = 0; ch < 8; ch++) {
		buff_offset = hsi_hst_buffer_reg(hsi_ctrl, port, ch);
		if (buff_offset >= 0)
			seq_printf(m, "BUFFER_CH%d\t: 0x%08x\n", ch,
				   hsi_inl(base, buff_offset));
	}
	if (hsi_driver_device_is_hsi(pdev)) {
		for (fifo = 0; fifo < HSI_HST_FIFO_COUNT; fifo++) {
			seq_printf(m, "FIFO MAPPING%d\t: 0x%08x\n", fifo,
				   hsi_inl(base,
					   HSI_HST_MAPPING_FIFO_REG(fifo)));
		}
	}
	/* HSR */
	seq_printf(m, "\nHSR\n===\n");
	seq_printf(m, "MODE\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HSR_MODE_REG(port)));
	seq_printf(m, "FRAMESIZE\t: 0x%08x\n",
		   hsi_inl(base, HSI_HSR_FRAMESIZE_REG(port)));
	seq_printf(m, "CHANNELS\t: 0x%08x\n",
		   hsi_inl(base, HSI_HSR_CHANNELS_REG(port)));
	seq_printf(m, "COUNTERS\t: 0x%08x\n",
		   hsi_inl(base, HSI_HSR_COUNTERS_REG(port)));
	seq_printf(m, "RXSTATE\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HSR_RXSTATE_REG(port)));
	if (hsi_driver_device_is_hsi(pdev)) {
		seq_printf(m, "BUFSTATE P1\t: 0x%08x\n",
			   hsi_inl(base, HSI_HSR_BUFSTATE_REG(1)));
		seq_printf(m, "BUFSTATE P2\t: 0x%08x\n",
			   hsi_inl(base, HSI_HSR_BUFSTATE_REG(2)));
	} else {
		seq_printf(m, "BUFSTATE\t: 0x%08x\n",
			   hsi_inl(base, HSI_HSR_BUFSTATE_REG(port)));
	}
	seq_printf(m, "BREAK\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HSR_BREAK_REG(port)));
	seq_printf(m, "ERROR\t\t: 0x%08x\n",
		   hsi_inl(base, HSI_HSR_ERROR_REG(port)));
	seq_printf(m, "ERRORACK\t: 0x%08x\n",
		   hsi_inl(base, HSI_HSR_ERRORACK_REG(port)));
	for (ch = 0; ch < 8; ch++) {
		buff_offset = hsi_hsr_buffer_reg(hsi_ctrl, port, ch);
		if (buff_offset >= 0)
			seq_printf(m, "BUFFER_CH%d\t: 0x%08x\n", ch,
				   hsi_inl(base, buff_offset));
	}
	if (hsi_driver_device_is_hsi(pdev)) {
		for (fifo = 0; fifo < HSI_HSR_FIFO_COUNT; fifo++) {
			seq_printf(m, "FIFO MAPPING%d\t: 0x%08x\n", fifo,
				   hsi_inl(base,
					   HSI_HSR_MAPPING_FIFO_REG(fifo)));
		}
		seq_printf(m, "DLL\t: 0x%08x\n",
			   hsi_inl(base, HSI_HSR_DLL_REG));
		seq_printf(m, "DIVISOR\t: 0x%08x\n",
			   hsi_inl(base, HSI_HSR_DIVISOR_REG(port)));
	}

	hsi_clocks_disable(hsi_ctrl->dev, __func__);

	return 0;
}

static int hsi_debug_gdd_show(struct seq_file *m, void *p)
{
	struct hsi_dev *hsi_ctrl = m->private;
	void __iomem *base = hsi_ctrl->base;
	int lch;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);

	hsi_clocks_enable(hsi_ctrl->dev, __func__);

	seq_printf(m, "GDD_MPU_STATUS\t: 0x%08x\n",
		   hsi_inl(base, HSI_SYS_GDD_MPU_IRQ_STATUS_REG));
	seq_printf(m, "GDD_MPU_ENABLE\t: 0x%08x\n\n",
		   hsi_inl(base, HSI_SYS_GDD_MPU_IRQ_ENABLE_REG));

	if (!hsi_driver_device_is_hsi(pdev)) {
		seq_printf(m, "HW_ID\t\t: 0x%08x\n",
			   hsi_inl(base, HSI_SSI_GDD_HW_ID_REG));
		seq_printf(m, "PPORT_ID\t: 0x%08x\n",
			   hsi_inl(base, HSI_SSI_GDD_PPORT_ID_REG));
		seq_printf(m, "MPORT_ID\t: 0x%08x\n",
			   hsi_inl(base, HSI_SSI_GDD_MPORT_ID_REG));
		seq_printf(m, "TEST\t\t: 0x%08x\n",
			   hsi_inl(base, HSI_SSI_GDD_TEST_REG));
	}

	seq_printf(m, "GCR\t\t: 0x%08x\n", hsi_inl(base, HSI_GDD_GCR_REG));

	for (lch = 0; lch < hsi_ctrl->gdd_chan_count; lch++) {
		seq_printf(m, "\nGDD LCH %d\n=========\n", lch);
		seq_printf(m, "CSDP\t\t: 0x%04x\n",
			   hsi_inw(base, HSI_GDD_CSDP_REG(lch)));
		seq_printf(m, "CCR\t\t: 0x%04x\n",
			   hsi_inw(base, HSI_GDD_CCR_REG(lch)));
		seq_printf(m, "CICR\t\t: 0x%04x\n",
			   hsi_inw(base, HSI_GDD_CCIR_REG(lch)));
		seq_printf(m, "CSR\t\t: 0x%04x\n",
			   hsi_inw(base, HSI_GDD_CSR_REG(lch)));
		seq_printf(m, "CSSA\t\t: 0x%08x\n",
			   hsi_inl(base, HSI_GDD_CSSA_REG(lch)));
		seq_printf(m, "CDSA\t\t: 0x%08x\n",
			   hsi_inl(base, HSI_GDD_CDSA_REG(lch)));
		seq_printf(m, "CEN\t\t: 0x%04x\n",
			   hsi_inw(base, HSI_GDD_CEN_REG(lch)));
		seq_printf(m, "CSAC\t\t: 0x%04x\n",
			   hsi_inw(base, HSI_GDD_CSAC_REG(lch)));
		seq_printf(m, "CDAC\t\t: 0x%04x\n",
			   hsi_inw(base, HSI_GDD_CDAC_REG(lch)));
		if (!hsi_driver_device_is_hsi(pdev))
			seq_printf(m, "CLNK_CTRL\t: 0x%04x\n",
				   hsi_inw(base,
					   HSI_SSI_GDD_CLNK_CTRL_REG(lch)));
	}

	hsi_clocks_disable(hsi_ctrl->dev, __func__);

	return 0;
}

static int hsi_port_counters_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int hsi_port_counters_release(struct inode *inode, struct file *file)
{
	return 0;
}

static loff_t hsi_port_counters_seek(struct file *file, loff_t off, int whence)
{
	return 0;
}

static ssize_t hsi_port_counters_read(struct file *filep, char __user * buff,
				      size_t count, loff_t *offp)
{
	ssize_t ret;
	struct hsi_port *hsi_port = filep->private_data;
	struct hsi_dev *hsi_ctrl = hsi_port->hsi_controller;
	void __iomem *base = hsi_ctrl->base;
	unsigned int port = hsi_port->port_number;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);
	char str[50];
	unsigned long reg;

	if (*offp > 0) {
		ret = 0;
		goto hsi_cnt_rd_bk;
	}

	hsi_clocks_enable(hsi_ctrl->dev, __func__);

	reg = hsi_inl(base, HSI_HSR_COUNTERS_REG(port));

	hsi_clocks_disable(hsi_ctrl->dev, __func__);

	if (hsi_driver_device_is_hsi(pdev)) {
		sprintf(str, "FT:%d, TB:%d, FB:%d\n",
			(int)(reg & HSI_COUNTERS_FT_MASK) >>
			HSI_COUNTERS_FT_OFFSET,
			(int)(reg & HSI_COUNTERS_TB_MASK) >>
			HSI_COUNTERS_TB_OFFSET,
			(int)(reg & HSI_COUNTERS_FB_MASK) >>
			HSI_COUNTERS_FB_OFFSET);
	} else {
		sprintf(str, "timeout:%d\n", (int)reg);
	}

	ret = strlen(str);
	if (copy_to_user((void __user *)buff, str, ret)) {
		dev_err(hsi_ctrl->dev, "copy_to_user failed\n");
		ret = 0;
	} else {
		*offp = ret;
	}

hsi_cnt_rd_bk:
	return ret;
}

/*
 * Split the buffer `buf' into space-separated words.
 * Return the number of words or <0 on error.
 */
static int hsi_debug_tokenize(char *buf, char *words[], int maxwords)
{
	int nwords = 0;

	while (*buf) {
		char *end;

		/* Skip leading whitespace */
		while (*buf && isspace(*buf))
			buf++;
		if (!*buf)
			break;	/* oh, it was trailing whitespace */

		/* Run `end' over a word */
		for (end = buf; *end && !isspace(*end); end++)
			;
		/* `buf' is the start of the word, `end' is one past the end */

		if (nwords == maxwords)
			return -EINVAL;	/* ran out of words[] before bytes */
		if (*end)
			*end++ = '\0';	/* terminate the word */
		words[nwords++] = buf;
		buf = end;
	}
	return nwords;
}

static ssize_t hsi_port_counters_write(struct file *filep,
				       const char __user *buff, size_t count,
				       loff_t *offp)
{
	ssize_t ret;
	struct hsi_port *hsi_port = filep->private_data;
	struct hsi_dev *hsi_ctrl = hsi_port->hsi_controller;
	void __iomem *base = hsi_ctrl->base;
	unsigned int port = hsi_port->port_number;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);
#define MAXWORDS 4
	int nwords;
	char *words[MAXWORDS];
	char tmpbuf[256];
	unsigned long reg, ft, tb, fb;

	if (count == 0)
		return 0;
	if (count > sizeof(tmpbuf) - 1)
		return -E2BIG;
	if (copy_from_user(tmpbuf, buff, count))
		return -EFAULT;
	tmpbuf[count] = '\0';
	dev_dbg(hsi_ctrl->dev, "%s: read %d bytes from userspace\n",
		__func__, (int)count);

	nwords = hsi_debug_tokenize(tmpbuf, words, MAXWORDS);
	if (nwords < 0) {
		dev_warn(hsi_ctrl->dev,
			"HSI counters write usage: echo <values> > counters\n");
		return -EINVAL;
	}

	hsi_clocks_enable(hsi_ctrl->dev, __func__);

	if (hsi_driver_device_is_hsi(pdev)) {
		if (nwords != 3) {
			dev_warn(hsi_ctrl->dev, "HSI counters write usage: "
				 "echo \"FT TB FB\" > counters\n");
			ret = -EINVAL;
			goto hsi_cnt_w_bk1;
		}
		strict_strtoul(words[0], 0, &ft);
		strict_strtoul(words[1], 0, &tb);
		strict_strtoul(words[2], 0, &fb);
		reg = ((ft << HSI_COUNTERS_FT_OFFSET & HSI_COUNTERS_FT_MASK) |
		       (tb << HSI_COUNTERS_TB_OFFSET & HSI_COUNTERS_TB_MASK) |
		       (fb << HSI_COUNTERS_FB_OFFSET & HSI_COUNTERS_FB_MASK));
	} else {
		if (nwords != 1) {
			dev_warn(hsi_ctrl->dev, "HSI counters write usage: "
				 "echo \"timeout\" > counters\n");
			ret = -EINVAL;
			goto hsi_cnt_w_bk1;
		}
		strict_strtoul(words[0], 0, &reg);
	}
	hsi_outl(reg, base, HSI_HSR_COUNTERS_REG(port));
	ret = count;
	*offp += count;

hsi_cnt_w_bk1:

	hsi_clocks_disable(hsi_ctrl->dev, __func__);

	return ret;
}

static int hsi_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsi_debug_show, inode->i_private);
}

static int hsi_port_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsi_debug_port_show, inode->i_private);
}

static int hsi_gdd_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsi_debug_gdd_show, inode->i_private);
}

static const struct file_operations hsi_regs_fops = {
	.open = hsi_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations hsi_port_regs_fops = {
	.open = hsi_port_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations hsi_port_counters_fops = {
	.open = hsi_port_counters_open,
	.read = hsi_port_counters_read,
	.write = hsi_port_counters_write,
	.llseek = hsi_port_counters_seek,
	.release = hsi_port_counters_release,
};

static const struct file_operations hsi_gdd_regs_fops = {
	.open = hsi_gdd_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int __init hsi_debug_add_ctrl(struct hsi_dev *hsi_ctrl)
{
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);
	unsigned char dir_name[HSI_DIR_NAME_SIZE];
	struct dentry *dir;
	unsigned int port;

	if (pdev->id < 0) {
		hsi_ctrl->dir = debugfs_create_dir(pdev->name, hsi_dir);
	} else {
		snprintf(dir_name, sizeof(dir_name), "%s%d", pdev->name,
			 pdev->id);
		hsi_ctrl->dir = debugfs_create_dir(dir_name, hsi_dir);
	}
	if (IS_ERR(hsi_ctrl->dir))
		return PTR_ERR(hsi_ctrl->dir);

	debugfs_create_file("regs", S_IRUGO, hsi_ctrl->dir, hsi_ctrl,
			    &hsi_regs_fops);

	for (port = 0; port < hsi_ctrl->max_p; port++) {
		snprintf(dir_name, sizeof(dir_name), "port%d", port + 1);
		dir = debugfs_create_dir(dir_name, hsi_ctrl->dir);
		if (IS_ERR(dir))
			goto rback;
		debugfs_create_file("regs", S_IRUGO, dir,
				    &hsi_ctrl->hsi_port[port],
				    &hsi_port_regs_fops);
		debugfs_create_file("counters", S_IRUGO | S_IWUSR, dir,
				    &hsi_ctrl->hsi_port[port],
				    &hsi_port_counters_fops);
	}

	dir = debugfs_create_dir("gdd", hsi_ctrl->dir);
	if (IS_ERR(dir))
		goto rback;
	debugfs_create_file("regs", S_IRUGO, dir, hsi_ctrl, &hsi_gdd_regs_fops);

	return 0;
rback:
	debugfs_remove_recursive(hsi_ctrl->dir);
	return PTR_ERR(dir);
}

void hsi_debug_remove_ctrl(struct hsi_dev *hsi_ctrl)
{
	debugfs_remove_recursive(hsi_ctrl->dir);
}

int __init hsi_debug_init(void)
{
	hsi_dir = debugfs_create_dir("hsi", NULL);
	if (IS_ERR(hsi_dir))
		return PTR_ERR(hsi_dir);

	return 0;
}

void hsi_debug_exit(void)
{
	debugfs_remove_recursive(hsi_dir);
}
