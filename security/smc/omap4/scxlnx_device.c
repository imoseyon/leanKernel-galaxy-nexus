/*
 * Copyright (c) 2006-2010 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <asm/atomic.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/pm.h>
#include <linux/sysdev.h>
#include <linux/vmalloc.h>
#include <linux/signal.h>
#ifdef CONFIG_ANDROID
#include <linux/device.h>
#endif

#include "scx_protocol.h"
#include "scxlnx_defs.h"
#include "scxlnx_util.h"
#ifdef CONFIG_TF_MSHIELD
#include <plat/cpu.h>
#include "scx_public_crypto.h"
#endif

/* The single device supported by this driver */
static struct SCXLNX_DEVICE g_SCXLNXDevice = {0, };

/*----------------------------------------------------------------------------
 * Implementations
 *----------------------------------------------------------------------------*/

struct SCXLNX_DEVICE *SCXLNXGetDevice(void)
{
	return &g_SCXLNXDevice;
}

/*----------------------------------------------------------------------------*/

static int __init register_dmcrypt_engines(void)
{
        int ret;

        printk(KERN_INFO "Entered register_dmcrypt_engines");

        ret = SCXPublicCryptoInit();
        if (ret) {
                printk(KERN_ERR "register_dmcrypt_engines():"
                        " SCXPublicCryptoInit failed, (error %d)!\n", ret);
                goto out;
        }

        ret = register_smc_public_crypto_aes();
        if (ret) {
                printk(KERN_ERR "register_dmcrypt_engines():"
                        " regiser_smc_public_crypto_aes failed, (error %d)!\n", ret);
                goto out;
        }

        ret = register_smc_public_crypto_digest();
        if (ret) {
                printk(KERN_ERR "register_dmcrypt_engines():"
                        " regiser_smc_public_crypto_digest failed, (error %d)!\n", ret);
                goto out;
        }

out:
        return ret;
}
module_init(register_dmcrypt_engines);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Trusted Logic S.A.");
