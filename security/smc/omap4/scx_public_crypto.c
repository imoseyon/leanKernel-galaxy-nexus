/*
 * Copyright (c) 2006-2010 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "scxlnx_defs.h"
#include "scxlnx_util.h"
#include "scxlnx_mshield.h"
#include "scx_public_crypto.h"
#include "scx_public_dma.h"

#define IO_ADDRESS OMAP2_L4_IO_ADDRESS

#define S_SUCCESS		0x00000000
#define S_ERROR_GENERIC		0xFFFF0000
#define S_ERROR_ACCESS_DENIED	0xFFFF0001
#define S_ERROR_BAD_FORMAT	0xFFFF0005
#define S_ERROR_BAD_PARAMETERS	0xFFFF0006
#define S_ERROR_OUT_OF_MEMORY	0xFFFF000C
#define S_ERROR_SHORT_BUFFER	0xFFFF0010
#define S_ERROR_UNREACHABLE	0xFFFF3013
#define S_ERROR_SERVICE		0xFFFF1000

#define CKR_OK			0x00000000

#define PUBLIC_CRYPTO_TIMEOUT_CONST	0x000FFFFF

#define RPC_AES1_CODE	PUBLIC_CRYPTO_HWA_AES1
#define RPC_AES2_CODE	PUBLIC_CRYPTO_HWA_AES2
#define RPC_DES_CODE	PUBLIC_CRYPTO_HWA_DES
#define RPC_SHA_CODE	PUBLIC_CRYPTO_HWA_SHA

#define RPC_CRYPTO_COMMAND_MASK	0x000003c0

#define RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR		0x200
#define RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR_UNLOCK	0x000
#define RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR_LOCK	0x001

#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT			0x240
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_LOCK_AES1	RPC_AES1_CODE
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_LOCK_AES2	RPC_AES2_CODE
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_LOCK_DES		RPC_DES_CODE
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_LOCK_SHA		RPC_SHA_CODE
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_SUSPEND		0x010
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_UNINSTALL	0x020

#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS			0x280
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_AES1	RPC_AES1_CODE
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_AES2	RPC_AES2_CODE
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_DES	RPC_DES_CODE
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_SHA	RPC_SHA_CODE
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_RESUME		0x010

#define RPC_CLEAR_GLOBAL_KEY_CONTEXT			0x2c0
#define RPC_CLEAR_GLOBAL_KEY_CONTEXT_CLEARED_AES	0x001
#define RPC_CLEAR_GLOBAL_KEY_CONTEXT_CLEARED_DES	0x002

#define ENABLE_CLOCK	true
#define DISABLE_CLOCK	false

/*---------------------------------------------------------------------------*/
/*RPC IN/OUT structures for CUS implementation                               */
/*---------------------------------------------------------------------------*/

struct RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR_OUT {
	u32 nShortcutID;
	u32 nError;
};

struct RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR_IN {
	u32 nDeviceContextID;
	u32 hClientSession;
	u32 nCommandID;
	u32 hKeyContext;
	/**
	 *The identifier of the HWA accelerator that this shortcut uses!
	 *Possible values are:
	 *- 1 (RPC_AES1_CODE)
	 *- 2 (RPC_AES2_CODE)
	 *- 4 (RPC_DES_CODE)
	 *- 8 (RPC_SHA_CODE)
	 **/
	u32 nHWAID;
	/**
	 *This field defines the algorithm, direction, mode, key size.
	 *It contains some of the bits of the corresponding "CTRL" register
	 *of the accelerator.
	 *
	 *More precisely:
	 *For AES1 accelerator, nHWA_CTRL contains the following bits:
	 *- CTR (bit 6):
	 * when 1, selects CTR mode.
	 * when 0, selects CBC or ECB mode (according to CBC bit)
	 *- CBC (bit 5)
	 * when 1, selects CBC mode (but only if CTR=0)
	 * when 0, selects EBC mode (but only if CTR=0)
	 *- DIRECTION (bit 2)
	 *  0: decryption
	 *  1: encryption
	 *
	 *For the DES2 accelerator, nHWA_CTRL contains the following bits:
	 *- CBC (bit 4): 1 for CBC, 0 for ECB
	 *- DIRECTION (bit 2): 0 for decryption, 1 for encryption
	 *
	 *For the SHA accelerator, nHWA_CTRL contains the following bits:
	 *- ALGO (bit 2:1):
	 *  0x0: MD5
	 *  0x1: SHA1
	 *  0x2: SHA-224
	 *  0x3: SHA-256
	 **/
	u32 nHWA_CTRL;
	union PUBLIC_CRYPTO_OPERATION_STATE sOperationState;
};

struct RPC_LOCK_HWA_SUSPEND_SHORTCUT_OUT {
	union PUBLIC_CRYPTO_OPERATION_STATE sOperationState;
};

struct RPC_LOCK_HWA_SUSPEND_SHORTCUT_IN {
	u32 nShortcutID;
};

struct RPC_RESUME_SHORTCUT_UNLOCK_HWA_IN {
	u32 nShortcutID;
	u32 hAES1KeyContext;
	u32 hAES2KeyContext;
	u32 hDESKeyContext;
	union PUBLIC_CRYPTO_OPERATION_STATE sOperationState;
};

/*------------------------------------------------------------------------- */
/*
 * HWA public lock or unlock one HWA according algo specified by nHWAID
 */
void PDrvCryptoLockUnlockHWA(u32 nHWAID, bool bDoLock)
{
	int is_sem = 0;
	struct semaphore *s = NULL;
	struct mutex *m = NULL;
	struct SCXLNX_DEVICE *dev = SCXLNXGetDevice();

	dprintk(KERN_INFO "PDrvCryptoLockUnlockHWA:nHWAID=0x%04X bDoLock=%d\n",
		nHWAID, bDoLock);

	switch (nHWAID) {
	case RPC_AES1_CODE:
		s = &dev->sAES1CriticalSection;
		is_sem = 1;
		break;
	case RPC_AES2_CODE:
		s = &dev->sAES2CriticalSection;
		is_sem = 1;
		break;
	case RPC_DES_CODE:
		m = &dev->sDESCriticalSection;
		break;
	default:
	case RPC_SHA_CODE:
		m = &dev->sSHACriticalSection;
		break;
	}

	if (bDoLock == LOCK_HWA) {
		dprintk(KERN_INFO "PDrvCryptoLockUnlockHWA: "
			"Wait for HWAID=0x%04X\n", nHWAID);
		if (is_sem) {
			while (down_trylock(s))
				cpu_relax();
		} else {
			while (!mutex_trylock(m))
				cpu_relax();
		}
		dprintk(KERN_INFO "PDrvCryptoLockUnlockHWA: "
			"Locked on HWAID=0x%04X\n", nHWAID);
	} else {
		if (is_sem)
			up(s);
		else
			mutex_unlock(m);
		dprintk(KERN_INFO "PDrvCryptoLockUnlockHWA: "
			"Released for HWAID=0x%04X\n", nHWAID);
	}
}

/*------------------------------------------------------------------------- */
/**
 *Initialize the public crypto DMA channels, global HWA semaphores and handles
 */
u32 SCXPublicCryptoInit(void)
{
	struct SCXLNX_DEVICE *pDevice = SCXLNXGetDevice();
	u32 nError = PUBLIC_CRYPTO_OPERATION_SUCCESS;

	/* Initialize HWAs */
	PDrvCryptoAESInit();
	PDrvCryptoDigestInit();

	/*initialize the HWA semaphores */
	sema_init(&pDevice->sAES1CriticalSection, 1);
	sema_init(&pDevice->sAES2CriticalSection, 1);
	mutex_init(&pDevice->sSHACriticalSection);

	/*initialize the current key handle loaded in the AESn/DES HWA */
	pDevice->hAES1SecureKeyContext = 0;
	pDevice->hAES2SecureKeyContext = 0;
	pDevice->bSHAM1IsPublic = false;

	/*initialize the DMA semaphores */
	mutex_init(&pDevice->sm.sDMALock);

	/*allocate DMA buffer */
	pDevice->nDMABufferLength = PAGE_SIZE * 16;
	pDevice->pDMABuffer = dma_alloc_coherent(NULL,
		pDevice->nDMABufferLength,
		&(pDevice->pDMABufferPhys),
		GFP_KERNEL);
	if (pDevice->pDMABuffer == NULL) {
		printk(KERN_ERR
			"SCXPublicCryptoInit: Out of memory for DMA buffer\n");
		nError = S_ERROR_OUT_OF_MEMORY;
	}

	return nError;
}

/*------------------------------------------------------------------------- */
/*
 *Initialize the device context CUS fields (shortcut semaphore and public CUS
 *list)
 */
void SCXPublicCryptoInitDeviceContext(struct SCXLNX_CONNECTION *pDeviceContext)
{
	/*initialize the CUS list in the given device context */
	spin_lock_init(&(pDeviceContext->shortcutListCriticalSectionLock));
	INIT_LIST_HEAD(&(pDeviceContext->ShortcutList));
}

/*------------------------------------------------------------------------- */
/**
 *Terminate the public crypto (including DMA)
 */
void SCXPublicCryptoTerminate()
{
	struct SCXLNX_DEVICE *pDevice = SCXLNXGetDevice();

	if (pDevice->pDMABuffer != NULL) {
		dma_free_coherent(NULL, pDevice->nDMABufferLength,
			pDevice->pDMABuffer,
			pDevice->pDMABufferPhys);
		pDevice->pDMABuffer = NULL;
	}

	PDrvCryptoDigestExit();
	PDrvCryptoAESExit();
}

/*------------------------------------------------------------------------- */

void SCXPublicCryptoWaitForReadyBitInfinitely(u32 *pRegister, u32 vBit)
{
	while (!(INREG32(pRegister) & vBit))
		;
}

/*------------------------------------------------------------------------- */

u32 SCXPublicCryptoWaitForReadyBit(u32 *pRegister, u32 vBit)
{
	u32 timeoutCounter = PUBLIC_CRYPTO_TIMEOUT_CONST;

	while ((!(INREG32(pRegister) & vBit)) && ((--timeoutCounter) != 0))
		;

	if (timeoutCounter == 0)
		return PUBLIC_CRYPTO_ERR_TIMEOUT;

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*------------------------------------------------------------------------- */

static DEFINE_SPINLOCK(clk_lock);

void SCXPublicCryptoDisableClock(uint32_t vClockPhysAddr)
{
	u32 *pClockReg;
	u32 val;
	unsigned long flags;

	dprintk(KERN_INFO "SCXPublicCryptoDisableClock: " \
		"vClockPhysAddr=0x%08X\n",
		vClockPhysAddr);

	/* Ensure none concurrent access when changing clock registers */
	spin_lock_irqsave(&clk_lock, flags);

	pClockReg = (u32 *)IO_ADDRESS(vClockPhysAddr);

	val = __raw_readl(pClockReg);
	val &= ~(0x3);
	__raw_writel(val, pClockReg);

	/* Wait for clock to be fully disabled */
	while ((__raw_readl(pClockReg) & 0x30000) == 0)
		;

	spin_unlock_irqrestore(&clk_lock, flags);

	tf_l4sec_clkdm_allow_idle(false, true);
}

/*------------------------------------------------------------------------- */

void SCXPublicCryptoEnableClock(uint32_t vClockPhysAddr)
{
	u32 *pClockReg;
	u32 val;
	unsigned long flags;

	dprintk(KERN_INFO "SCXPublicCryptoEnableClock: " \
		"vClockPhysAddr=0x%08X\n",
		vClockPhysAddr);

	tf_l4sec_clkdm_wakeup(false, true);

	/* Ensure none concurrent access when changing clock registers */
	spin_lock_irqsave(&clk_lock, flags);

	pClockReg = (u32 *)IO_ADDRESS(vClockPhysAddr);

	val = __raw_readl(pClockReg);
	val |= 0x2;
	__raw_writel(val, pClockReg);

	/* Wait for clock to be fully enabled */
	while ((__raw_readl(pClockReg) & 0x30000) != 0)
		;

	spin_unlock_irqrestore(&clk_lock, flags);
}

