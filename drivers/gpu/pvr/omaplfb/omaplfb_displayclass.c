/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/fb.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32))
#include <plat/vrfb.h>
#include <plat/display.h>
#else
#include <mach/vrfb.h>
#include <mach/display.h>
#endif

#ifdef RELEASE
#include <../drivers/video/omap2/omapfb/omapfb.h>
#undef DEBUG
#else
#undef DEBUG
#include <../drivers/video/omap2/omapfb/omapfb.h>
#endif

#include <linux/module.h>
#include <linux/string.h>
#include <linux/notifier.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"

#define OMAPLFB_COMMAND_COUNT		1
#define MAX_BUFFERS_FLIPPING		3
/* Put 0 as desired bpp to use the default in the framebuffer */
#define DESIRED_BPP			0 /* Possible values 32,16,0 */

/* Pointer Display->Services */
static PFN_DC_GET_PVRJTABLE pfnGetPVRJTable = NULL;

/* Pointer to the display devices */
static OMAPLFB_DEVINFO *pDisplayDevices = NULL;

static void OMAPLFBSyncIHandler(struct work_struct*);

/*
 * Swap to display buffer. This buffer refers to one inside the
 * framebuffer memory.
 * in: hDevice, hBuffer, ui32SwapInterval, hPrivateTag, ui32ClipRectCount,
 * psClipRect
 */
static PVRSRV_ERROR SwapToDCBuffer(IMG_HANDLE hDevice,
                                   IMG_HANDLE hBuffer,
                                   IMG_UINT32 ui32SwapInterval,
                                   IMG_HANDLE hPrivateTag,
                                   IMG_UINT32 ui32ClipRectCount,
                                   IMG_RECT *psClipRect)
{
	/* Nothing to do */
	return PVRSRV_OK;
}

/*
 * Set display destination rectangle.
 * in: hDevice, hSwapChain, psRect
 */
static PVRSRV_ERROR SetDCDstRect(IMG_HANDLE hDevice,
	IMG_HANDLE hSwapChain,
	IMG_RECT *psRect)
{
	/* Nothing to do */
	return PVRSRV_ERROR_NOT_SUPPORTED;
}

/*
 * Set display source rectangle.
 * in: hDevice, hSwapChain, psRect
 */
static PVRSRV_ERROR SetDCSrcRect(IMG_HANDLE hDevice,
                                 IMG_HANDLE hSwapChain,
                                 IMG_RECT *psRect)
{
	/* Nothing to do */
	return PVRSRV_ERROR_NOT_SUPPORTED;
}

/*
 * Set display destination colour key.
 * in: hDevice, hSwapChain, ui32CKColour
 */
static PVRSRV_ERROR SetDCDstColourKey(IMG_HANDLE hDevice,
                                      IMG_HANDLE hSwapChain,
                                      IMG_UINT32 ui32CKColour)
{
	/* Nothing to do */
	return PVRSRV_ERROR_NOT_SUPPORTED;
}

/*
 * Set display source colour key.
 * in: hDevice, hSwapChain, ui32CKColour
 */
static PVRSRV_ERROR SetDCSrcColourKey(IMG_HANDLE hDevice,
                                      IMG_HANDLE hSwapChain,
                                      IMG_UINT32 ui32CKColour)
{
	/* Nothing to do */
	return PVRSRV_ERROR_NOT_SUPPORTED;
}

/*
 * Closes the display.
 * in: hDevice
 */
static PVRSRV_ERROR CloseDCDevice(IMG_HANDLE hDevice)
{
	/* Nothing to do */
	return PVRSRV_OK;
}

/*
 * Flushes the sync queue present in the specified swap chain.
 * in: psSwapChain
 */
static void FlushInternalSyncQueue(OMAPLFB_SWAPCHAIN *psSwapChain)
{
#ifdef DEBUG
	OMAPLFB_DEVINFO	*psDevInfo = (OMAPLFB_DEVINFO *) psSwapChain->pvDevInfo;
#endif
	OMAPLFB_FLIP_ITEM *psFlipItem;
	unsigned long            ulMaxIndex;
	unsigned long            i;
	
	psFlipItem = &psSwapChain->psFlipItems[psSwapChain->ulRemoveIndex];
	ulMaxIndex = psSwapChain->ulBufferCount - 1;

#ifdef DEBUG
	DEBUG_PRINTK("Flushing sync queue on display %u",
		psDevInfo->uDeviceID);
#endif
	for(i = 0; i < psSwapChain->ulBufferCount; i++)
	{
		if (psFlipItem->bValid == OMAP_FALSE)
			continue;

		DEBUG_PRINTK("Flushing swap buffer index %lu",
			psSwapChain->ulRemoveIndex);

		/* Flip the buffer if it hasn't been flipped */
		if(psFlipItem->bFlipped == OMAP_FALSE)
		{
			OMAPLFBFlip(psSwapChain,
				(unsigned long)psFlipItem->sSysAddr);
		}

		/* If the command didn't complete, assume it did */
		if(psFlipItem->bCmdCompleted == OMAP_FALSE)
		{
			DEBUG_PRINTK("Calling command complete for swap "
				"buffer index %lu",
				psSwapChain->ulRemoveIndex);
			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(
				(IMG_HANDLE)psFlipItem->hCmdComplete,
				IMG_TRUE);
		}
		
		psSwapChain->ulRemoveIndex++;
		if(psSwapChain->ulRemoveIndex > ulMaxIndex)
			psSwapChain->ulRemoveIndex = 0;
		
		/* Put the state of the buffer to be used again later */
		psFlipItem->bFlipped = OMAP_FALSE;
		psFlipItem->bCmdCompleted = OMAP_FALSE;
		psFlipItem->bValid = OMAP_FALSE;
		psFlipItem =
			&psSwapChain->psFlipItems[psSwapChain->ulRemoveIndex];
	}

	psSwapChain->ulInsertIndex = 0;
	psSwapChain->ulRemoveIndex = 0;
}

/*
 * Sets the flush state of the specified display device
 * at the swap chain level without blocking the call.
 * in: psDevInfo, bFlushState
 */
static void SetFlushStateInternalNoLock(OMAPLFB_DEVINFO* psDevInfo,
                                        OMAP_BOOL bFlushState)
{
	OMAPLFB_SWAPCHAIN *psSwapChain = psDevInfo->psSwapChain;

	/* Nothing to do if there is no swap chain */
	if (psSwapChain == NULL){
		DEBUG_PRINTK("Swap chain is null, nothing to do for"
			" display %u", psDevInfo->uDeviceID);
		return;
	}

	if (bFlushState)
	{
		DEBUG_PRINTK("Desired flushState is true for display %u",
			psDevInfo->uDeviceID);
		if (psSwapChain->ulSetFlushStateRefCount == 0)
		{
			psSwapChain->bFlushCommands = OMAP_TRUE;
			FlushInternalSyncQueue(psSwapChain);
		}
		psSwapChain->ulSetFlushStateRefCount++;
	}
	else
	{
		DEBUG_PRINTK("Desired flushState is false for display %u",
			psDevInfo->uDeviceID);
		if (psSwapChain->ulSetFlushStateRefCount != 0)
		{
			psSwapChain->ulSetFlushStateRefCount--;
			if (psSwapChain->ulSetFlushStateRefCount == 0)
			{
				psSwapChain->bFlushCommands = OMAP_FALSE;
			}
		}
	}
}

/*
 * Sets the flush state of the specified display device
 * at the swap chain level blocking the call if needed.
 * in: psDevInfo, bFlushState
 */
static IMG_VOID SetFlushStateInternal(OMAPLFB_DEVINFO* psDevInfo,
                                      OMAP_BOOL bFlushState)
{
	DEBUG_PRINTK("Executing for display %u",
		psDevInfo->uDeviceID);
	mutex_lock(&psDevInfo->sSwapChainLockMutex);
	SetFlushStateInternalNoLock(psDevInfo, bFlushState);
	mutex_unlock(&psDevInfo->sSwapChainLockMutex);
}

/*
 * Sets the flush state of the specified display device
 * at device level blocking the call if needed.
 * in: psDevInfo, bFlushState
 */
static void SetFlushStateExternal(OMAPLFB_DEVINFO* psDevInfo,
                                  OMAP_BOOL bFlushState)
{
	DEBUG_PRINTK("Executing for display %u",
		psDevInfo->uDeviceID);
	mutex_lock(&psDevInfo->sSwapChainLockMutex);
	if (psDevInfo->bFlushCommands != bFlushState)
	{
		psDevInfo->bFlushCommands = bFlushState;
		SetFlushStateInternalNoLock(psDevInfo, bFlushState);
	}
	mutex_unlock(&psDevInfo->sSwapChainLockMutex);
}

/*
 * Unblank the framebuffer display
 * in: psDevInfo
 */
static OMAP_ERROR UnBlankDisplay(OMAPLFB_DEVINFO *psDevInfo)
{
	DEBUG_PRINTK("Executing for display %u",
		psDevInfo->uDeviceID);

	acquire_console_sem();
	if (fb_blank(psDevInfo->psLINFBInfo, FB_BLANK_UNBLANK))
	{
		release_console_sem();
		WARNING_PRINTK("fb_blank failed");
		return OMAP_ERROR_GENERIC;
	}
	release_console_sem();

	return OMAP_OK;
}

/*
 * Framebuffer listener
 * in: psNotif, event, data
 */
static int FrameBufferEvents(struct notifier_block *psNotif,
                             unsigned long event, void *data)
{
	OMAPLFB_DEVINFO *psDevInfo;
	OMAPLFB_SWAPCHAIN *psSwapChain;
	struct fb_event *psFBEvent = (struct fb_event *)data;
	OMAP_BOOL bBlanked;
	int i;

	DEBUG_PRINTK("Framebuffer event (%lu) happened", event);
	if (event != FB_EVENT_BLANK){
		DEBUG_PRINTK("Ignoring");
		return 0;
	}

	DEBUG_PRINTK("Event is FB_EVENT_BLANK");
	
	psDevInfo = 0;
	for(i = 0; i < FRAMEBUFFER_COUNT; i++)
	{
		if(psFBEvent->info == (&pDisplayDevices[i])->psLINFBInfo)
		{
			psDevInfo = &pDisplayDevices[i];
			break;
		}
	}

	if(!psDevInfo)
	{
		WARNING_PRINTK("Unable to find the display related to "
			" the framebuffer event");
		return 1;
	}

	psSwapChain = psDevInfo->psSwapChain;

	if(!psSwapChain)
	{
		DEBUG_PRINTK("No swapchain associated with this display");
		return 0;
	}

	bBlanked = (*(IMG_INT *)psFBEvent->data != 0) ?
		OMAP_TRUE: OMAP_FALSE;

	/* Check if the blank state is the same as the swap chain */
	if (bBlanked != psSwapChain->bBlanked)
	{
		DEBUG_PRINTK("Executing for display %u",
			psDevInfo->uDeviceID);

		/* Set the new blank state in the swap chain */
		psSwapChain->bBlanked = bBlanked;

		if (bBlanked)
		{
			DEBUG_PRINTK("Requesting flush state true for"
				" display %u", psDevInfo->uDeviceID);
			SetFlushStateInternal(psDevInfo, OMAP_TRUE);
		}
		else
		{
			DEBUG_PRINTK("Requesting flush state false for"
				" display %u", psDevInfo->uDeviceID);
			SetFlushStateInternal(psDevInfo, OMAP_FALSE);
		}
	}
	else
	{
		DEBUG_PRINTK("Ignoring event for display %u",
			psDevInfo->uDeviceID);
	}

	return 0;
}

/*
 * Registers a listener for changes in the framebuffer
 * in: psDevInfo
 */
static OMAP_ERROR EnableLFBEventNotification(OMAPLFB_DEVINFO *psDevInfo)
{
	OMAPLFB_SWAPCHAIN *psSwapChain = psDevInfo->psSwapChain;
	OMAP_ERROR         eError;
	
	memset(&psDevInfo->sLINNotifBlock, 0,
		sizeof(psDevInfo->sLINNotifBlock));

	/* Register the function to listen the changes */
	psDevInfo->sLINNotifBlock.notifier_call = FrameBufferEvents;
	psSwapChain->bBlanked = OMAP_FALSE;

	DEBUG_PRINTK("Registering framebuffer event listener for"
		" display %u", psDevInfo->uDeviceID);

	if (fb_register_client(&psDevInfo->sLINNotifBlock))
	{
		WARNING_PRINTK("fb_register_client failed for"
			" display %u", psDevInfo->uDeviceID);
		return OMAP_ERROR_GENERIC;
	}

	eError = UnBlankDisplay(psDevInfo);
	if (eError != OMAP_OK)
	{
		WARNING_PRINTK("UnBlankDisplay failed for"
			" display %u", psDevInfo->uDeviceID);
		return eError;
	}

	return OMAP_OK;
}

/*
 * Unregister a listener from the framebuffer
 * in: psDevInfo
 */
static OMAP_ERROR DisableLFBEventNotification(OMAPLFB_DEVINFO *psDevInfo)
{
	DEBUG_PRINTK("Removing framebuffer event listener for"
		" display %u", psDevInfo->uDeviceID);

	if (fb_unregister_client(&psDevInfo->sLINNotifBlock))
	{
		WARNING_PRINTK("fb_unregister_client failed for"
			" display %u", psDevInfo->uDeviceID);
		return OMAP_ERROR_GENERIC;
	}

	return OMAP_OK;
}

/*
 * Opens the display.
 * in: ui32DeviceID, phDevice
 * out: psSystemBufferSyncData
 */
static PVRSRV_ERROR OpenDCDevice(IMG_UINT32 ui32DeviceID,
                                 IMG_HANDLE *phDevice,
                                 PVRSRV_SYNC_DATA* psSystemBufferSyncData)
{
	OMAPLFB_DEVINFO *psDevInfo;
	int i;

	psDevInfo = 0;
	for(i = 0; i < FRAMEBUFFER_COUNT; i++)
	{
		if (ui32DeviceID == (&pDisplayDevices[i])->uDeviceID)
		{
			psDevInfo = &pDisplayDevices[i];
			break;
		}
	}

	if(!psDevInfo)
	{
		WARNING_PRINTK("Unable to identify display device with id %i",
			(int)ui32DeviceID);
		return 1;
	}

	psDevInfo->sSystemBuffer.psSyncData = psSystemBufferSyncData;
	if ( UnBlankDisplay(psDevInfo) != OMAP_OK)
	{
		WARNING_PRINTK("UnBlankDisplay failed for"
			" display %u", psDevInfo->uDeviceID);
		return PVRSRV_ERROR_UNBLANK_DISPLAY_FAILED;
	}
	*phDevice = (IMG_HANDLE)psDevInfo;

	return PVRSRV_OK;
}

/*
 * Gets the available formats for the display.
 * in: hDevice
 * out: pui32NumFormats, psFormat
 */
static PVRSRV_ERROR EnumDCFormats(IMG_HANDLE hDevice,
                                  IMG_UINT32 *pui32NumFormats,
                                  DISPLAY_FORMAT *psFormat)
{
	OMAPLFB_DEVINFO	*psDevInfo;
	if(!hDevice || !pui32NumFormats)
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	*pui32NumFormats = 1;
	
	if(psFormat)
		psFormat[0] = psDevInfo->sDisplayFormat;
	else
		WARNING_PRINTK("Display format is null for"
			" display %u", psDevInfo->uDeviceID);

	return PVRSRV_OK;
}

/*
 * Gets the available dimensions for the display.
 * in: hDevice, psFormat
 * out: pui32NumDims, psDim
 */
static PVRSRV_ERROR EnumDCDims(IMG_HANDLE hDevice, 
                               DISPLAY_FORMAT *psFormat,
                               IMG_UINT32 *pui32NumDims,
                               DISPLAY_DIMS *psDim)
{
	OMAPLFB_DEVINFO	*psDevInfo;
	if(!hDevice || !psFormat || !pui32NumDims)
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	*pui32NumDims = 1;
	
	if(psDim)
		psDim[0] = psDevInfo->sDisplayDim;
	else
		WARNING_PRINTK("Display dimensions are null for"
			" display %u", psDevInfo->uDeviceID);

	return PVRSRV_OK;
}

/*
 * Gets the display framebuffer physical address.
 * in: hDevice
 * out: phBuffer
 */
static PVRSRV_ERROR GetDCSystemBuffer(IMG_HANDLE hDevice, IMG_HANDLE *phBuffer)
{
	OMAPLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !phBuffer)
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	*phBuffer = (IMG_HANDLE)&psDevInfo->sSystemBuffer;

	return PVRSRV_OK;
}

/*
 * Gets the display general information.
 * in: hDevice
 * out: psDCInfo
 */
static PVRSRV_ERROR GetDCInfo(IMG_HANDLE hDevice, DISPLAY_INFO *psDCInfo)
{
	OMAPLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !psDCInfo)
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	*psDCInfo = psDevInfo->sDisplayInfo;

	return PVRSRV_OK;
}

/*
 * Gets the display framebuffer virtual address.
 * in: hDevice
 * out: ppsSysAddr, pui32ByteSize, ppvCpuVAddr, phOSMapInfo, pbIsContiguous
 */
static PVRSRV_ERROR GetDCBufferAddr(
				IMG_HANDLE        hDevice,
				IMG_HANDLE        hBuffer,
				IMG_SYS_PHYADDR   **ppsSysAddr,
				IMG_UINT32        *pui32ByteSize,
				IMG_VOID          **ppvCpuVAddr,
				IMG_HANDLE        *phOSMapInfo,
				IMG_BOOL          *pbIsContiguous,
				IMG_UINT32        *pui32TilingStride)
{
	OMAPLFB_DEVINFO	*psDevInfo;
	OMAPLFB_BUFFER *psSystemBuffer;

	if(!hDevice || !hBuffer || !ppsSysAddr || !pui32ByteSize )
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	psSystemBuffer = (OMAPLFB_BUFFER *)hBuffer;
	*ppsSysAddr = &psSystemBuffer->sSysAddr;
	*pui32ByteSize = (IMG_UINT32)psDevInfo->sFBInfo.ulBufferSize;

	if (ppvCpuVAddr)
		*ppvCpuVAddr = psSystemBuffer->sCPUVAddr;

	if (phOSMapInfo)
		*phOSMapInfo = (IMG_HANDLE)0;

	if (pbIsContiguous)
		*pbIsContiguous = IMG_TRUE;

	return PVRSRV_OK;
}

/*
 * Creates a swap chain. Called when a 3D application begins.
 * in: hDevice, ui32Flags, ui32BufferCount, psDstSurfAttrib, psSrcSurfAttrib
 * ui32OEMFlags
 * out: phSwapChain, ppsSyncData, pui32SwapChainID
 */
static PVRSRV_ERROR CreateDCSwapChain(IMG_HANDLE hDevice,
                                      IMG_UINT32 ui32Flags,
                                      DISPLAY_SURF_ATTRIBUTES *psDstSurfAttrib,
                                      DISPLAY_SURF_ATTRIBUTES *psSrcSurfAttrib,
                                      IMG_UINT32 ui32BufferCount,
                                      PVRSRV_SYNC_DATA **ppsSyncData,
                                      IMG_UINT32 ui32OEMFlags,
                                      IMG_HANDLE *phSwapChain,
                                      IMG_UINT32 *pui32SwapChainID)
{
	OMAPLFB_DEVINFO	*psDevInfo;
	OMAPLFB_SWAPCHAIN *psSwapChain;
	OMAPLFB_BUFFER *psBuffer;
	OMAPLFB_FLIP_ITEM *psFlipItems;
	IMG_UINT32 i;
	PVRSRV_ERROR eError = PVRSRV_OK;
	IMG_UINT32 ui32BuffersToSkip;
	
	if(!hDevice || !psDstSurfAttrib || !psSrcSurfAttrib ||
		!ppsSyncData || !phSwapChain)
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	
	if (psDevInfo->sDisplayInfo.ui32MaxSwapChains == 0)
	{
		ERROR_PRINTK("Unable to operate with 0 MaxSwapChains for"
			" display %u", psDevInfo->uDeviceID);
		return PVRSRV_ERROR_NOT_SUPPORTED;
	}

	if(psDevInfo->psSwapChain != NULL)
	{
		ERROR_PRINTK("Swap chain already exists for"
			" display %u", psDevInfo->uDeviceID);
		return PVRSRV_ERROR_FLIP_CHAIN_EXISTS;
	}

	if(ui32BufferCount > psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers)
	{
		ERROR_PRINTK("Too many buffers. Trying to use %u buffers while"
			" there is only %u available for display %u",
			(unsigned int)ui32BufferCount,
			(unsigned int)psDevInfo->
			sDisplayInfo.ui32MaxSwapChainBuffers,
			psDevInfo->uDeviceID);
		return PVRSRV_ERROR_TOOMANYBUFFERS;
	}


	if ((psDevInfo->sFBInfo.ulRoundedBufferSize *
		(unsigned long)ui32BufferCount) > psDevInfo->sFBInfo.ulFBSize)
	{
		ERROR_PRINTK("Too many buffers. Trying to use %u buffers "
			"(%lu bytes each) while there is only %lu memory for"
			" display %u",
			(unsigned int)ui32BufferCount,
			psDevInfo->sFBInfo.ulRoundedBufferSize,
			psDevInfo->sFBInfo.ulFBSize,
			psDevInfo->uDeviceID);
		return PVRSRV_ERROR_TOOMANYBUFFERS;
	}

	ui32BuffersToSkip = psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers -
		ui32BufferCount;

	if((psDstSurfAttrib->pixelformat !=
		psDevInfo->sDisplayFormat.pixelformat) ||
		(psDstSurfAttrib->sDims.ui32ByteStride !=
		psDevInfo->sDisplayDim.ui32ByteStride) ||
		(psDstSurfAttrib->sDims.ui32Width !=
		psDevInfo->sDisplayDim.ui32Width) ||
		(psDstSurfAttrib->sDims.ui32Height !=
		psDevInfo->sDisplayDim.ui32Height))
	{
		ERROR_PRINTK("Destination surface attributes differ from the"
			" current framebuffer for display %u",
			psDevInfo->uDeviceID);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}		

	if((psDstSurfAttrib->pixelformat !=
		psSrcSurfAttrib->pixelformat) ||
		(psDstSurfAttrib->sDims.ui32ByteStride !=
		psSrcSurfAttrib->sDims.ui32ByteStride) ||
		(psDstSurfAttrib->sDims.ui32Width !=
		psSrcSurfAttrib->sDims.ui32Width) ||
		(psDstSurfAttrib->sDims.ui32Height !=
		psSrcSurfAttrib->sDims.ui32Height))
	{
		ERROR_PRINTK("Destination surface attributes differ from the"
			" target destination surface for display %u",
			psDevInfo->uDeviceID);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}		

	/* Allocate memory needed for the swap chain */
	psSwapChain = (OMAPLFB_SWAPCHAIN*)OMAPLFBAllocKernelMem(
		sizeof(OMAPLFB_SWAPCHAIN));
	if(!psSwapChain)
	{
		ERROR_PRINTK("Out of memory to allocate swap chain for"
			" display %u", psDevInfo->uDeviceID);
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	DEBUG_PRINTK("Creating swap chain 0x%lx for display %u",
		(unsigned long)psSwapChain, psDevInfo->uDeviceID);

	/* Allocate memory for the buffer abstraction structures */
	psBuffer = (OMAPLFB_BUFFER*)OMAPLFBAllocKernelMem(
		sizeof(OMAPLFB_BUFFER) * ui32BufferCount);
	if(!psBuffer)
	{
		ERROR_PRINTK("Out of memory to allocate the buffer"
			" abstraction structures for display %u",
			psDevInfo->uDeviceID);
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorFreeSwapChain;
	}

	/* Allocate memory for the flip item abstraction structures */
	psFlipItems = (OMAPLFB_FLIP_ITEM *)OMAPLFBAllocKernelMem(
		sizeof(OMAPLFB_FLIP_ITEM) * ui32BufferCount);
	if (!psFlipItems)
	{
		ERROR_PRINTK("Out of memory to allocate the flip item"
			" abstraction structures for display %u",
			psDevInfo->uDeviceID);
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorFreeBuffers;
	}

	/* Assign to the swap chain structure the initial data */
	psSwapChain->ulBufferCount = (unsigned long)ui32BufferCount;
	psSwapChain->psBuffer = psBuffer;
	psSwapChain->psFlipItems = psFlipItems;
	psSwapChain->ulInsertIndex = 0;
	psSwapChain->ulRemoveIndex = 0;
	psSwapChain->psPVRJTable = &psDevInfo->sPVRJTable;
	psSwapChain->pvDevInfo = (void*)psDevInfo;

	/*
	 * Init the workqueue (single thread, freezable and real time)
	 * and its own work for this display
	 */
	INIT_WORK(&psDevInfo->sync_display_work, OMAPLFBSyncIHandler);
	psDevInfo->sync_display_wq =
		__create_workqueue("pvr_display_sync_wq", 1, 1, 1);

	DEBUG_PRINTK("Swap chain will have %u buffers for display %u",
		(unsigned int)ui32BufferCount, psDevInfo->uDeviceID);
	/* Link the buffers available like a circular list */
	for(i=0; i<ui32BufferCount-1; i++)
	{
		psBuffer[i].psNext = &psBuffer[i+1];
	}
	psBuffer[i].psNext = &psBuffer[0];

	/* Initialize each buffer abstraction structure */
	for(i=0; i<ui32BufferCount; i++)
	{
		IMG_UINT32 ui32SwapBuffer = i + ui32BuffersToSkip;
		IMG_UINT32 ui32BufferOffset = ui32SwapBuffer *
			(IMG_UINT32)psDevInfo->sFBInfo.ulRoundedBufferSize;
		psBuffer[i].psSyncData = ppsSyncData[i];
		psBuffer[i].sSysAddr.uiAddr =
			psDevInfo->sFBInfo.sSysAddr.uiAddr +
			ui32BufferOffset;
		psBuffer[i].sCPUVAddr = psDevInfo->sFBInfo.sCPUVAddr +
			ui32BufferOffset;
		DEBUG_PRINTK("Display %u buffer index %u has physical "
			"address 0x%x",
			psDevInfo->uDeviceID,
			(unsigned int)i,
			(unsigned int)psBuffer[i].sSysAddr.uiAddr);
	}

	/* Initialize each flip item abstraction structure */
	for(i=0; i<ui32BufferCount; i++)
	{
		psFlipItems[i].bValid = OMAP_FALSE;
		psFlipItems[i].bFlipped = OMAP_FALSE;
		psFlipItems[i].bCmdCompleted = OMAP_FALSE;
	}

	mutex_lock(&psDevInfo->sSwapChainLockMutex);
	
	psDevInfo->psSwapChain = psSwapChain;
	psSwapChain->bFlushCommands = psDevInfo->bFlushCommands;
	if (psSwapChain->bFlushCommands)
		psSwapChain->ulSetFlushStateRefCount = 1;
	else
		psSwapChain->ulSetFlushStateRefCount = 0;
		
	mutex_unlock(&psDevInfo->sSwapChainLockMutex);

	if (EnableLFBEventNotification(psDevInfo)!= OMAP_OK)
	{
		WARNING_PRINTK("Couldn't enable framebuffer event"
			" notification for display %u",
			psDevInfo->uDeviceID);
		goto ErrorUnRegisterDisplayClient;
	}
	
	*phSwapChain = (IMG_HANDLE)psSwapChain;

	return PVRSRV_OK;

ErrorUnRegisterDisplayClient:
	OMAPLFBFreeKernelMem(psFlipItems);
ErrorFreeBuffers:
	OMAPLFBFreeKernelMem(psBuffer);
ErrorFreeSwapChain:
	OMAPLFBFreeKernelMem(psSwapChain);

	return eError;
}

/*
 * Destroy a swap chain. Called when a 3D application ends.
 * in: hDevice, hSwapChain
 */
static PVRSRV_ERROR DestroyDCSwapChain(IMG_HANDLE hDevice,
	IMG_HANDLE hSwapChain)
{
	OMAPLFB_DEVINFO	*psDevInfo;
	OMAPLFB_SWAPCHAIN *psSwapChain;
	OMAP_ERROR eError;
	
	if(!hDevice || !hSwapChain)
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	
	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	psSwapChain = (OMAPLFB_SWAPCHAIN*)hSwapChain;

	if (psSwapChain != psDevInfo->psSwapChain)
	{
		ERROR_PRINTK("Swap chain handler differs from the one "
			"present in the display device pointer");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	DEBUG_PRINTK("Destroying swap chain for display %u",
		psDevInfo->uDeviceID);

	eError = DisableLFBEventNotification(psDevInfo);
	if (eError != OMAP_OK)
	{
		WARNING_PRINTK("Couldn't disable framebuffer event "
			"notification");
	}

	mutex_lock(&psDevInfo->sSwapChainLockMutex);

	FlushInternalSyncQueue(psSwapChain);

	/*
	 * Present the buffer which is at the base of address of
	 * the framebuffer
	 */
	OMAPLFBFlip(psSwapChain,
		(unsigned long)psDevInfo->sFBInfo.sSysAddr.uiAddr);
	psDevInfo->psSwapChain = NULL;

	mutex_unlock(&psDevInfo->sSwapChainLockMutex);

	/* Destroy the workqueue */
	flush_workqueue(psDevInfo->sync_display_wq);
	destroy_workqueue(psDevInfo->sync_display_wq);

	OMAPLFBFreeKernelMem(psSwapChain->psFlipItems);
	OMAPLFBFreeKernelMem(psSwapChain->psBuffer);
	OMAPLFBFreeKernelMem(psSwapChain);

	return PVRSRV_OK;
}


/*
 * Get display buffers. These are the buffers that can be allocated
 * inside the framebuffer memory.
 * in: hDevice, hSwapChain
 * out: pui32BufferCount, phBuffer
 */
static PVRSRV_ERROR GetDCBuffers(IMG_HANDLE hDevice,
                                 IMG_HANDLE hSwapChain,
                                 IMG_UINT32 *pui32BufferCount,
                                 IMG_HANDLE *phBuffer)
{
	OMAPLFB_DEVINFO   *psDevInfo;
	OMAPLFB_SWAPCHAIN *psSwapChain;
	unsigned long      i;

	if(!hDevice || !hSwapChain || !pui32BufferCount || !phBuffer)
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	
	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	psSwapChain = (OMAPLFB_SWAPCHAIN*)hSwapChain;
	if (psSwapChain != psDevInfo->psSwapChain)
	{
		ERROR_PRINTK("Swap chain handler differs from the one "
			"present in the display device %u pointer",
			psDevInfo->uDeviceID);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	*pui32BufferCount = (IMG_UINT32)psSwapChain->ulBufferCount;

	for(i=0; i<psSwapChain->ulBufferCount; i++)
		phBuffer[i] = (IMG_HANDLE)&psSwapChain->psBuffer[i];

	return PVRSRV_OK;
}

/*
 * Sets the display state.
 * in: ui32State, hDevice
 */
static IMG_VOID SetDCState(IMG_HANDLE hDevice, IMG_UINT32 ui32State)
{
	OMAPLFB_DEVINFO *psDevInfo = (OMAPLFB_DEVINFO *)hDevice;

	switch (ui32State)
	{
		case DC_STATE_FLUSH_COMMANDS:
			DEBUG_PRINTK("Setting state to flush commands for"
				" display %u", psDevInfo->uDeviceID);
			SetFlushStateExternal(psDevInfo, OMAP_TRUE);
			break;
		case DC_STATE_NO_FLUSH_COMMANDS:
			DEBUG_PRINTK("Setting state to not flush commands for"
				" display %u", psDevInfo->uDeviceID);
			SetFlushStateExternal(psDevInfo, OMAP_FALSE);
			break;
		default:
			WARNING_PRINTK("Unknown command state %u for display"
				" %u", (unsigned int)ui32State,
				psDevInfo->uDeviceID);
			break;
	}
}

/*
 * Swap to display system buffer. This buffer refers to the one which
 * is that fits in the framebuffer memory.
 * in: hDevice, hSwapChain
 */
static PVRSRV_ERROR SwapToDCSystem(IMG_HANDLE hDevice,
                                   IMG_HANDLE hSwapChain)
{
	OMAPLFB_DEVINFO   *psDevInfo;
	OMAPLFB_SWAPCHAIN *psSwapChain;

	if(!hDevice || !hSwapChain)
	{
		ERROR_PRINTK("Invalid parameters");
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (OMAPLFB_DEVINFO*)hDevice;
	psSwapChain = (OMAPLFB_SWAPCHAIN*)hSwapChain;

	DEBUG_PRINTK("Executing for display %u",
		psDevInfo->uDeviceID);

	if (psSwapChain != psDevInfo->psSwapChain)
	{
		ERROR_PRINTK("Swap chain handler differs from the one "
			"present in the display device %u pointer",
			psDevInfo->uDeviceID);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	
	mutex_lock(&psDevInfo->sSwapChainLockMutex);
	
	FlushInternalSyncQueue(psSwapChain);
	OMAPLFBFlip(psSwapChain,
		(unsigned long)psDevInfo->sFBInfo.sSysAddr.uiAddr);

	mutex_unlock(&psDevInfo->sSwapChainLockMutex);

	return PVRSRV_OK;
}

/*
 * Handles the synchronization with the display
 * in: work
 */
static void OMAPLFBSyncIHandler(struct work_struct *work)
{
	OMAPLFB_DEVINFO *psDevInfo = container_of(work, OMAPLFB_DEVINFO,
		sync_display_work);
	OMAPLFB_FLIP_ITEM *psFlipItem;
	OMAPLFB_SWAPCHAIN *psSwapChain;
	unsigned long ulMaxIndex;

	mutex_lock(&psDevInfo->sSwapChainLockMutex);

	psSwapChain = psDevInfo->psSwapChain;
	if (!psSwapChain || psSwapChain->bFlushCommands)
		goto ExitUnlock;

	psFlipItem = &psSwapChain->psFlipItems[psSwapChain->ulRemoveIndex];
	ulMaxIndex = psSwapChain->ulBufferCount - 1;

	/* Synchronize with the display */
	OMAPLFBWaitForSync(psDevInfo);

	/* Iterate through the flip items and flip them if necessary */
	while(psFlipItem->bValid)
	{	
		if(psFlipItem->bFlipped)
		{
			if(!psFlipItem->bCmdCompleted)
			{
				psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(
					(IMG_HANDLE)psFlipItem->hCmdComplete,
					IMG_TRUE);
				psFlipItem->bCmdCompleted = OMAP_TRUE;
			}
			psFlipItem->ulSwapInterval--;
			
			if(psFlipItem->ulSwapInterval == 0)
			{
				psSwapChain->ulRemoveIndex++;
				if(psSwapChain->ulRemoveIndex > ulMaxIndex)
					psSwapChain->ulRemoveIndex = 0;
				psFlipItem->bCmdCompleted = OMAP_FALSE;
				psFlipItem->bFlipped = OMAP_FALSE;
				psFlipItem->bValid = OMAP_FALSE;
			}
			else
			{
				/*
			 	 * Here the swap interval is not zero yet
			 	 * we need to schedule another work until
				 * it reaches zero
			 	 */
				queue_work(psDevInfo->sync_display_wq,
					&psDevInfo->sync_display_work);
				goto ExitUnlock;
			}
		}
		else
		{
			OMAPLFBFlip(psSwapChain,
				(unsigned long)psFlipItem->sSysAddr);
			psFlipItem->bFlipped = OMAP_TRUE;
			/*
			 * If the flip has been presented here then we need
			 * in the next sync execute the command complete,
			 * schedule another work
			 */
			queue_work(psDevInfo->sync_display_wq,
				&psDevInfo->sync_display_work);
			goto ExitUnlock;
		}
		psFlipItem =
			&psSwapChain->psFlipItems[psSwapChain->ulRemoveIndex];
	}
		
ExitUnlock:
	mutex_unlock(&psDevInfo->sSwapChainLockMutex);
}

/*
 * Performs a flip. This function takes the necessary steps to present
 * the buffer to be flipped in the display.
 * in: hCmdCookie, ui32DataSize, pvData
 */
static IMG_BOOL ProcessFlip(IMG_HANDLE  hCmdCookie,
                            IMG_UINT32  ui32DataSize,
                            IMG_VOID   *pvData)
{
	DISPLAYCLASS_FLIP_COMMAND *psFlipCmd;
	OMAPLFB_DEVINFO *psDevInfo;
	OMAPLFB_BUFFER *psBuffer;
	OMAPLFB_SWAPCHAIN *psSwapChain;
#if defined(SYS_USING_INTERRUPTS)
	OMAPLFB_FLIP_ITEM* psFlipItem;
#endif

	if(!hCmdCookie || !pvData)
	{
		WARNING_PRINTK("Ignoring call with NULL parameters");
		return IMG_FALSE;
	}
	
	psFlipCmd = (DISPLAYCLASS_FLIP_COMMAND*)pvData;

	if (psFlipCmd == IMG_NULL ||
		sizeof(DISPLAYCLASS_FLIP_COMMAND) != ui32DataSize)
	{
		WARNING_PRINTK("NULL command or command data size is wrong");
		return IMG_FALSE;
	}
	
	psDevInfo = (OMAPLFB_DEVINFO*)psFlipCmd->hExtDevice;
	psBuffer = (OMAPLFB_BUFFER*)psFlipCmd->hExtBuffer;
	psSwapChain = (OMAPLFB_SWAPCHAIN*) psFlipCmd->hExtSwapChain;

	mutex_lock(&psDevInfo->sSwapChainLockMutex);

	if (psDevInfo->bDeviceSuspended)
	{
		/* If is suspended then assume the commands are completed */
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(
			hCmdCookie, IMG_TRUE);
		goto ExitTrueUnlock;
	}

#if defined(SYS_USING_INTERRUPTS)

	if( psFlipCmd->ui32SwapInterval == 0 ||
		psSwapChain->bFlushCommands == OMAP_TRUE)
	{
#endif
		OMAPLFBFlip(psSwapChain,
			(unsigned long)psBuffer->sSysAddr.uiAddr);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(
			hCmdCookie, IMG_TRUE);

#if defined(SYS_USING_INTERRUPTS)
		goto ExitTrueUnlock;
	}

	psFlipItem = &psSwapChain->psFlipItems[psSwapChain->ulInsertIndex];

	if(psFlipItem->bValid == OMAP_FALSE)
	{
		unsigned long ulMaxIndex = psSwapChain->ulBufferCount - 1;

		/*
		 * If both indexes are equal the queue is empty,
		 * present immediatly
		 */
		if(psSwapChain->ulInsertIndex == psSwapChain->ulRemoveIndex)
		{
			OMAPLFBFlip(psSwapChain,
				(unsigned long)psBuffer->sSysAddr.uiAddr);
			psFlipItem->bFlipped = OMAP_TRUE;
		}
		else
			psFlipItem->bFlipped = OMAP_FALSE;

		/*
		 * The buffer is queued here, must be consumed by the workqueue
		 */
		psFlipItem->hCmdComplete = (OMAP_HANDLE)hCmdCookie;
		psFlipItem->ulSwapInterval =
			(unsigned long)psFlipCmd->ui32SwapInterval;
		psFlipItem->sSysAddr = &psBuffer->sSysAddr;
		psFlipItem->bValid = OMAP_TRUE;

		psSwapChain->ulInsertIndex++;
		if(psSwapChain->ulInsertIndex > ulMaxIndex)
			psSwapChain->ulInsertIndex = 0;

		/* Give work to the workqueue to sync with the display */
		queue_work(psDevInfo->sync_display_wq, &psDevInfo->sync_display_work);

		goto ExitTrueUnlock;
	}

	mutex_unlock(&psDevInfo->sSwapChainLockMutex);
	return IMG_FALSE;
#endif

ExitTrueUnlock:
	mutex_unlock(&psDevInfo->sSwapChainLockMutex);
	return IMG_TRUE;
}

#if defined(LDM_PLATFORM)

/*
 *  Function called when the driver must suspend
 */
void OMAPLFBDriverSuspend(void)
{
	OMAPLFB_DEVINFO *psDevInfo;
	int i;

	if(!pDisplayDevices)
		return;

	for(i = 0; i < FRAMEBUFFER_COUNT; i++)
	{
		psDevInfo = &pDisplayDevices[i];

		mutex_lock(&psDevInfo->sSwapChainLockMutex);

		if (psDevInfo->bDeviceSuspended)
		{
			mutex_unlock(&psDevInfo->sSwapChainLockMutex);
			continue;
		}

		psDevInfo->bDeviceSuspended = OMAP_TRUE;
		SetFlushStateInternalNoLock(psDevInfo, OMAP_TRUE);

		mutex_unlock(&psDevInfo->sSwapChainLockMutex);
	}
}

/*
 *  Function called when the driver must resume
 */
void OMAPLFBDriverResume(void)
{
	OMAPLFB_DEVINFO *psDevInfo;
	int i;

	if(!pDisplayDevices)
		return;

	for(i = 0; i < FRAMEBUFFER_COUNT; i++)
	{
		psDevInfo = &pDisplayDevices[i];

		mutex_lock(&psDevInfo->sSwapChainLockMutex);

		if (!psDevInfo->bDeviceSuspended)
		{
			mutex_unlock(&psDevInfo->sSwapChainLockMutex);
			continue;
		}

		SetFlushStateInternalNoLock(psDevInfo, OMAP_FALSE);
		psDevInfo->bDeviceSuspended = OMAP_FALSE;

		mutex_unlock(&psDevInfo->sSwapChainLockMutex);
	}
}
#endif /* defined(LDM_PLATFORM) */

/*
 * Frees the kernel framebuffer
 * in: psDevInfo
 */
static void DeInitDev(OMAPLFB_DEVINFO *psDevInfo)
{
	struct fb_info *psLINFBInfo = psDevInfo->psLINFBInfo;
	struct module *psLINFBOwner;

	acquire_console_sem();
	psLINFBOwner = psLINFBInfo->fbops->owner;

	if (psLINFBInfo->fbops->fb_release != NULL)
		(void) psLINFBInfo->fbops->fb_release(psLINFBInfo, 0);

	module_put(psLINFBOwner);

	release_console_sem();
}

/*
 *  Deinitialization routine for the 3rd party display driver
 */
OMAP_ERROR OMAPLFBDeinit(void)
{
	OMAPLFB_DEVINFO *psDevInfo;
	PVRSRV_DC_DISP2SRV_KMJTABLE *psJTable;
	int i;

	DEBUG_PRINTK("Deinitializing 3rd party display driver");

	if(!pDisplayDevices)
		return OMAP_OK;

	for(i = 0; i < FRAMEBUFFER_COUNT; i++)
	{
		psDevInfo = &pDisplayDevices[i];

		/* Remove the ProcessFlip command callback */
		psJTable = &psDevInfo->sPVRJTable;
		if (psDevInfo->sPVRJTable.pfnPVRSRVRemoveCmdProcList(
			psDevInfo->uDeviceID,
			OMAPLFB_COMMAND_COUNT) != PVRSRV_OK)
		{
			ERROR_PRINTK("Unable to remove callback for "
				"ProcessFlip command for display %u",
				psDevInfo->uDeviceID);
			return OMAP_ERROR_GENERIC;
		}

		/* Remove the display device from services */
		if (psJTable->pfnPVRSRVRemoveDCDevice(
			psDevInfo->uDeviceID) != PVRSRV_OK)
		{
			ERROR_PRINTK("Unable to remove the display %u "
				"from services", psDevInfo->uDeviceID);
			return OMAP_ERROR_GENERIC;
		}

		DeInitDev(psDevInfo);
	}

	OMAPLFBFreeKernelMem(pDisplayDevices);

	return OMAP_OK;
}

/*
 * Extracts the framebuffer data from the kernel driver
 * in: psDevInfo
 */
static OMAP_ERROR InitDev(OMAPLFB_DEVINFO *psDevInfo, int fb_idx)
{
	struct fb_info *psLINFBInfo;
	struct module *psLINFBOwner;
	OMAPLFB_FBINFO *psPVRFBInfo = &psDevInfo->sFBInfo;
	unsigned long FBSize;
	int buffers_available;

	/* Check if the framebuffer index to use is valid */
	if (fb_idx < 0 || fb_idx >= num_registered_fb)
	{
		ERROR_PRINTK("Framebuffer index %i out of range, "
			"only %i available", fb_idx, num_registered_fb);
		return OMAP_ERROR_INVALID_DEVICE;
	}

	/* Get the framebuffer from the kernel */
	if (!registered_fb[fb_idx])
	{
		ERROR_PRINTK("Framebuffer index %i is null", fb_idx);
		return OMAP_ERROR_INVALID_DEVICE;
	}

	psLINFBInfo = registered_fb[fb_idx];

	/* Check the framebuffer width and height are valid */
	if(psLINFBInfo->var.xres <= 0 || psLINFBInfo->var.yres <= 0)
	{
		ERROR_PRINTK("Framebuffer %i has an invalid state, "
			"width and height are %u,%u", fb_idx,
			psLINFBInfo->var.xres, psLINFBInfo->var.yres);
		return OMAP_ERROR_INVALID_DEVICE;
	}

	/* Configure framebuffer for flipping and desired bpp */
	psLINFBInfo->var.yres_virtual = psLINFBInfo->var.yres *
		MAX_BUFFERS_FLIPPING;

	if(DESIRED_BPP != 0){
		psLINFBInfo->var.bits_per_pixel = DESIRED_BPP;
		if(DESIRED_BPP == 16){
			psLINFBInfo->var.red.offset     = 11;
			psLINFBInfo->var.red.length     = 5;
			psLINFBInfo->var.green.offset   = 5;
			psLINFBInfo->var.green.length   = 6;
			psLINFBInfo->var.blue.offset    = 0;
			psLINFBInfo->var.blue.length    = 5;
			psLINFBInfo->var.transp.offset  = 0;
			psLINFBInfo->var.transp.length  = 0;
		}
		else if(DESIRED_BPP == 32)
		{
			psLINFBInfo->var.red.offset     = 16;
			psLINFBInfo->var.red.length     = 8;
			psLINFBInfo->var.green.offset   = 8;
			psLINFBInfo->var.green.length   = 8;
			psLINFBInfo->var.blue.offset    = 0;
			psLINFBInfo->var.blue.length    = 8;
			psLINFBInfo->var.transp.offset  = 0;
			psLINFBInfo->var.transp.length  = 0;
		}
		else
			WARNING_PRINTK("Unknown bits per pixel format %i",
				DESIRED_BPP);
	}
	acquire_console_sem();
	psLINFBInfo->var.activate = FB_ACTIVATE_FORCE;
	fb_set_var(psLINFBInfo, &psLINFBInfo->var);
	buffers_available =
		psLINFBInfo->var.yres_virtual / psLINFBInfo->var.yres;

	if(buffers_available <= 1)
	{
		/*
		 * Flipping is not supported, return the framebuffer to
		 * its original state
		 */
		psLINFBInfo->var.yres_virtual = psLINFBInfo->var.yres;
		psLINFBInfo->var.activate = FB_ACTIVATE_FORCE;
		fb_set_var(psLINFBInfo, &psLINFBInfo->var);
		buffers_available = 1;
	}
	psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers = buffers_available;

	psLINFBOwner = psLINFBInfo->fbops->owner;
	if (!try_module_get(psLINFBOwner))
	{
		ERROR_PRINTK("Couldn't get framebuffer module");
		release_console_sem();
		return OMAP_ERROR_GENERIC;
	}

	if (psLINFBInfo->fbops->fb_open != NULL)
	{
		if (psLINFBInfo->fbops->fb_open(psLINFBInfo, 0))
		{
			ERROR_PRINTK("Couldn't open framebuffer with"
				" index %d", fb_idx);
			module_put(psLINFBOwner);
			release_console_sem();
			return OMAP_ERROR_GENERIC;
		}
	}
	psDevInfo->psLINFBInfo = psLINFBInfo;

	/* Extract the needed data from the framebuffer structures */
	FBSize = (psLINFBInfo->screen_size) != 0 ?
		psLINFBInfo->screen_size : psLINFBInfo->fix.smem_len;
	DEBUG_PRINTK("Framebuffer index %d information:", fb_idx);
	DEBUG_PRINTK("*Physical address: 0x%lx",
		psLINFBInfo->fix.smem_start);
	DEBUG_PRINTK("*Virtual address: 0x%lx",
		(unsigned long)psLINFBInfo->screen_base);
	DEBUG_PRINTK("*Size (bytes): %lu",FBSize);
	DEBUG_PRINTK("*Width, height: %u,%u",
		psLINFBInfo->var.xres, psLINFBInfo->var.yres);
	DEBUG_PRINTK("*Virtual width, height: %u,%u",
		psLINFBInfo->var.xres_virtual, psLINFBInfo->var.yres_virtual);
	DEBUG_PRINTK("*Rotation: %u", psLINFBInfo->var.rotate);
	DEBUG_PRINTK("*Stride (bytes): %u",
		(unsigned int)psLINFBInfo->fix.line_length);

	psPVRFBInfo->sSysAddr.uiAddr = psLINFBInfo->fix.smem_start;
	psPVRFBInfo->sCPUVAddr = psLINFBInfo->screen_base;
	psPVRFBInfo->ulWidth = psLINFBInfo->var.xres;
	psPVRFBInfo->ulHeight = psLINFBInfo->var.yres;
	psPVRFBInfo->ulByteStride = psLINFBInfo->fix.line_length;
	psPVRFBInfo->ulFBSize = FBSize;
	psPVRFBInfo->ulBufferSize =
		psPVRFBInfo->ulHeight * psPVRFBInfo->ulByteStride;

	/* XXX: Page aligning with 16bpp causes the
	 * position of framebuffer address to look in the wrong place.
	 */
	psPVRFBInfo->ulRoundedBufferSize =
			OMAPLFB_PAGE_ROUNDUP(psPVRFBInfo->ulBufferSize);

	psDevInfo->sFBInfo.sSysAddr.uiAddr = psPVRFBInfo->sSysAddr.uiAddr;
	psDevInfo->sFBInfo.sCPUVAddr = psPVRFBInfo->sCPUVAddr;

	/* Get the current bits per pixel configured in the framebuffer */
	DEBUG_PRINTK("*Bits per pixel: %d", psLINFBInfo->var.bits_per_pixel);
	if(psLINFBInfo->var.bits_per_pixel == 16)
	{
		if((psLINFBInfo->var.red.length == 5) &&
			(psLINFBInfo->var.green.length == 6) && 
			(psLINFBInfo->var.blue.length == 5) && 
			(psLINFBInfo->var.red.offset == 11) &&
			(psLINFBInfo->var.green.offset == 5) && 
			(psLINFBInfo->var.blue.offset == 0) && 
			(psLINFBInfo->var.red.msb_right == 0))
		{
			DEBUG_PRINTK("*Format: RGB565");
			psPVRFBInfo->ePixelFormat = PVRSRV_PIXEL_FORMAT_RGB565;
		}
		else
			WARNING_PRINTK("*Format: Unknown framebuffer"
				" format");
	}
	else if(psLINFBInfo->var.bits_per_pixel == 32)
	{
		if((psLINFBInfo->var.red.length == 8) &&
			(psLINFBInfo->var.green.length == 8) && 
			(psLINFBInfo->var.blue.length == 8) && 
			(psLINFBInfo->var.red.offset == 16) &&
			(psLINFBInfo->var.green.offset == 8) && 
			(psLINFBInfo->var.blue.offset == 0) && 
			(psLINFBInfo->var.red.msb_right == 0))
		{
			psPVRFBInfo->ePixelFormat =
				PVRSRV_PIXEL_FORMAT_ARGB8888;
			DEBUG_PRINTK("*Format: ARGB8888");
		}
		else
			WARNING_PRINTK("*Format: Unknown framebuffer"
				"format");
	}	
	else
		WARNING_PRINTK("*Format: Unknown framebuffer format");

	release_console_sem();
	return OMAP_OK;
}

/*
 *  Initialization routine for the 3rd party display driver
 */
OMAP_ERROR OMAPLFBInit(void)
{
	OMAPLFB_DEVINFO *psDevInfo;
	PFN_CMD_PROC pfnCmdProcList[OMAPLFB_COMMAND_COUNT];
	IMG_UINT32 aui32SyncCountList[OMAPLFB_COMMAND_COUNT][2];
	int i;

	DEBUG_PRINTK("Initializing 3rd party display driver");
	DEBUG_PRINTK("Found %u framebuffers", FRAMEBUFFER_COUNT);

#if defined(REQUIRES_TWO_FRAMEBUFFERS)
	/*
	 * Fail hard if there isn't at least two framebuffers available
	 */
	if(FRAMEBUFFER_COUNT < 2)
	{
		ERROR_PRINTK("Driver needs at least two framebuffers");
		return OMAP_ERROR_INIT_FAILURE;
	}
#endif

	/*
	 * Obtain the function pointer for the jump table from
	 * services to fill it with the function pointers that we want
	 */
	if(OMAPLFBGetLibFuncAddr ("PVRGetDisplayClassJTable",
		&pfnGetPVRJTable) != OMAP_OK)
	{
		ERROR_PRINTK("Unable to get the function to get the"
			" jump table display->services");
		return OMAP_ERROR_INIT_FAILURE;
	}

	/*
	 * Allocate the display device structures, one per framebuffer
	 */
	pDisplayDevices = (OMAPLFB_DEVINFO *)OMAPLFBAllocKernelMem(
			sizeof(OMAPLFB_DEVINFO) * FRAMEBUFFER_COUNT);
	if(!pDisplayDevices)
	{
		pDisplayDevices = NULL;
		ERROR_PRINTK("Out of memory");
		return OMAP_ERROR_OUT_OF_MEMORY;
	}
	memset(pDisplayDevices, 0, sizeof(OMAPLFB_DEVINFO) *
		FRAMEBUFFER_COUNT);

	/*
	 * Initialize each display device
	 */
	for (i = FRAMEBUFFER_COUNT - 1; i >= 0; i--)
	{
		DEBUG_PRINTK("-> Initializing display device %i", i);
		/*
		 * Here we get the framebuffer data for each display device
		 * and check for error
		 */
		if(InitDev(&pDisplayDevices[i], i) != OMAP_OK)
		{
			ERROR_PRINTK("Unable to initialize display "
				"device %i",i);
			OMAPLFBFreeKernelMem(pDisplayDevices);
			pDisplayDevices = NULL;
			return OMAP_ERROR_INIT_FAILURE;
		}

    		/*
		 * Populate each display device structure
		*/
		DEBUG_PRINTK("-> Populating display device %i", i);
		psDevInfo = &pDisplayDevices[i];

		if(!(*pfnGetPVRJTable)(&psDevInfo->sPVRJTable))
		{
			ERROR_PRINTK("Unable to get the jump table"
				" display->services");
			return OMAP_ERROR_INIT_FAILURE;
		}

		mutex_init(&psDevInfo->sSwapChainLockMutex);

		psDevInfo->psSwapChain = 0;
		psDevInfo->bFlushCommands = OMAP_FALSE;
		psDevInfo->bDeviceSuspended = OMAP_FALSE;

		if(psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers > 1)
		{
			if(MAX_BUFFERS_FLIPPING == 1)
			{
				DEBUG_PRINTK("Flipping support is possible"
					" but you decided not to use it, "
					"no swap chain will be created");
			}

			DEBUG_PRINTK("Flipping support");
			if(psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers >
				MAX_BUFFERS_FLIPPING)
			psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers =
				MAX_BUFFERS_FLIPPING;
		}
		else
		{
			DEBUG_PRINTK("Flipping not supported, no swap chain"
				" will be created");
		}

		if (psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers == 0)
		{
			psDevInfo->sDisplayInfo.ui32MaxSwapChains = 0;
			psDevInfo->sDisplayInfo.ui32MaxSwapInterval = 0;
		}
		else
		{
			psDevInfo->sDisplayInfo.ui32MaxSwapChains = 1;
			psDevInfo->sDisplayInfo.ui32MaxSwapInterval = 3;
		}
		psDevInfo->sDisplayInfo.ui32MinSwapInterval = 0;

		/* Get the display and framebuffer needed info */
		strncpy(psDevInfo->sDisplayInfo.szDisplayName,
			DISPLAY_DEVICE_NAME, MAX_DISPLAY_NAME_SIZE);
		psDevInfo->sDisplayFormat.pixelformat =
			psDevInfo->sFBInfo.ePixelFormat;
		psDevInfo->sDisplayDim.ui32Width =
			(IMG_UINT32)psDevInfo->sFBInfo.ulWidth;
		psDevInfo->sDisplayDim.ui32Height =
			(IMG_UINT32)psDevInfo->sFBInfo.ulHeight;
		psDevInfo->sDisplayDim.ui32ByteStride =
			(IMG_UINT32)psDevInfo->sFBInfo.ulByteStride;
		psDevInfo->sSystemBuffer.sSysAddr =
			psDevInfo->sFBInfo.sSysAddr;
		psDevInfo->sSystemBuffer.sCPUVAddr =
			psDevInfo->sFBInfo.sCPUVAddr;
		psDevInfo->sSystemBuffer.ulBufferSize =
			psDevInfo->sFBInfo.ulRoundedBufferSize;
		DEBUG_PRINTK("Buffers available: %u (%lu bytes per buffer)",
			psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers,
			psDevInfo->sFBInfo.ulBufferSize);

		/* Populate the function table that services will use */
		psDevInfo->sDCJTable.ui32TableSize =
			sizeof(PVRSRV_DC_SRV2DISP_KMJTABLE);
		psDevInfo->sDCJTable.pfnOpenDCDevice = OpenDCDevice;
		psDevInfo->sDCJTable.pfnCloseDCDevice = CloseDCDevice;
		psDevInfo->sDCJTable.pfnEnumDCFormats = EnumDCFormats;
		psDevInfo->sDCJTable.pfnEnumDCDims = EnumDCDims;
		psDevInfo->sDCJTable.pfnGetDCSystemBuffer = GetDCSystemBuffer;
		psDevInfo->sDCJTable.pfnGetDCInfo = GetDCInfo;
		psDevInfo->sDCJTable.pfnGetBufferAddr = GetDCBufferAddr;
		psDevInfo->sDCJTable.pfnCreateDCSwapChain = CreateDCSwapChain;
		psDevInfo->sDCJTable.pfnDestroyDCSwapChain =
			DestroyDCSwapChain;
		psDevInfo->sDCJTable.pfnSetDCDstRect = SetDCDstRect;
		psDevInfo->sDCJTable.pfnSetDCSrcRect = SetDCSrcRect;
		psDevInfo->sDCJTable.pfnSetDCDstColourKey = SetDCDstColourKey;
		psDevInfo->sDCJTable.pfnSetDCSrcColourKey = SetDCSrcColourKey;
		psDevInfo->sDCJTable.pfnGetDCBuffers = GetDCBuffers;
		psDevInfo->sDCJTable.pfnSwapToDCBuffer = SwapToDCBuffer;
		psDevInfo->sDCJTable.pfnSwapToDCSystem = SwapToDCSystem;
		psDevInfo->sDCJTable.pfnSetDCState = SetDCState;

		/* Register the display device */
		if(psDevInfo->sPVRJTable.pfnPVRSRVRegisterDCDevice(
			&psDevInfo->sDCJTable,
			&psDevInfo->uDeviceID) != PVRSRV_OK)
		{
			ERROR_PRINTK("Unable to register the jump table"
				" services->display");
			return OMAP_ERROR_DEVICE_REGISTER_FAILED;
		}

		DEBUG_PRINTK("Display device %i registered with id %u",
			i, psDevInfo->uDeviceID);

		/*
		 * Register the ProcessFlip function to notify when a frame is
		 * ready to be flipped
		 */
		pfnCmdProcList[DC_FLIP_COMMAND] = ProcessFlip;
		aui32SyncCountList[DC_FLIP_COMMAND][0] = 0;
		aui32SyncCountList[DC_FLIP_COMMAND][1] = 2;
		if (psDevInfo->sPVRJTable.pfnPVRSRVRegisterCmdProcList(
			psDevInfo->uDeviceID, &pfnCmdProcList[0],
			aui32SyncCountList, OMAPLFB_COMMAND_COUNT) != PVRSRV_OK)
		{
			ERROR_PRINTK("Unable to register callback for "
				"ProcessFlip command");
			return OMAP_ERROR_CANT_REGISTER_CALLBACK;
		}

	}
	return OMAP_OK;
}

