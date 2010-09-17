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

#ifndef __OMAPLFB_H__
#define __OMAPLFB_H__

extern IMG_BOOL PVRGetDisplayClassJTable(PVRSRV_DC_DISP2SRV_KMJTABLE *psJTable);

typedef void * OMAP_HANDLE;

typedef enum tag_omap_bool
{
	OMAP_FALSE = 0,
	OMAP_TRUE  = 1,
} OMAP_BOOL, *OMAP_PBOOL;

typedef struct OMAPLFB_BUFFER_TAG
{
	unsigned long                ulBufferSize;
	IMG_SYS_PHYADDR              sSysAddr;
	IMG_CPU_VIRTADDR             sCPUVAddr;
	PVRSRV_SYNC_DATA*            psSyncData;
	struct OMAPLFB_BUFFER_TAG*   psNext;

} OMAPLFB_BUFFER;

typedef struct OMAPLFB_FLIP_ITEM_TAG
{
	OMAP_HANDLE      hCmdComplete;
	unsigned long    ulSwapInterval;
	OMAP_BOOL        bValid;
	OMAP_BOOL        bFlipped;
	OMAP_BOOL        bCmdCompleted;
	IMG_SYS_PHYADDR* sSysAddr;

} OMAPLFB_FLIP_ITEM;

typedef struct PVRPDP_SWAPCHAIN_TAG
{
	unsigned long                   ulBufferCount;
	OMAPLFB_BUFFER*                 psBuffer;
	OMAPLFB_FLIP_ITEM*              psFlipItems;
	unsigned long                   ulInsertIndex;
	unsigned long                   ulRemoveIndex;
	PVRSRV_DC_DISP2SRV_KMJTABLE*	psPVRJTable;
	OMAP_BOOL                       bFlushCommands;
	unsigned long                   ulSetFlushStateRefCount;
	OMAP_BOOL                       bBlanked;
	spinlock_t*                     psSwapChainLock;
	void*                           pvDevInfo;

} OMAPLFB_SWAPCHAIN;

typedef struct OMAPLFB_FBINFO_TAG
{
	unsigned long       ulFBSize;
	unsigned long       ulBufferSize;
	unsigned long       ulRoundedBufferSize;
	unsigned long       ulWidth;
	unsigned long       ulHeight;
	unsigned long       ulByteStride;
	IMG_SYS_PHYADDR     sSysAddr;
	IMG_CPU_VIRTADDR    sCPUVAddr;
	PVRSRV_PIXEL_FORMAT ePixelFormat;

}OMAPLFB_FBINFO;

typedef struct OMAPLFB_DEVINFO_TAG
{
	IMG_UINT32                      uDeviceID;
	OMAPLFB_BUFFER                  sSystemBuffer;
	PVRSRV_DC_DISP2SRV_KMJTABLE	sPVRJTable;
	PVRSRV_DC_SRV2DISP_KMJTABLE	sDCJTable;
	OMAPLFB_FBINFO                  sFBInfo;
	OMAPLFB_SWAPCHAIN*              psSwapChain;
	OMAP_BOOL                       bFlushCommands;
	struct fb_info*                 psLINFBInfo;
	struct notifier_block           sLINNotifBlock;
	OMAP_BOOL                       bDeviceSuspended;
	struct mutex                    sSwapChainLockMutex;
	IMG_DEV_VIRTADDR	        sDisplayDevVAddr;
	DISPLAY_INFO                    sDisplayInfo;
	DISPLAY_FORMAT                  sDisplayFormat;
	DISPLAY_DIMS                    sDisplayDim;
	struct workqueue_struct*        sync_display_wq;
	struct work_struct	        sync_display_work;

}  OMAPLFB_DEVINFO;

typedef enum _OMAP_ERROR_
{
	OMAP_OK                             =  0,
	OMAP_ERROR_GENERIC                  =  1,
	OMAP_ERROR_OUT_OF_MEMORY            =  2,
	OMAP_ERROR_TOO_FEW_BUFFERS          =  3,
	OMAP_ERROR_INVALID_PARAMS           =  4,
	OMAP_ERROR_INIT_FAILURE             =  5,
	OMAP_ERROR_CANT_REGISTER_CALLBACK   =  6,
	OMAP_ERROR_INVALID_DEVICE           =  7,
	OMAP_ERROR_DEVICE_REGISTER_FAILED   =  8

} OMAP_ERROR;

#define	OMAPLFB_PAGE_SIZE 4096
#define	OMAPLFB_PAGE_MASK (OMAPLFB_PAGE_SIZE - 1)
#define	OMAPLFB_PAGE_TRUNC (~OMAPLFB_PAGE_MASK)

#define	OMAPLFB_PAGE_ROUNDUP(x) (((x)+OMAPLFB_PAGE_MASK) & OMAPLFB_PAGE_TRUNC)

#define DISPLAY_DEVICE_NAME "PowerVR OMAP Linux Display Driver"
#define	DRVNAME	"omaplfb"
#define	DEVNAME	DRVNAME
#define	DRIVER_PREFIX DRVNAME

#define FRAMEBUFFER_COUNT		num_registered_fb

#ifdef	DEBUG
#define	DEBUG_PRINTK(format, ...) printk("DEBUG " DRIVER_PREFIX \
	" (%s %i): " format "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define	DEBUG_PRINTK(format,...)
#endif

#define	WARNING_PRINTK(format, ...) printk("WARNING " DRIVER_PREFIX \
	" (%s %i): " format "\n", __func__, __LINE__, ## __VA_ARGS__)
#define	ERROR_PRINTK(format, ...) printk("ERROR " DRIVER_PREFIX \
	" (%s %i): " format "\n", __func__, __LINE__, ## __VA_ARGS__)

OMAP_ERROR OMAPLFBInit(void);
OMAP_ERROR OMAPLFBDeinit(void);
void *OMAPLFBAllocKernelMem(unsigned long ulSize);
void OMAPLFBFreeKernelMem(void *pvMem);
void OMAPLFBWaitForSync(OMAPLFB_DEVINFO *psDevInfo);
OMAP_ERROR OMAPLFBGetLibFuncAddr(char *szFunctionName,
	PFN_DC_GET_PVRJTABLE *ppfnFuncTable);
void OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long aPhyAddr);
#ifdef LDM_PLATFORM
void OMAPLFBDriverSuspend(void);
void OMAPLFBDriverResume(void);
#endif

#endif

