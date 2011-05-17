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

#ifndef __SCXLNX_DEFS_H__
#define __SCXLNX_DEFS_H__

#include <asm/atomic.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/sysdev.h>
#include <linux/sysfs.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#include "scx_protocol.h"

/*----------------------------------------------------------------------------*/

#define SIZE_1KB 0x400

/*
 * Maximum number of shared memory blocks that can be reigsters in a connection
 */
#define SCXLNX_SHMEM_MAX_COUNT   (64)

/*
 * Describes the possible types of shared memories
 *
 * SCXLNX_SHMEM_TYPE_PREALLOC_REGISTERED_SHMEM :
 *    The descriptor describes a registered shared memory.
 *    Its coarse pages are preallocated when initializing the
 *    connection
 * SCXLNX_SHMEM_TYPE_REGISTERED_SHMEM :
 *    The descriptor describes a registered shared memory.
 *    Its coarse pages are not preallocated
 * SCXLNX_SHMEM_TYPE_PM_HIBERNATE :
 *    The descriptor describes a power management shared memory.
 */
enum SCXLNX_SHMEM_TYPE {
	SCXLNX_SHMEM_TYPE_PREALLOC_REGISTERED_SHMEM = 0,
	SCXLNX_SHMEM_TYPE_REGISTERED_SHMEM,
	SCXLNX_SHMEM_TYPE_PM_HIBERNATE,
};


/*
 * This structure contains a pointer on a coarse page table
 */
struct SCXLNX_COARSE_PAGE_TABLE {
	/*
	 * Identifies the coarse page table descriptor in
	 * sFreeCoarsePageTables list
	 */
	struct list_head list;

	/*
	 * The address of the coarse page table
	 */
	u32 *pDescriptors;

	/*
	 * The address of the array containing this coarse page table
	 */
	struct SCXLNX_COARSE_PAGE_TABLE_ARRAY *pParent;
};


#define SCXLNX_PAGE_DESCRIPTOR_TYPE_NORMAL       0
#define SCXLNX_PAGE_DESCRIPTOR_TYPE_PREALLOCATED 1

/*
 * This structure describes an array of up to 4 coarse page tables
 * allocated within a single 4KB page.
 */
struct SCXLNX_COARSE_PAGE_TABLE_ARRAY {
	/*
	 * identifies the element in the sCoarsePageTableArrays list
	 */
	struct list_head list;

	/*
	 * Type of page descriptor
	 * can take any of SCXLNX_PAGE_DESCRIPTOR_TYPE_XXX value
	 */
	u32 nType;

	struct SCXLNX_COARSE_PAGE_TABLE sCoarsePageTables[4];

	/*
	 * A counter of the number of coarse pages currently used
	 * the max value should be 4 (one coarse page table is 1KB while one
	 * page is 4KB)
	 */
	u8 nReferenceCount;
};


/*
 * This structure describes a list of coarse page table arrays
 * with some of the coarse page tables free. It is used
 * when the driver needs to allocate a new coarse page
 * table.
 */
struct SCXLNX_COARSE_PAGE_TABLE_ALLOCATION_CONTEXT {
	/*
	 * The spin lock protecting concurrent access to the structure.
	 */
	spinlock_t lock;

	/*
	 * The list of allocated coarse page table arrays
	 */
	struct list_head sCoarsePageTableArrays;

	/*
	 * The list of free coarse page tables
	 */
	struct list_head sFreeCoarsePageTables;
};


/*
 * Fully describes a shared memory block
 */
struct SCXLNX_SHMEM_DESC {
	/*
	 * Identifies the shared memory descriptor in the list of free shared
	 * memory descriptors
	 */
	struct list_head list;

	/*
	 * Identifies the type of shared memory descriptor
	 */
	enum SCXLNX_SHMEM_TYPE nType;

	/*
	 * The identifier of the block of shared memory, as returned by the
	 * Secure World.
	 * This identifier is hBlock field of a REGISTER_SHARED_MEMORY answer
	 */
	u32 hIdentifier;

	/* Client buffer */
	u8 *pBuffer;

	/* Up to eight coarse page table context */
	struct SCXLNX_COARSE_PAGE_TABLE *pCoarsePageTable[SCX_MAX_COARSE_PAGES];

	u32 nNumberOfCoarsePageTables;

	/* Reference counter */
	atomic_t nRefCnt;
};


/*----------------------------------------------------------------------------*/

/*
 * This structure describes the communication with the Secure World
 *
 * Note that this driver supports only one instance of the Secure World
 */
struct SCXLNX_COMM {
	/*
	 * The spin lock protecting concurrent access to the structure.
	 */
	spinlock_t lock;

	/*
	 * Bit vector with the following possible flags:
	 *    - SCXLNX_COMM_FLAG_IRQ_REQUESTED: If set, indicates that
	 *      the IRQ has been successfuly requested.
	 *    - SCXLNX_COMM_FLAG_TERMINATING: If set, indicates that the
	 *      communication with the Secure World is being terminated.
	 *      Transmissions to the Secure World are not permitted
	 *    - SCXLNX_COMM_FLAG_W3B_ALLOCATED: If set, indicates that the
	 *      W3B buffer has been allocated.
	 *
	 * This bit vector must be accessed with the kernel's atomic bitwise
	 * operations.
	 */
	unsigned long nFlags;

	/*
	 * The virtual address of the L1 shared buffer.
	 */
	struct SCHANNEL_C1S_BUFFER *pBuffer;

	/*
	 * The wait queue the client threads are waiting on.
	 */
	wait_queue_head_t waitQueue;

#ifdef CONFIG_TF_TRUSTZONE
	/*
	 * The interrupt line used by the Secure World.
	 */
	int nSoftIntIrq;

	/* ----- W3B ----- */
	/* shared memory descriptor to identify the W3B */
	struct SCXLNX_SHMEM_DESC sW3BShmemDesc;

	/* Virtual address of the kernel allocated shared memory */
	u32 nW3BShmemVAddr;

	/* offset of data in shared memory coarse pages */
	u32 nW3BShmemOffset;

	u32 nW3BShmemSize;

	struct SCXLNX_COARSE_PAGE_TABLE_ALLOCATION_CONTEXT
		sW3BAllocationContext;
#endif
#ifdef CONFIG_TF_MSHIELD
	/*
	 * The SE SDP can only be initialized once...
	 */
	int bSEInitialized;

	/* Virtual address of the L0 communication buffer */
	void *pInitSharedBuffer;

	/*
	 * Lock to be held by a client when executing an RPC
	 */
	struct mutex sRPCLock;

	/*
	 * Lock to protect concurrent accesses to DMA channels
	 */
	struct mutex sDMALock;
#endif
};


#define SCXLNX_COMM_FLAG_IRQ_REQUESTED		(0)
#define SCXLNX_COMM_FLAG_PA_AVAILABLE		(1)
#define SCXLNX_COMM_FLAG_TERMINATING		(2)
#define SCXLNX_COMM_FLAG_W3B_ALLOCATED		(3)
#define SCXLNX_COMM_FLAG_L1_SHARED_ALLOCATED	(4)

/*----------------------------------------------------------------------------*/

struct SCXLNX_DEVICE_STATS {
	struct kobject kobj;

	struct kobj_type kobj_type;

	struct attribute kobj_stat_attribute;

	struct attribute *kobj_attribute_list[2];

	atomic_t stat_pages_allocated;
	atomic_t stat_memories_allocated;
	atomic_t stat_pages_locked;
};

/*
 * This structure describes the information about one device handled by the
 * driver. Note that the driver supports only a single device. see the global
 * variable g_SCXLNXDevice
 */
struct SCXLNX_DEVICE {
	/*
	 * The device number for the device.
	 */
	dev_t nDevNum;

	/*
	 * Interfaces the system device with the kernel.
	 */
	struct sys_device sysdev;

	/*
	 * Interfaces the char device with the kernel.
	 */
	struct cdev cdev;

#ifdef CONFIG_TF_MSHIELD
	struct cdev cdev_ctrl;

	/*
	 * Globals for CUS
	 */
	/* Current key handles loaded in HWAs */
	u32 hAES1SecureKeyContext;
	u32 hAES2SecureKeyContext;
	u32 hDESSecureKeyContext;
	bool bSHAM1IsPublic;

	/* Semaphores used to serialize HWA accesses */
	struct semaphore sAES1CriticalSection;
	struct semaphore sAES2CriticalSection;
	struct mutex sDESCriticalSection;
	struct mutex sSHACriticalSection;

	/*
	 * An aligned and correctly shaped pre-allocated buffer used for DMA
	 * transfers
	 */
	u32 nDMABufferLength;
	u8 *pDMABuffer;
	dma_addr_t pDMABufferPhys;

	/* Workspace allocated at boot time and reserved to the Secure World */
	u32 nWorkspaceAddr;
	u32 nWorkspaceSize;
#endif

	/*
	 * Communications with the SM.
	 */
	struct SCXLNX_COMM sm;

	/*
	 * Lists the connections attached to this device.  A connection is
	 * created each time a user space application "opens" a file descriptor
	 * on the driver
	 */
	struct list_head conns;

	/*
	 * The spin lock used to protect concurrent access to the connection
	 * list.
	 */
	spinlock_t connsLock;

	struct SCXLNX_DEVICE_STATS sDeviceStats;

        /* 
         * A Mutex to provide exlusive locking of the ioctl()
         */
        struct mutex dev_mutex;
};

/* the bits of the nFlags field of the SCXLNX_DEVICE structure */
#define SCXLNX_DEVICE_FLAG_CDEV_INITIALIZED              (0)
#define SCXLNX_DEVICE_FLAG_SYSDEV_CLASS_REGISTERED       (1)
#define SCXLNX_DEVICE_FLAG_SYSDEV_REGISTERED             (2)
#define SCXLNX_DEVICE_FLAG_CDEV_REGISTERED               (3)
#define SCXLNX_DEVICE_FLAG_CDEV_ADDED                    (4)
#define SCXLNX_DEVICE_SYSFS_REGISTERED                   (5)

/*----------------------------------------------------------------------------*/
/*
 * This type describes a connection state.
 * This is used to determine whether a message is valid or not.
 *
 * Messages are only valid in a certain device state.
 * Messages may be invalidated between the start of the ioctl call and the
 * moment the message is sent to the Secure World.
 *
 * SCXLNX_CONN_STATE_NO_DEVICE_CONTEXT :
 *    The connection has no DEVICE_CONTEXT created and no
 *    CREATE_DEVICE_CONTEXT being processed by the Secure World
 * SCXLNX_CONN_STATE_CREATE_DEVICE_CONTEXT_SENT :
 *    The connection has a CREATE_DEVICE_CONTEXT being processed by the Secure
 *    World
 * SCXLNX_CONN_STATE_VALID_DEVICE_CONTEXT :
 *    The connection has a DEVICE_CONTEXT created and no
 *    DESTROY_DEVICE_CONTEXT is being processed by the Secure World
 * SCXLNX_CONN_STATE_DESTROY_DEVICE_CONTEXT_SENT :
 *    The connection has a DESTROY_DEVICE_CONTEXT being processed by the Secure
 *    World
 */
enum SCXLNX_CONN_STATE {
	SCXLNX_CONN_STATE_NO_DEVICE_CONTEXT = 0,
	SCXLNX_CONN_STATE_CREATE_DEVICE_CONTEXT_SENT,
	SCXLNX_CONN_STATE_VALID_DEVICE_CONTEXT,
	SCXLNX_CONN_STATE_DESTROY_DEVICE_CONTEXT_SENT
};


/*
 *  This type describes the  status of the command.
 *
 *  PENDING:
 *     The initial state; the command has not been sent yet.
 *	SENT:
 *     The command has been sent, we are waiting for an answer.
 *	ABORTED:
 *     The command cannot be sent because the device context is invalid.
 *     Note that this only covers the case where some other thread
 *     sent a DESTROY_DEVICE_CONTEXT command.
 */
enum SCXLNX_COMMAND_STATE {
	SCXLNX_COMMAND_STATE_PENDING = 0,
	SCXLNX_COMMAND_STATE_SENT,
	SCXLNX_COMMAND_STATE_ABORTED
};


/*
 * This structure describes a connection to the driver
 * A connection is created each time an application opens a file descriptor on
 * the driver
 */
struct SCXLNX_CONNECTION {
	/*
	 * Identifies the connection in the list of the connections attached to
	 * the same device.
	 */
	struct list_head list;

	/*
	 * State of the connection.
	 */
	enum SCXLNX_CONN_STATE nState;

	/*
	 * A pointer to the corresponding device structure
	 */
	struct SCXLNX_DEVICE *pDevice;

	/*
	 * A spinlock to use to access nState
	 */
	spinlock_t stateLock;

	/*
	 * Counts the number of operations currently pending on the connection.
	 * (for debug only)
	 */
	atomic_t nPendingOpCounter;

	/*
	 * A handle for the device context
	 */
	 u32 hDeviceContext;

	/*
	 * Lists the used shared memory descriptors
	 */
	struct list_head sUsedSharedMemoryList;

	/*
	 * Lists the free shared memory descriptors
	 */
	struct list_head sFreeSharedMemoryList;

	/*
	 * A mutex to use to access this structure
	 */
	struct mutex sharedMemoriesMutex;

	/*
	 * Counts the number of shared memories registered.
	 */
	atomic_t nShmemAllocated;

	/*
	 * Page to retrieve memory properties when
	 * registering shared memory through REGISTER_SHARED_MEMORY
	 * messages
	 */
	struct vm_area_struct **ppVmas;

	/*
	 * coarse page table allocation context
	 */
	struct SCXLNX_COARSE_PAGE_TABLE_ALLOCATION_CONTEXT sAllocationContext;

#ifdef CONFIG_TF_MSHIELD
	/* Lists all the Cryptoki Update Shortcuts */
	struct list_head ShortcutList;

	/* Lock to protect concurrent accesses to ShortcutList */
	spinlock_t shortcutListCriticalSectionLock;
#endif
};

/*----------------------------------------------------------------------------*/

/*
 * The nOperationID field of a message points to this structure.
 * It is used to identify the thread that triggered the message transmission
 * Whoever reads an answer can wake up that thread using the completion event
 */
struct SCXLNX_ANSWER_STRUCT {
	bool bAnswerCopied;
	union SCX_ANSWER_MESSAGE *pAnswer;
};

/*----------------------------------------------------------------------------*/

/**
 * The ASCII-C string representation of the base name of the devices managed by
 * this driver.
 */
#define SCXLNX_DEVICE_BASE_NAME	"tf_driver"


/**
 * The major and minor numbers of the registered character device driver.
 * Only 1 instance of the driver is supported.
 */
#define SCXLNX_DEVICE_MINOR_NUMBER	(0)

struct SCXLNX_DEVICE *SCXLNXGetDevice(void);

#define CLEAN_CACHE_CFG_MASK	(~0xC) /* 1111 0011 */

/*----------------------------------------------------------------------------*/
/*
 * Kernel Differences
 */

#ifdef CONFIG_ANDROID
#define GROUP_INFO		get_current_groups()
#else
#define GROUP_INFO		(current->group_info)
#endif

#endif  /* !defined(__SCXLNX_DEFS_H__) */
