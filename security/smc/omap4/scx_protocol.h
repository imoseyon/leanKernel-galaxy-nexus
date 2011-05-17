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

#ifndef __SCX_PROTOCOL_H__
#define __SCX_PROTOCOL_H__

/*----------------------------------------------------------------------------
 *
 * This header file defines the structure used in the SChannel Protocol.
 * See your Product Reference Manual for a specification of the SChannel
 * protocol.
 *---------------------------------------------------------------------------*/

/*
 * The driver interface version returned by the version ioctl
 */
#define SCX_DRIVER_INTERFACE_VERSION     0x04000000

/*
 * Protocol version handling
 */
#define SCX_S_PROTOCOL_MAJOR_VERSION  (0x06)
#define GET_PROTOCOL_MAJOR_VERSION(a) (a >> 24)
#define GET_PROTOCOL_MINOR_VERSION(a) ((a >> 16) & 0xFF)

/*
 * The size, in bytes, of the L1 Shared Buffer.
 */
#define SCX_COMM_BUFFER_SIZE  (0x1000)  /* 4kB*/

/*
 * The S flag of the nConfigFlags_S register.
 */
#define SCX_CONFIG_FLAG_S   (1 << 3)

/*
 * The TimeSlot field of the nSyncSerial_N register.
 */
#define SCX_SYNC_SERIAL_TIMESLOT_N   (1)

/*
 * nStatus_S related defines.
 */
#define SCX_STATUS_P_MASK            (0X00000001)
#define SCX_STATUS_POWER_STATE_SHIFT (3)
#define SCX_STATUS_POWER_STATE_MASK  (0x1F << SCX_STATUS_POWER_STATE_SHIFT)

/*
 * Possible power states of the POWER_STATE field of the nStatus_S register
 */
#define SCX_POWER_MODE_COLD_BOOT          (0)
#define SCX_POWER_MODE_WARM_BOOT          (1)
#define SCX_POWER_MODE_ACTIVE             (3)
#define SCX_POWER_MODE_READY_TO_SHUTDOWN  (5)
#define SCX_POWER_MODE_READY_TO_HIBERNATE (7)
#define SCX_POWER_MODE_WAKEUP             (8)
#define SCX_POWER_MODE_PANIC              (15)

/*
 * Possible nCommand values for MANAGEMENT commands
 */
#define SCX_MANAGEMENT_HIBERNATE            (1)
#define SCX_MANAGEMENT_SHUTDOWN             (2)
#define SCX_MANAGEMENT_PREPARE_FOR_CORE_OFF (3)
#define SCX_MANAGEMENT_RESUME_FROM_CORE_OFF (4)

/*
 * The capacity of the Normal Word message queue, in number of slots.
 */
#define SCX_N_MESSAGE_QUEUE_CAPACITY  (512)

/*
 * The capacity of the Secure World message answer queue, in number of slots.
 */
#define SCX_S_ANSWER_QUEUE_CAPACITY  (256)

/*
 * The value of the S-timeout register indicating an infinite timeout.
 */
#define SCX_S_TIMEOUT_0_INFINITE  (0xFFFFFFFF)
#define SCX_S_TIMEOUT_1_INFINITE  (0xFFFFFFFF)

/*
 * The value of the S-timeout register indicating an immediate timeout.
 */
#define SCX_S_TIMEOUT_0_IMMEDIATE  (0x0)
#define SCX_S_TIMEOUT_1_IMMEDIATE  (0x0)

/*
 * Identifies the get protocol version SMC.
 */
#define SCX_SMC_GET_PROTOCOL_VERSION (0XFFFFFFFB)

/*
 * Identifies the init SMC.
 */
#define SCX_SMC_INIT                 (0XFFFFFFFF)

/*
 * Identifies the reset irq SMC.
 */
#define SCX_SMC_RESET_IRQ            (0xFFFFFFFE)

/*
 * Identifies the SET_W3B SMC.
 */
#define SCX_SMC_WAKE_UP              (0xFFFFFFFD)

/*
 * Identifies the STOP SMC.
 */
#define SCX_SMC_STOP                 (0xFFFFFFFC)

/*
 * Identifies the n-yield SMC.
 */
#define SCX_SMC_N_YIELD              (0X00000003)


/* Possible stop commands for SMC_STOP */
#define SCSTOP_HIBERNATE           (0xFFFFFFE1)
#define SCSTOP_SHUTDOWN            (0xFFFFFFE2)

/*
 * representation of an UUID.
 */
struct SCX_UUID {
	u32 time_low;
	u16 time_mid;
	u16 time_hi_and_version;
	u8 clock_seq_and_node[8];
};


/**
 * Command parameters.
 */
struct SCX_COMMAND_PARAM_VALUE {
	u32    a;
	u32    b;
};

struct SCX_COMMAND_PARAM_TEMP_MEMREF {
	u32    nDescriptor; /* data pointer for exchange message.*/
	u32    nSize;
	u32    nOffset;
};

struct SCX_COMMAND_PARAM_MEMREF {
	u32      hBlock;
	u32      nSize;
	u32      nOffset;
};

union SCX_COMMAND_PARAM {
	struct SCX_COMMAND_PARAM_VALUE        sValue;
	struct SCX_COMMAND_PARAM_TEMP_MEMREF  sTempMemref;
	struct SCX_COMMAND_PARAM_MEMREF       sMemref;
};

/**
 * Answer parameters.
 */
struct SCX_ANSWER_PARAM_VALUE {
	u32   a;
	u32   b;
};

struct SCX_ANSWER_PARAM_SIZE {
	u32   _ignored;
	u32   nSize;
};

union SCX_ANSWER_PARAM {
	struct SCX_ANSWER_PARAM_SIZE    sSize;
	struct SCX_ANSWER_PARAM_VALUE   sValue;
};

/*
 * Descriptor tables capacity
 */
#define SCX_MAX_W3B_COARSE_PAGES                 (2)
#define SCX_MAX_COARSE_PAGES                     (8)
#define SCX_DESCRIPTOR_TABLE_CAPACITY_BIT_SHIFT  (8)
#define SCX_DESCRIPTOR_TABLE_CAPACITY \
	(1 << SCX_DESCRIPTOR_TABLE_CAPACITY_BIT_SHIFT)
#define SCX_DESCRIPTOR_TABLE_CAPACITY_MASK \
	(SCX_DESCRIPTOR_TABLE_CAPACITY - 1)
/* Shared memories coarse pages can map up to 1MB */
#define SCX_MAX_COARSE_PAGE_MAPPED_SIZE \
	(PAGE_SIZE * SCX_DESCRIPTOR_TABLE_CAPACITY)
/* Shared memories cannot exceed 8MB */
#define SCX_MAX_SHMEM_SIZE \
	(SCX_MAX_COARSE_PAGE_MAPPED_SIZE << 3)

/*
 * Buffer size for version description fields
 */
#define SCX_DESCRIPTION_BUFFER_LENGTH 64

/*
 * Shared memory type flags.
 */
#define SCX_SHMEM_TYPE_READ         (0x00000001)
#define SCX_SHMEM_TYPE_WRITE        (0x00000002)

/*
 * Shared mem flags
 */
#define SCX_SHARED_MEM_FLAG_INPUT   1
#define SCX_SHARED_MEM_FLAG_OUTPUT  2
#define SCX_SHARED_MEM_FLAG_INOUT   3


/*
 * Parameter types
 */
#define SCX_PARAM_TYPE_NONE               0x0
#define SCX_PARAM_TYPE_VALUE_INPUT        0x1
#define SCX_PARAM_TYPE_VALUE_OUTPUT       0x2
#define SCX_PARAM_TYPE_VALUE_INOUT        0x3
#define SCX_PARAM_TYPE_MEMREF_TEMP_INPUT  0x5
#define SCX_PARAM_TYPE_MEMREF_TEMP_OUTPUT 0x6
#define SCX_PARAM_TYPE_MEMREF_TEMP_INOUT  0x7
#define SCX_PARAM_TYPE_MEMREF_INPUT       0xD
#define SCX_PARAM_TYPE_MEMREF_OUTPUT      0xE
#define SCX_PARAM_TYPE_MEMREF_INOUT       0xF

#define SCX_PARAM_TYPE_MEMREF_FLAG               0x4
#define SCX_PARAM_TYPE_REGISTERED_MEMREF_FLAG    0x8


#define SCX_MAKE_PARAM_TYPES(t0, t1, t2, t3) \
	((t0) | ((t1) << 4) | ((t2) << 8) | ((t3) << 12))
#define SCX_GET_PARAM_TYPE(t, i) (((t) >> (4 * i)) & 0xF)

/*
 * Login types.
 */
#define SCX_LOGIN_PUBLIC              0x00000000
#define SCX_LOGIN_USER                0x00000001
#define SCX_LOGIN_GROUP               0x00000002
#define SCX_LOGIN_APPLICATION         0x00000004
#define SCX_LOGIN_APPLICATION_USER    0x00000005
#define SCX_LOGIN_APPLICATION_GROUP   0x00000006
#define SCX_LOGIN_AUTHENTICATION      0x80000000
#define SCX_LOGIN_PRIVILEGED          0x80000002

/* Login variants */

#define SCX_LOGIN_VARIANT(mainType, os, variant) \
	((mainType) | (1 << 27) | ((os) << 16) | ((variant) << 8))

#define SCX_LOGIN_GET_MAIN_TYPE(type) \
	((type) & ~SCX_LOGIN_VARIANT(0, 0xFF, 0xFF))

#define SCX_LOGIN_OS_ANY       0x00
#define SCX_LOGIN_OS_LINUX     0x01
#define SCX_LOGIN_OS_ANDROID   0x04

/* OS-independent variants */
#define SCX_LOGIN_USER_NONE \
	SCX_LOGIN_VARIANT(SCX_LOGIN_USER, SCX_LOGIN_OS_ANY, 0xFF)
#define SCX_LOGIN_GROUP_NONE \
	SCX_LOGIN_VARIANT(SCX_LOGIN_GROUP, SCX_LOGIN_OS_ANY, 0xFF)
#define SCX_LOGIN_APPLICATION_USER_NONE \
	SCX_LOGIN_VARIANT(SCX_LOGIN_APPLICATION_USER, SCX_LOGIN_OS_ANY, 0xFF)
#define SCX_LOGIN_AUTHENTICATION_BINARY_SHA1_HASH \
	SCX_LOGIN_VARIANT(SCX_LOGIN_AUTHENTICATION, SCX_LOGIN_OS_ANY, 0x01)
#define SCX_LOGIN_PRIVILEGED_KERNEL \
	SCX_LOGIN_VARIANT(SCX_LOGIN_PRIVILEGED, SCX_LOGIN_OS_ANY, 0x01)

/* Linux variants */
#define SCX_LOGIN_USER_LINUX_EUID \
	SCX_LOGIN_VARIANT(SCX_LOGIN_USER, SCX_LOGIN_OS_LINUX, 0x01)
#define SCX_LOGIN_GROUP_LINUX_GID \
	SCX_LOGIN_VARIANT(SCX_LOGIN_GROUP, SCX_LOGIN_OS_LINUX, 0x01)
#define SCX_LOGIN_APPLICATION_LINUX_PATH_SHA1_HASH \
	SCX_LOGIN_VARIANT(SCX_LOGIN_APPLICATION, SCX_LOGIN_OS_LINUX, 0x01)
#define SCX_LOGIN_APPLICATION_USER_LINUX_PATH_EUID_SHA1_HASH \
	SCX_LOGIN_VARIANT(SCX_LOGIN_APPLICATION_USER, SCX_LOGIN_OS_LINUX, 0x01)
#define SCX_LOGIN_APPLICATION_GROUP_LINUX_PATH_GID_SHA1_HASH \
	SCX_LOGIN_VARIANT(SCX_LOGIN_APPLICATION_GROUP, SCX_LOGIN_OS_LINUX, 0x01)

/* Android variants */
#define SCX_LOGIN_USER_ANDROID_EUID \
	SCX_LOGIN_VARIANT(SCX_LOGIN_USER, SCX_LOGIN_OS_ANDROID, 0x01)
#define SCX_LOGIN_GROUP_ANDROID_GID \
	SCX_LOGIN_VARIANT(SCX_LOGIN_GROUP, SCX_LOGIN_OS_ANDROID, 0x01)
#define SCX_LOGIN_APPLICATION_ANDROID_UID \
	SCX_LOGIN_VARIANT(SCX_LOGIN_APPLICATION, SCX_LOGIN_OS_ANDROID, 0x01)
#define SCX_LOGIN_APPLICATION_USER_ANDROID_UID_EUID \
	SCX_LOGIN_VARIANT(SCX_LOGIN_APPLICATION_USER, SCX_LOGIN_OS_ANDROID, \
		0x01)
#define SCX_LOGIN_APPLICATION_GROUP_ANDROID_UID_GID \
	SCX_LOGIN_VARIANT(SCX_LOGIN_APPLICATION_GROUP, SCX_LOGIN_OS_ANDROID, \
		0x01)

/*
 *  return origins
 */
#define SCX_ORIGIN_COMMS       2
#define SCX_ORIGIN_TEE         3
#define SCX_ORIGIN_TRUSTED_APP 4
/*
 * The SCX message types.
 */
#define SCX_MESSAGE_TYPE_CREATE_DEVICE_CONTEXT   0x02
#define SCX_MESSAGE_TYPE_DESTROY_DEVICE_CONTEXT  0xFD
#define SCX_MESSAGE_TYPE_REGISTER_SHARED_MEMORY  0xF7
#define SCX_MESSAGE_TYPE_RELEASE_SHARED_MEMORY   0xF9
#define SCX_MESSAGE_TYPE_OPEN_CLIENT_SESSION     0xF0
#define SCX_MESSAGE_TYPE_CLOSE_CLIENT_SESSION    0xF2
#define SCX_MESSAGE_TYPE_INVOKE_CLIENT_COMMAND   0xF5
#define SCX_MESSAGE_TYPE_CANCEL_CLIENT_COMMAND   0xF4
#define SCX_MESSAGE_TYPE_MANAGEMENT              0xFE


/*
 * The error codes
 */
#define S_SUCCESS						0x00000000
#define S_ERROR_NO_DATA				0xFFFF000B
#define S_ERROR_OUT_OF_MEMORY		0xFFFF000C


struct SCX_COMMAND_HEADER {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo;
	u32                      nOperationID;
};

struct SCX_ANSWER_HEADER {
	u8                   nMessageSize;
	u8                   nMessageType;
	u16                  nMessageInfo;
	u32                  nOperationID;
	u32                  nErrorCode;
};

/*
 * CREATE_DEVICE_CONTEXT command message.
 */
struct SCX_COMMAND_CREATE_DEVICE_CONTEXT {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	u32                      nOperationID;
	u32                      nDeviceContextID;
};

/*
 * CREATE_DEVICE_CONTEXT answer message.
 */
struct SCX_ANSWER_CREATE_DEVICE_CONTEXT {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	/* an opaque Normal World identifier for the operation */
	u32                      nOperationID;
	u32                      nErrorCode;
	/* an opaque Normal World identifier for the device context */
	u32                      hDeviceContext;
};

/*
 * DESTROY_DEVICE_CONTEXT command message.
 */
struct SCX_COMMAND_DESTROY_DEVICE_CONTEXT {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	u32                      nOperationID;
	u32                      hDeviceContext;
};

/*
 * DESTROY_DEVICE_CONTEXT answer message.
 */
struct SCX_ANSWER_DESTROY_DEVICE_CONTEXT {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	/* an opaque Normal World identifier for the operation */
	u32                      nOperationID;
	u32                      nErrorCode;
	u32                      nDeviceContextID;
};

/*
 * OPEN_CLIENT_SESSION command message.
 */
struct SCX_COMMAND_OPEN_CLIENT_SESSION {
	u8                            nMessageSize;
	u8                            nMessageType;
	u16                           nParamTypes;
	/* an opaque Normal World identifier for the operation */
	u32                           nOperationID;
	u32                           hDeviceContext;
	u32                           nCancellationID;
	u64                           sTimeout;
	struct SCX_UUID               sDestinationUUID;
	union SCX_COMMAND_PARAM       sParams[4];
	u32                           nLoginType;
	/*
	 * Size = 0 for public, [16] for group identification, [20] for
	 * authentication
	 */
	u8                            sLoginData[20];
};

/*
 * OPEN_CLIENT_SESSION answer message.
 */
struct SCX_ANSWER_OPEN_CLIENT_SESSION {
	u8                       nMessageSize;
	u8                       nMessageType;
	u8                       nReturnOrigin;
	u8                       __nReserved;
	/* an opaque Normal World identifier for the operation */
	u32                      nOperationID;
	u32                      nErrorCode;
	u32                      hClientSession;
	union SCX_ANSWER_PARAM   sAnswers[4];
};

/*
 * CLOSE_CLIENT_SESSION command message.
 */
struct SCX_COMMAND_CLOSE_CLIENT_SESSION {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	/* an opaque Normal World identifier for the operation */
	u32                      nOperationID;
	u32                      hDeviceContext;
	u32                      hClientSession;
};

/*
 * CLOSE_CLIENT_SESSION answer message.
 */
struct SCX_ANSWER_CLOSE_CLIENT_SESSION {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	/* an opaque Normal World identifier for the operation */
	u32                      nOperationID;
	u32                      nErrorCode;
};


/*
 * REGISTER_SHARED_MEMORY command message
 */
struct SCX_COMMAND_REGISTER_SHARED_MEMORY {
	u8  nMessageSize;
	u8  nMessageType;
	u16 nMemoryFlags;
	u32 nOperationID;
	u32 hDeviceContext;
	u32 nBlockID;
	u32 nSharedMemSize;
	u32 nSharedMemStartOffset;
	u32 nSharedMemDescriptors[SCX_MAX_COARSE_PAGES];
};

/*
 * REGISTER_SHARED_MEMORY answer message.
 */
struct SCX_ANSWER_REGISTER_SHARED_MEMORY {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	/* an opaque Normal World identifier for the operation */
	u32                      nOperationID;
	u32                      nErrorCode;
	u32                      hBlock;
};

/*
 * RELEASE_SHARED_MEMORY command message.
 */
struct SCX_COMMAND_RELEASE_SHARED_MEMORY {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	/* an opaque Normal World identifier for the operation */
	u32                      nOperationID;
	u32                      hDeviceContext;
	u32                      hBlock;
};

/*
 * RELEASE_SHARED_MEMORY answer message.
 */
struct SCX_ANSWER_RELEASE_SHARED_MEMORY {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nMessageInfo_RFU;
	u32                      nOperationID;
	u32                      nErrorCode;
	u32                      nBlockID;
};

/*
 * INVOKE_CLIENT_COMMAND command message.
 */
struct SCX_COMMAND_INVOKE_CLIENT_COMMAND {
	u8                       nMessageSize;
	u8                       nMessageType;
	u16                      nParamTypes;
	u32                      nOperationID;
	u32                      hDeviceContext;
	u32                      hClientSession;
	u64                      sTimeout;
	u32                      nCancellationID;
	u32                      nClientCommandIdentifier;
	union SCX_COMMAND_PARAM  sParams[4];
};

/*
 * INVOKE_CLIENT_COMMAND command answer.
 */
struct SCX_ANSWER_INVOKE_CLIENT_COMMAND {
	u8                     nMessageSize;
	u8                     nMessageType;
	u8                     nReturnOrigin;
	u8                     __nReserved;
	u32                    nOperationID;
	u32                    nErrorCode;
	union SCX_ANSWER_PARAM sAnswers[4];
};

/*
 * CANCEL_CLIENT_OPERATION command message.
 */
struct SCX_COMMAND_CANCEL_CLIENT_OPERATION {
	u8                   nMessageSize;
	u8                   nMessageType;
	u16                  nMessageInfo_RFU;
	/* an opaque Normal World identifier for the operation */
	u32                  nOperationID;
	u32                  hDeviceContext;
	u32                  hClientSession;
	u32                  nCancellationID;
};

struct SCX_ANSWER_CANCEL_CLIENT_OPERATION {
	u8                   nMessageSize;
	u8                   nMessageType;
	u16                  nMessageInfo_RFU;
	u32                  nOperationID;
	u32                  nErrorCode;
};

/*
 * MANAGEMENT command message.
 */
struct SCX_COMMAND_MANAGEMENT {
	u8                   nMessageSize;
	u8                   nMessageType;
	u16                  nCommand;
	u32                  nOperationID;
	u32                  nW3BSize;
	u32                  nW3BStartOffset;
	u32                  nSharedMemDescriptors[1];
};

/*
 * POWER_MANAGEMENT answer message.
 * The message does not provide message specific parameters.
 * Therefore no need to define a specific answer structure
 */

/*
 * Structure for L2 messages
 */
union SCX_COMMAND_MESSAGE {
	struct SCX_COMMAND_HEADER sHeader;
	struct SCX_COMMAND_CREATE_DEVICE_CONTEXT sCreateDeviceContextMessage;
	struct SCX_COMMAND_DESTROY_DEVICE_CONTEXT sDestroyDeviceContextMessage;
	struct SCX_COMMAND_OPEN_CLIENT_SESSION sOpenClientSessionMessage;
	struct SCX_COMMAND_CLOSE_CLIENT_SESSION sCloseClientSessionMessage;
	struct SCX_COMMAND_REGISTER_SHARED_MEMORY sRegisterSharedMemoryMessage;
	struct SCX_COMMAND_RELEASE_SHARED_MEMORY sReleaseSharedMemoryMessage;
	struct SCX_COMMAND_INVOKE_CLIENT_COMMAND sInvokeClientCommandMessage;
	struct SCX_COMMAND_CANCEL_CLIENT_OPERATION
		sCancelClientOperationMessage;
	struct SCX_COMMAND_MANAGEMENT sManagementMessage;
};

/*
 * Structure for any L2 answer
 */

union SCX_ANSWER_MESSAGE {
	struct SCX_ANSWER_HEADER sHeader;
	struct SCX_ANSWER_CREATE_DEVICE_CONTEXT sCreateDeviceContextAnswer;
	struct SCX_ANSWER_OPEN_CLIENT_SESSION sOpenClientSessionAnswer;
	struct SCX_ANSWER_CLOSE_CLIENT_SESSION sCloseClientSessionAnswer;
	struct SCX_ANSWER_REGISTER_SHARED_MEMORY sRegisterSharedMemoryAnswer;
	struct SCX_ANSWER_RELEASE_SHARED_MEMORY sReleaseSharedMemoryAnswer;
	struct SCX_ANSWER_INVOKE_CLIENT_COMMAND sInvokeClientCommandAnswer;
	struct SCX_ANSWER_DESTROY_DEVICE_CONTEXT sDestroyDeviceContextAnswer;
	struct SCX_ANSWER_CANCEL_CLIENT_OPERATION sCancelClientOperationAnswer;
};

/* Structure of the Communication Buffer */
struct SCHANNEL_C1S_BUFFER {
	u32 nConfigFlags_S;
	u32 nW3BSizeMax_S;
	u32 nReserved0;
	u32 nW3BSizeCurrent_S;
	u8 sReserved1[48];
	u8 sVersionDescription[SCX_DESCRIPTION_BUFFER_LENGTH];
	u32 nStatus_S;
	u32 sReserved2;
	u32 nSyncSerial_N;
	u32 nSyncSerial_S;
	u64 sTime_N[2];
	u64 sTimeout_S[2];
	u32 nFirstCommand;
	u32 nFirstFreeCommand;
	u32 nFirstAnswer;
	u32 nFirstFreeAnswer;
	u32 nW3BDescriptors[128];
	#ifdef CONFIG_TF_MSHIELD
	u8  sRPCTraceBuffer[140];
	u8  sRPCShortcutBuffer[180];
	#else
	u8  sReserved3[320];
	#endif
	u32 sCommandQueue[SCX_N_MESSAGE_QUEUE_CAPACITY];
	u32 sAnswerQueue[SCX_S_ANSWER_QUEUE_CAPACITY];
};


/*
 * SCX_VERSION_INFORMATION_BUFFER structure description
 * Description of the sVersionBuffer handed over from user space to kernel space
 * This field is filled by the driver during a CREATE_DEVICE_CONTEXT ioctl
 * and handed back to user space
 */
struct SCX_VERSION_INFORMATION_BUFFER {
	u8 sDriverDescription[65];
	u8 sSecureWorldDescription[65];
};


/* The IOCTLs the driver supports */
#include <linux/ioctl.h>

#define IOCTL_SCX_GET_VERSION     _IO('z', 0)
#define IOCTL_SCX_EXCHANGE        _IOWR('z', 1, union SCX_COMMAND_MESSAGE)
#define IOCTL_SCX_GET_DESCRIPTION _IOR('z', 2, \
	struct SCX_VERSION_INFORMATION_BUFFER)

#endif  /* !defined(__SCX_PROTOCOL_H__) */
