/*
 * Remote processor messaging
 *
 * Copyright(c) 2011 Texas Instruments. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name Texas Instruments nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _LINUX_RPMSG_H
#define _LINUX_RPMSG_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>

/* The feature bitmap for virtio rpmsg */
#define VIRTIO_RPMSG_F_NS	0 /* RP supports name service notifications */

/**
 * struct rpmsg_hdr -
 *
 * ... keep documenting ...
 */
struct rpmsg_hdr {
	u16 len;
	u16 flags;
	u32 src;
	u32 dst;
	u32 unused;
	u8 data[0];
} __packed;

enum rpmsg_ns_flags {
	RPMSG_NS_CREATE		= 0,
	RPMSG_NS_DESTROY	= 1,
};

struct rpmsg_ns_msg {
	char name[RPMSG_NAME_SIZE];
	u32 addr;
	u32 flags;
} __packed;

/* driver requests */
enum {
	VPROC_BUF_ADDR,
	VPROC_BUF_NUM,
	VPROC_BUF_SZ,
	VPROC_SIM_BASE,
	VPROC_STATIC_CHANNELS,
};

#define RPMSG_ADDR_ANY		0xFFFFFFFF

struct virtproc_info;

/**
 * rpmsg_channel - rpmsg channels are the devices of the rpmsg bus
 *
 * @vrp: the remote processor this channel connects to
 * @dev: underlying device
 * @id: the device type identification (used to match an rpmsg driver)
 * @src: local address of this channel
 * @dst: destination address of the remote service
 * @priv: private pointer for the driver's use.
 * @ept: local rpmsg endpoint of this channel
 * @announce: need to tell remoteproc about channel creation/removal
 */
struct rpmsg_channel {
	struct virtproc_info *vrp;
	struct device dev;
	struct rpmsg_device_id id;
	u32 src;
	u32 dst;
	void *priv;
	struct rpmsg_endpoint *ept;
	bool announce;
};

struct rpmsg_channel_info {
	char name[RPMSG_NAME_SIZE];
	u32 src;
	u32 dst;
};

/**
 * struct rpmsg_endpoint
 *
 * @rpdev:
 * @cb:
 * @src: local rpmsg address
 * @priv:
 */
struct rpmsg_endpoint {
	struct rpmsg_channel *rpdev;
	void (*cb)(struct rpmsg_channel *, void *, int, void *, u32);
	u32 addr;
	void *priv;
};

/**
 * rpmsg_driver - operations for a rpmsg I/O driver
 * @driver: underlying device driver (populate name and owner).
 * @id_table: the ids serviced by this driver.
 * @probe: the function to call when a device is found.  Returns 0 or -errno.
 * @remove: the function when a device is removed.
 * @callback: invoked when a message is received on the channel
 */
struct rpmsg_driver {
	struct device_driver drv;
	const struct rpmsg_device_id *id_table;
	int (*probe)(struct rpmsg_channel *dev);
	void (*remove)(struct rpmsg_channel *dev);
	void (*callback)(struct rpmsg_channel *, void *, int, void *, u32);
};

int register_rpmsg_device(struct rpmsg_channel *dev);
void unregister_rpmsg_device(struct rpmsg_channel *dev);
int register_rpmsg_driver(struct rpmsg_driver *drv);
void unregister_rpmsg_driver(struct rpmsg_driver *drv);
void rpmsg_destroy_ept(struct rpmsg_endpoint *);
struct rpmsg_endpoint *rpmsg_create_ept(struct rpmsg_channel *,
		void (*cb)(struct rpmsg_channel *, void *, int, void *, u32),
		void *priv, u32 addr);

int
rpmsg_send_offchannel_raw(struct rpmsg_channel *, u32, u32, void *, int, bool);

static inline
int rpmsg_send_offchannel(struct rpmsg_channel *rpdev, u32 src, u32 dst,
							void *data, int len)
{
	return rpmsg_send_offchannel_raw(rpdev, src, dst, data, len, true);
}

static inline int rpmsg_send(struct rpmsg_channel *rpdev, void *data, int len)
{
	return rpmsg_send_offchannel(rpdev, rpdev->src, rpdev->dst, data, len);
}

static inline
int rpmsg_sendto(struct rpmsg_channel *rpdev, void *data, int len, u32 dst)
{
	return rpmsg_send_offchannel(rpdev, rpdev->src, dst, data, len);
}

static inline
int rpmsg_trysend_offchannel(struct rpmsg_channel *rpdev, u32 src, u32 dst,
							void *data, int len)
{
	return rpmsg_send_offchannel_raw(rpdev, src, dst, data, len, false);
}

static inline
int rpmsg_trysend(struct rpmsg_channel *rpdev, void *data, int len)
{
	return rpmsg_trysend_offchannel(rpdev, rpdev->src, rpdev->dst,
								data, len);
}

static inline
int rpmsg_trysendto(struct rpmsg_channel *rpdev, void *data, int len, u32 dst)
{
	return rpmsg_trysend_offchannel(rpdev, rpdev->src, dst, data, len);
}

#endif /* _LINUX_RPMSG_H */
