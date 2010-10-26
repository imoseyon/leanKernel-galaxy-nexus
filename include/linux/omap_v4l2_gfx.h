/*
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * This file specifies the custom ioctl API between a client "consumer"
 * process and the V4L2-GFX driver. The consumer process should only use
 * these APIs and will typically/ultimately be a GL application.
 *
 * There will also be a "producer" process which queues multimedia
 * content to the driver, however, this will only use standard V4L2 APIs.
 */

#ifndef _OMAP_V4L2_GFX_H_
#define _OMAP_V4L2_GFX_H_

#include <linux/videodev.h>

/*
 * @see V4L2_GFX_IOC_CONSUMER, struct v4l2_gfx_consumer_params
 */
enum v4l2_gfx_consumer_type {
	/*
	 * Wait for the producer process to activate a video stream
	 */
	V4L2_GFX_CONSUMER_WAITSTREAM,
	};

/*
 * @see V4L2_GFX_IOC_CONSUMER
 */
struct v4l2_gfx_consumer_params {
	/*
	 * @see v4l2_gfx_consumer_type
	 */
	int type;			/* w */
	/*
	 * If the consumer process is waiting the ioctl will block until the
	 * timeout expires or the expected event occurs, see the type field
	 */
	unsigned int timeout_ms;	/* w */
	/*
	 * If acquire_timeout_ms > 0 and no streaming activity has been detected
	 * for acquire_timeout_ms milliseconds the V4L2_GFX_IOC_ACQ ioctl will
	 * return with ETIMEOUT
	 */
	unsigned int acquire_timeout_ms;	/* w */
};

/*
 * @see V4L2_GFX_IOC_INFO
 */
struct v4l2_gfx_info_params {

	/*
	 * Return how many times the device has been opened, this number will
	 * decrement when the device is closed.
	 *
	 * One use for this might be to detect if a consumer or producer is
	 * active and in the process of setting up a stream. However this could
	 * be unreliable if the processes are in the process of closing / crashing.
	 *
	 * Obviously this value will always be at least one i.e. the process
	 * issuing the ioctl opens the device.
	 */
	unsigned int opencnt;			/* r */

};

/*
 * @see V4L2_GFX_IOC_PRODUCER
 */
struct v4l2_gfx_producer_params {
	/*
	 * If set mark the producer side as open, if not set mark as closed.
	 * For Android we need this because the mediaserver won't close the
	 * driver.
	 */
	#define V4L2_GFX_PRODUCER_MASK_OPEN	0x1
	unsigned int flags;				/* w */
};

struct v4l2_gfx_buf_params {
	/*
	 * Buffer index.
	 *
	 * On acquire, when the ioctl returns the bufid field will be filled in
	 * with the next buffer with data available.
	 *
	 * On release, the consumer process just specifies the buffer to release
	 * which usually is the last acquired buffer index.
	 */
    int bufid;	/* r/w */

	/*
	 * Cropping information
	 * For the acquire ioctl only
	 */
	int crop_top;		/* r */
	int crop_left;		/* r */
	int crop_width;		/* r */
	int crop_height;	/* r */
};

/*
 * This ioctl should be issued once by the consumer process before starting
 * any rendering loop. It allows the process to wait for the producer process
 * to become ready.
 *
 * @see struct v4l2_gfx_consumer_params
 *
 * Return value:
 * 		Returns 0 if successful, or -1 on error, in which case errno indicates
 *		the error.
 */
#define V4L2_GFX_IOC_CONSUMER _IOWR ('v', BASE_VIDIOCPRIVATE+0, \
									struct v4l2_gfx_consumer_params)

/*
 * Acquire the buffer to be rendered and its properties.
 *
 * @see struct v4l2_gfx_buf_params
 *
 * Return value:
 * 		Returns 0 if successful, or -1 on error, in which case errno indicates
 *		the error.
 *
 *		ETIMEDOUT	If acquire_timeout_ms is set via V4L2_GFX_IOC_CONSUMER
 *					this error code can be returned.
 *		ENODEV		If the producer side of the stream stops this error will
 *					be returned.
 */
#define V4L2_GFX_IOC_ACQ _IOR ('v', BASE_VIDIOCPRIVATE+1, \
									struct v4l2_gfx_buf_params)

/*
 * Release the buffer that was rendered
 *
 * @see struct v4l2_gfx_buf_params
 *
 * Return value:
 * 		Returns 0 if successful, or -1 on error, in which case errno indicates
 *		the error.
 *
 *		ETIMEDOUT	It took longer than 16ms for the app to render the frame
 *					(This will probably go away to avoid render loop stalls)
 *		EINVAL		Attempted to release an invalid buffer index.
 */
#define V4L2_GFX_IOC_REL _IOW ('v', BASE_VIDIOCPRIVATE+2, \
									struct v4l2_gfx_buf_params)

/*
 * Ioctl used to get information about the device
 *
 * @see struct v4l2_gfx_info_params
 *
 * Return value:
 * 		Returns 0 if successful, or -1 on error, in which case errno indicates
 *		the error.
 */
#define V4L2_GFX_IOC_INFO _IOWR ('v', BASE_VIDIOCPRIVATE+3, \
									struct v4l2_gfx_info_params)

/*
 * Ioctl used to set producer params
 *
 * @see struct v4l2_gfx_producer_params
 *
 * Return value:
 * 		Returns 0 if successful, or -1 on error, in which case errno indicates
 *		the error.
 */
#define V4L2_GFX_IOC_PRODUCER _IOWR ('v', BASE_VIDIOCPRIVATE+4, \
									struct v4l2_gfx_producer_params)
#endif 	// _OMAP_V4L2_GFX_H_
