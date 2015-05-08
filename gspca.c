/*
 * Main USB camera driver
 *
 * Copyright (C) 2008-2011 Jean-François Moine <http://moinejf.free.fr>
 *
 * Camera button input handling by Márton Németh
 * Copyright (C) 2009-2010 Márton Németh <nm127@freemail.hu>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
 
/*
 * WARNING: THIS IS A PATCHED VERION OF alt_xfer(),
 * WICH RETURNS THE SECOND ISOC ENDPOINT, NOT THE FIRST!
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define pinect_VERSION	"2.14.0"

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/io.h>
#include <asm/page.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>

#include "gspca.h"
#include "lt.h"

#if IS_ENABLED(CONFIG_INPUT)
#include <linux/input.h>
#include <linux/usb/input.h>
#endif

/* global values */
#define DEF_NURBS 3		/* default number of URBs */
#if DEF_NURBS > MAX_NURBS
#error "DEF_NURBS too big"
#endif

int pinect_debug;

static void PDEBUG_MODE(struct pinect_dev *pinect_dev, int debug, char *txt,
			__u32 pixfmt, int w, int h)
{
	if ((pixfmt >> 24) >= '0' && (pixfmt >> 24) <= 'z') {
		PDEBUG(debug, "%s %c%c%c%c %dx%d",
			txt,
			pixfmt & 0xff,
			(pixfmt >> 8) & 0xff,
			(pixfmt >> 16) & 0xff,
			pixfmt >> 24,
			w, h);
	} else {
		PDEBUG(debug, "%s 0x%08x %dx%d",
			txt,
			pixfmt,
			w, h);
	}
}

/* specific memory types - !! should be different from V4L2_MEMORY_xxx */
#define pinect_MEMORY_NO 0	/* V4L2_MEMORY_xxx starts from 1 */
#define pinect_MEMORY_READ 7

#define BUF_ALL_FLAGS (V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_DONE)

/*
 * VMA operations.
 */
static void pinect_vm_open(struct vm_area_struct *vma)
{
	struct pinect_frame *frame = vma->vm_private_data;

	frame->vma_use_count++;
	frame->v4l2_buf.flags |= V4L2_BUF_FLAG_MAPPED;
}

static void pinect_vm_close(struct vm_area_struct *vma)
{
	struct pinect_frame *frame = vma->vm_private_data;

	if (--frame->vma_use_count <= 0)
		frame->v4l2_buf.flags &= ~V4L2_BUF_FLAG_MAPPED;
}

static const struct vm_operations_struct pinect_vm_ops = {
	.open		= pinect_vm_open,
	.close		= pinect_vm_close,
};

/*
 * Input and interrupt endpoint handling functions
 */
#if IS_ENABLED(CONFIG_INPUT)
static void int_irq(struct urb *urb)
{
	struct pinect_dev *pinect_dev = (struct pinect_dev *) urb->context;
	int ret;

	ret = urb->status;
	switch (ret) {
	case 0:
		if (pinect_dev->sd_desc->int_pkt_scan(pinect_dev,
		    urb->transfer_buffer, urb->actual_length) < 0) {
			PERR("Unknown packet received");
		}
		break;

	case -ENOENT:
	case -ECONNRESET:
	case -ENODEV:
	case -ESHUTDOWN:
		/* Stop is requested either by software or hardware is gone,
		 * keep the ret value non-zero and don't resubmit later.
		 */
		break;

	default:
		PERR("URB error %i, resubmitting", urb->status);
		urb->status = 0;
		ret = 0;
	}

	if (ret == 0) {
		ret = usb_submit_urb(urb, GFP_ATOMIC);
		if (ret < 0)
			pr_err("Resubmit URB failed with error %i\n", ret);
	}
}

static int pinect_input_connect(struct pinect_dev *dev)
{
	struct input_dev *input_dev;
	int err = 0;

	dev->input_dev = NULL;
	if (dev->sd_desc->int_pkt_scan || dev->sd_desc->other_input)  {
		input_dev = input_allocate_device();
		if (!input_dev)
			return -ENOMEM;

		usb_make_path(dev->dev, dev->phys, sizeof(dev->phys));
		strlcat(dev->phys, "/input0", sizeof(dev->phys));

		input_dev->name = dev->sd_desc->name;
		input_dev->phys = dev->phys;

		usb_to_input_id(dev->dev, &input_dev->id);

		input_dev->evbit[0] = BIT_MASK(EV_KEY);
		input_dev->keybit[BIT_WORD(KEY_CAMERA)] = BIT_MASK(KEY_CAMERA);
		input_dev->dev.parent = &dev->dev->dev;

		err = input_register_device(input_dev);
		if (err) {
			pr_err("Input device registration failed with error %i\n",
			       err);
			input_dev->dev.parent = NULL;
			input_free_device(input_dev);
		} else {
			dev->input_dev = input_dev;
		}
	}

	return err;
}

static int alloc_and_submit_int_urb(struct pinect_dev *pinect_dev,
			  struct usb_endpoint_descriptor *ep)
{
	unsigned int buffer_len;
	int interval;
	struct urb *urb;
	struct usb_device *dev;
	void *buffer = NULL;
	int ret = -EINVAL;

	buffer_len = le16_to_cpu(ep->wMaxPacketSize);
	interval = ep->bInterval;
	PDEBUG(D_CONF, "found int in endpoint: 0x%x, "
		"buffer_len=%u, interval=%u",
		ep->bEndpointAddress, buffer_len, interval);

	dev = pinect_dev->dev;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		ret = -ENOMEM;
		goto error;
	}

	buffer = usb_alloc_coherent(dev, buffer_len,
				GFP_KERNEL, &urb->transfer_dma);
	if (!buffer) {
		ret = -ENOMEM;
		goto error_buffer;
	}
	usb_fill_int_urb(urb, dev,
		usb_rcvintpipe(dev, ep->bEndpointAddress),
		buffer, buffer_len,
		int_irq, (void *)pinect_dev, interval);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret < 0) {
		PERR("submit int URB failed with error %i", ret);
		goto error_submit;
	}
	pinect_dev->int_urb = urb;
	return ret;

error_submit:
	usb_free_coherent(dev,
			  urb->transfer_buffer_length,
			  urb->transfer_buffer,
			  urb->transfer_dma);
error_buffer:
	usb_free_urb(urb);
error:
	return ret;
}

static void pinect_input_create_urb(struct pinect_dev *pinect_dev)
{
	struct usb_interface *intf;
	struct usb_host_interface *intf_desc;
	struct usb_endpoint_descriptor *ep;
	int i;

	if (pinect_dev->sd_desc->int_pkt_scan)  {
		intf = usb_ifnum_to_if(pinect_dev->dev, pinect_dev->iface);
		intf_desc = intf->cur_altsetting;
		for (i = 0; i < intf_desc->desc.bNumEndpoints; i++) {
			ep = &intf_desc->endpoint[i].desc;
			if (usb_endpoint_dir_in(ep) &&
			    usb_endpoint_xfer_int(ep)) {

				alloc_and_submit_int_urb(pinect_dev, ep);
				break;
			}
		}
	}
}

static void pinect_input_destroy_urb(struct pinect_dev *pinect_dev)
{
	struct urb *urb;

	urb = pinect_dev->int_urb;
	if (urb) {
		pinect_dev->int_urb = NULL;
		usb_kill_urb(urb);
		usb_free_coherent(pinect_dev->dev,
				  urb->transfer_buffer_length,
				  urb->transfer_buffer,
				  urb->transfer_dma);
		usb_free_urb(urb);
	}
}
#else
static inline void pinect_input_destroy_urb(struct pinect_dev *pinect_dev)
{
}

static inline void pinect_input_create_urb(struct pinect_dev *pinect_dev)
{
}

static inline int pinect_input_connect(struct pinect_dev *dev)
{
	return 0;
}
#endif

/*
 * fill a video frame from an URB and resubmit
 */
static void fill_frame(struct pinect_dev *pinect_dev,
			struct urb *urb)
{
	u8 *data;		/* address of data in the iso message */
	int i, len, st;
	cam_pkt_op pkt_scan;

	if (urb->status != 0) {
		if (urb->status == -ESHUTDOWN)
			return;		/* disconnection */
#ifdef CONFIG_PM
		if (pinect_dev->frozen)
			return;
#endif
		PERR("urb status: %d", urb->status);
		urb->status = 0;
		goto resubmit;
	}
	pkt_scan = pinect_dev->sd_desc->pkt_scan;
	for (i = 0; i < urb->number_of_packets; i++) {
		len = urb->iso_frame_desc[i].actual_length;

		/* check the packet status and length */
		st = urb->iso_frame_desc[i].status;
		if (st) {
			pr_err("ISOC data error: [%d] len=%d, status=%d\n",
			       i, len, st);
			pinect_dev->last_packet_type = DISCARD_PACKET;
			continue;
		}
		if (len == 0) {
			if (pinect_dev->empty_packet == 0)
				pinect_dev->empty_packet = 1;
			continue;
		}

		/* let the packet be analyzed by the subdriver */
		PDEBUG(D_PACK, "packet [%d] o:%d l:%d",
			i, urb->iso_frame_desc[i].offset, len);
		data = (u8 *) urb->transfer_buffer
					+ urb->iso_frame_desc[i].offset;
		pkt_scan(pinect_dev, data, len);
	}

resubmit:
	/* resubmit the URB */
	st = usb_submit_urb(urb, GFP_ATOMIC);
	if (st < 0)
		pr_err("usb_submit_urb() ret %d\n", st);
}

/*
 * ISOC message interrupt from the USB device
 *
 * Analyse each packet and call the subdriver for copy to the frame buffer.
 */
static void isoc_irq(struct urb *urb)
{
	struct pinect_dev *pinect_dev = (struct pinect_dev *) urb->context;

	PDEBUG(D_PACK, "isoc irq");
	if (!pinect_dev->streaming)
		return;
	fill_frame(pinect_dev, urb);
}

/*
 * bulk message interrupt from the USB device
 */
static void bulk_irq(struct urb *urb)
{
	struct pinect_dev *pinect_dev = (struct pinect_dev *) urb->context;
	int st;

	PDEBUG(D_PACK, "bulk irq");
	if (!pinect_dev->streaming)
		return;
	switch (urb->status) {
	case 0:
		break;
	case -ESHUTDOWN:
		return;		/* disconnection */
	default:
#ifdef CONFIG_PM
		if (pinect_dev->frozen)
			return;
#endif
		PERR("urb status: %d", urb->status);
		urb->status = 0;
		goto resubmit;
	}

	PDEBUG(D_PACK, "packet l:%d", urb->actual_length);
	pinect_dev->sd_desc->pkt_scan(pinect_dev,
				urb->transfer_buffer,
				urb->actual_length);

resubmit:
	/* resubmit the URB */
	if (pinect_dev->cam.bulk_nurbs != 0) {
		st = usb_submit_urb(urb, GFP_ATOMIC);
		if (st < 0)
			pr_err("usb_submit_urb() ret %d\n", st);
	}
}

/*
 * add data to the current frame
 *
 * This function is called by the subdrivers at interrupt level.
 *
 * To build a frame, these ones must add
 *	- one FIRST_PACKET
 *	- 0 or many INTER_PACKETs
 *	- one LAST_PACKET
 * DISCARD_PACKET invalidates the whole frame.
 */

static void chunk(void *buffer, const u8  *pkt_data, uint32_t pkt_num, int pkt_size)
{
	int n;
	uint8_t *raw = (uint8_t *) pkt_data;
	uint16_t *frame=(uint16_t *)buffer;
	
	if(pkt_num > 219){
		raw += (pkt_num-220) * 12;
	}else if(pkt_num > 146){
		raw += (pkt_num-147) * 12 + 4;
	}else if(pkt_num > 73){
		raw += (pkt_num-74) * 12 + 8;
	}else{
		raw += pkt_num * 12;
	}
	for(n=40;n--;){
		frame[0] =  depth_lt[(raw[0]<<3)  | (raw[1]>>5)];
        frame[1] = depth_lt[((raw[2]<<9)  | (raw[3]<<1) | (raw[4]>>7) ) & 2047];
        frame[2] = depth_lt[((raw[5]<<7)  | (raw[6]>>1) ) & 2047];
        frame[3] = depth_lt[((raw[8]<<5)  | (raw[9]>>3) ) & 2047];
        frame[4] =  depth_lt[(raw[11]<<3)  | (raw[12]>>5)];
        frame[5] = depth_lt[((raw[13]<<9)  | (raw[14]<<1) | (raw[15]>>7) ) & 2047];
        frame[6] = depth_lt[((raw[16]<<7)  | (raw[17]>>1) ) & 2047];
        frame[7] = depth_lt[((raw[19]<<5)  | (raw[20]>>3) ) & 2047];
        frame+=8;
        raw+=22;
    }
}

void pinect_frame_add(struct pinect_dev *pinect_dev,
			enum pinect_packet_type packet_type,
			const u8 *data,
			int len,
			uint32_t pkt_num,
			int rawlen, int diff)
{
	struct pinect_frame *frame;
	int i, j;

	PDEBUG(D_PACK, "add t:%d l:%d",	packet_type, len);

	if (packet_type == FIRST_PACKET) {
		i = atomic_read(&pinect_dev->fr_i);

		/* if there are no queued buffer, discard the whole frame */
		if (i == atomic_read(&pinect_dev->fr_q)) {
			pinect_dev->last_packet_type = DISCARD_PACKET;
			pinect_dev->sequence++;
			return;
		}
		j = pinect_dev->fr_queue[i];
		frame = &pinect_dev->frame[j];
		frame->v4l2_buf.timestamp = ktime_to_timeval(ktime_get());
		frame->v4l2_buf.sequence = pinect_dev->sequence++;
		pinect_dev->image = frame->data;
		pinect_dev->image_len = 0;
	} else {
		switch (pinect_dev->last_packet_type) {
		case DISCARD_PACKET:
			if (packet_type == LAST_PACKET) {
				pinect_dev->last_packet_type = packet_type;
				pinect_dev->image = NULL;
				pinect_dev->image_len = 0;
			}
			return;
		case LAST_PACKET:
			return;
		}
	}

	/* append the packet to the frame buffer */
	if (len > 0) {
		if (pinect_dev->image_len + len > pinect_dev->frsz) {
			PERR("frame overflow %d > %d",
				pinect_dev->image_len + len,
				pinect_dev->frsz);
			packet_type = DISCARD_PACKET;
		} else {
/* !! image is NULL only when last pkt is LAST or DISCARD
			if (pinect_dev->image == NULL) {
				pr_err("pinect_frame_add() image == NULL\n");
				return;
			}
 */
			if(pkt_num != 73 && pkt_num != 146 && diff >= 0){
				chunk(pinect_dev->image + diff, data, pkt_num, rawlen);
				pinect_dev->image_len = diff+len;
			}
	
		}
	}
	pinect_dev->last_packet_type = packet_type;

	/* if last packet, invalidate packet concatenation until
	 * next first packet, wake up the application and advance
	 * in the queue */
	if (packet_type == LAST_PACKET) {
		i = atomic_read(&pinect_dev->fr_i);
		j = pinect_dev->fr_queue[i];
		frame = &pinect_dev->frame[j];
		frame->v4l2_buf.bytesused = 320*240*2;
		frame->v4l2_buf.flags = (frame->v4l2_buf.flags
					 | V4L2_BUF_FLAG_DONE)
					& ~V4L2_BUF_FLAG_QUEUED;
		i = (i + 1) % pinect_MAX_FRAMES;
		atomic_set(&pinect_dev->fr_i, i);
		wake_up_interruptible(&pinect_dev->wq);	/* event = new frame */
		PDEBUG(D_FRAM, "frame complete len:%d",
			frame->v4l2_buf.bytesused);
		pinect_dev->image = NULL;
		pinect_dev->image_len = 0;
	}
}

static int frame_alloc(struct pinect_dev *pinect_dev, struct file *file,
			enum v4l2_memory memory, unsigned int count)
{
	struct pinect_frame *frame;
	unsigned int frsz;
	int i;

	i = pinect_dev->curr_mode;
	frsz = pinect_dev->cam.cam_mode[i].sizeimage;
	PDEBUG(D_STREAM, "frame alloc frsz: %d", frsz);
	frsz = PAGE_ALIGN(frsz);
	if (count >= pinect_MAX_FRAMES)
		count = pinect_MAX_FRAMES - 1;
	pinect_dev->frbuf = vmalloc_32(frsz * count);
	if (!pinect_dev->frbuf) {
		pr_err("frame alloc failed\n");
		return -ENOMEM;
	}
	pinect_dev->capt_file = file;
	pinect_dev->memory = memory;
	pinect_dev->frsz = frsz;
	pinect_dev->nframes = count;
	for (i = 0; i < count; i++) {
		frame = &pinect_dev->frame[i];
		frame->v4l2_buf.index = i;
		frame->v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		frame->v4l2_buf.flags = 0;
		frame->v4l2_buf.field = V4L2_FIELD_NONE;
		frame->v4l2_buf.length = frsz;
		frame->v4l2_buf.memory = memory;
		frame->v4l2_buf.sequence = 0;
		frame->data = pinect_dev->frbuf + i * frsz;
		frame->v4l2_buf.m.offset = i * frsz;
	}
	atomic_set(&pinect_dev->fr_q, 0);
	atomic_set(&pinect_dev->fr_i, 0);
	pinect_dev->fr_o = 0;
	return 0;
}

static void frame_free(struct pinect_dev *pinect_dev)
{
	int i;

	PDEBUG(D_STREAM, "frame free");
	if (pinect_dev->frbuf != NULL) {
		vfree(pinect_dev->frbuf);
		pinect_dev->frbuf = NULL;
		for (i = 0; i < pinect_dev->nframes; i++)
			pinect_dev->frame[i].data = NULL;
	}
	pinect_dev->nframes = 0;
	pinect_dev->frsz = 0;
	pinect_dev->capt_file = NULL;
	pinect_dev->memory = pinect_MEMORY_NO;
}

static void destroy_urbs(struct pinect_dev *pinect_dev)
{
	struct urb *urb;
	unsigned int i;

	PDEBUG(D_STREAM, "kill transfer");
	for (i = 0; i < MAX_NURBS; i++) {
		urb = pinect_dev->urb[i];
		if (urb == NULL)
			break;

		pinect_dev->urb[i] = NULL;
		usb_kill_urb(urb);
		usb_free_coherent(pinect_dev->dev,
				  urb->transfer_buffer_length,
				  urb->transfer_buffer,
				  urb->transfer_dma);
		usb_free_urb(urb);
	}
}

static int pinect_set_alt0(struct pinect_dev *pinect_dev)
{
	int ret;

	if (pinect_dev->alt == 0)
		return 0;
	ret = usb_set_interface(pinect_dev->dev, pinect_dev->iface, 0);
	if (ret < 0)
		pr_err("set alt 0 err %d\n", ret);
	return ret;
}

/* Note: both the queue and the usb locks should be held when calling this */
static void pinect_stream_off(struct pinect_dev *pinect_dev)
{
	pinect_dev->streaming = 0;
	pinect_dev->usb_err = 0;
	if (pinect_dev->sd_desc->stopN)
		pinect_dev->sd_desc->stopN(pinect_dev);
	destroy_urbs(pinect_dev);
	pinect_input_destroy_urb(pinect_dev);
	pinect_set_alt0(pinect_dev);
	pinect_input_create_urb(pinect_dev);
	if (pinect_dev->sd_desc->stop0)
		pinect_dev->sd_desc->stop0(pinect_dev);
	PDEBUG(D_STREAM, "stream off OK");
}

/*
 * look for an input transfer endpoint in an alternate setting
 * patched to get _only_ the second endpoint
 */
static struct usb_host_endpoint *alt_xfer(struct usb_host_interface *alt,
					  int xfer)
{
	struct usb_host_endpoint *ep;
	int i, attr, ep_nr;

	ep_nr=0;
	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		ep = &alt->endpoint[i];
		attr = ep->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
		if (attr == xfer
		    && ep->desc.wMaxPacketSize != 0
		    && usb_endpoint_dir_in(&ep->desc)){
				if(ep_nr > 0){
									
					return ep;
				}
				else
					ep_nr++;
		}
	}
	return NULL;
}

/* compute the minimum bandwidth for the current transfer */
static u32 which_bandwidth(struct pinect_dev *pinect_dev)
{
	u32 bandwidth;
	int i;

	/* get the (max) image size */
	i = pinect_dev->curr_mode;
	bandwidth = pinect_dev->cam.cam_mode[i].sizeimage;

	/* if the image is compressed, estimate its mean size */
	if (!pinect_dev->cam.needs_full_bandwidth &&
	    bandwidth < pinect_dev->cam.cam_mode[i].width *
				pinect_dev->cam.cam_mode[i].height)
		bandwidth = bandwidth * 3 / 8;	/* 0.375 */

	/* estimate the frame rate */
	if (pinect_dev->sd_desc->get_streamparm) {
		struct v4l2_streamparm parm;

		pinect_dev->sd_desc->get_streamparm(pinect_dev, &parm);
		bandwidth *= parm.parm.capture.timeperframe.denominator;
		bandwidth /= parm.parm.capture.timeperframe.numerator;
	} else {

		/* don't hope more than 15 fps with USB 1.1 and
		 * image resolution >= 640x480 */
		if (pinect_dev->width >= 640
		 && pinect_dev->dev->speed == USB_SPEED_FULL)
			bandwidth *= 15;		/* 15 fps */
		else
			bandwidth *= 30;		/* 30 fps */
	}

	PDEBUG(D_STREAM, "min bandwidth: %d", bandwidth);
	return bandwidth;
}

/* endpoint table */
#define MAX_ALT 16
struct ep_tb_s {
	u32 alt;
	u32 bandwidth;
};

/*
 * build the table of the endpoints
 * and compute the minimum bandwidth for the image transfer
 */
static int build_isoc_ep_tb(struct pinect_dev *pinect_dev,
			struct usb_interface *intf,
			struct ep_tb_s *ep_tb)
{
	struct usb_host_endpoint *ep;
	int i, j, nbalt, psize, found;
	u32 bandwidth, last_bw;

	nbalt = intf->num_altsetting;
	if (nbalt > MAX_ALT)
		nbalt = MAX_ALT;	/* fixme: should warn */

	/* build the endpoint table */
	i = 0;
	last_bw = 0;
	for (;;) {
		ep_tb->bandwidth = 2000 * 2000 * 120;
		found = 0;
		for (j = 0; j < nbalt; j++) {
			ep = alt_xfer(&intf->altsetting[j],
				      USB_ENDPOINT_XFER_ISOC);
			if (ep == NULL)
				continue;
			if (ep->desc.bInterval == 0) {
				pr_err("alt %d iso endp with 0 interval\n", j);
				continue;
			}
			psize = le16_to_cpu(ep->desc.wMaxPacketSize);
			psize = (psize & 0x07ff) * (1 + ((psize >> 11) & 3));
			bandwidth = psize * 1000;
			if (pinect_dev->dev->speed == USB_SPEED_HIGH
			 || pinect_dev->dev->speed == USB_SPEED_SUPER)
				bandwidth *= 8;
			bandwidth /= 1 << (ep->desc.bInterval - 1);
			if (bandwidth <= last_bw)
				continue;
			if (bandwidth < ep_tb->bandwidth) {
				ep_tb->bandwidth = bandwidth;
				ep_tb->alt = j;
				found = 1;
			}
		}
		if (!found)
			break;
		PDEBUG(D_STREAM, "alt %d bandwidth %d",
				ep_tb->alt, ep_tb->bandwidth);
		last_bw = ep_tb->bandwidth;
		i++;
		ep_tb++;
	}

	/*
	 * If the camera:
	 * has a usb audio class interface (a built in usb mic); and
	 * is a usb 1 full speed device; and
	 * uses the max full speed iso bandwidth; and
	 * and has more than 1 alt setting
	 * then skip the highest alt setting to spare bandwidth for the mic
	 */
	if (pinect_dev->audio &&
			pinect_dev->dev->speed == USB_SPEED_FULL &&
			last_bw >= 1000000 &&
			i > 1) {
		PDEBUG(D_STREAM, "dev has usb audio, skipping highest alt");
		i--;
		ep_tb--;
	}

	/* get the requested bandwidth and start at the highest atlsetting */
	bandwidth = which_bandwidth(pinect_dev);
	ep_tb--;
	while (i > 1) {
		ep_tb--;
		if (ep_tb->bandwidth < bandwidth)
			break;
		i--;
	}
	return i;
}

/*
 * create the URBs for image transfer
 */
static int create_urbs(struct pinect_dev *pinect_dev,
			struct usb_host_endpoint *ep)
{
	struct urb *urb;
	int n, nurbs, i, psize, npkt, bsize;

	/* calculate the packet size and the number of packets */
	psize = le16_to_cpu(ep->desc.wMaxPacketSize);

	if (!pinect_dev->cam.bulk) {		/* isoc */

		/* See paragraph 5.9 / table 5-11 of the usb 2.0 spec. */
		if (pinect_dev->pkt_size == 0)
			psize = (psize & 0x07ff) * (1 + ((psize >> 11) & 3));
		else
			psize = pinect_dev->pkt_size;
		npkt = pinect_dev->cam.npkt;
		if (npkt == 0)
			npkt = 32;		/* default value */
		bsize = psize * npkt;
		PDEBUG(D_STREAM,
			"isoc %d pkts size %d = bsize:%d",
			npkt, psize, bsize);
		pr_warn("isoc %d pkts size %d = bsize:%d",
			npkt, psize, bsize);
		nurbs = DEF_NURBS;
	} else {				/* bulk */
		npkt = 0;
		bsize = pinect_dev->cam.bulk_size;
		if (bsize == 0)
			bsize = psize;
		PDEBUG(D_STREAM, "bulk bsize:%d", bsize);
		if (pinect_dev->cam.bulk_nurbs != 0)
			nurbs = pinect_dev->cam.bulk_nurbs;
		else
			nurbs = 1;
	}

	for (n = 0; n < nurbs; n++) {
		urb = usb_alloc_urb(npkt, GFP_KERNEL);
		if (!urb) {
			pr_err("usb_alloc_urb failed\n");
			return -ENOMEM;
		}
		pinect_dev->urb[n] = urb;
		urb->transfer_buffer = usb_alloc_coherent(pinect_dev->dev,
						bsize,
						GFP_KERNEL,
						&urb->transfer_dma);

		if (urb->transfer_buffer == NULL) {
			pr_err("usb_alloc_coherent failed\n");
			return -ENOMEM;
		}
		urb->dev = pinect_dev->dev;
		urb->context = pinect_dev;
		urb->transfer_buffer_length = bsize;
		if (npkt != 0) {		/* ISOC */
			urb->pipe = usb_rcvisocpipe(pinect_dev->dev,
						    ep->desc.bEndpointAddress);
			urb->transfer_flags = URB_ISO_ASAP
					| URB_NO_TRANSFER_DMA_MAP;
			urb->interval = 1 << (ep->desc.bInterval - 1);
			urb->complete = isoc_irq;
			urb->number_of_packets = npkt;
			for (i = 0; i < npkt; i++) {
				urb->iso_frame_desc[i].length = psize;
				urb->iso_frame_desc[i].offset = psize * i;
			}
		} else {		/* bulk */
			urb->pipe = usb_rcvbulkpipe(pinect_dev->dev,
						ep->desc.bEndpointAddress);
			urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
			urb->complete = bulk_irq;
		}
	}
	return 0;
}

/*
 * start the USB transfer
 */
static int pinect_init_transfer(struct pinect_dev *pinect_dev)
{
	struct usb_interface *intf;
	struct usb_host_endpoint *ep;
	struct urb *urb;
	struct ep_tb_s ep_tb[MAX_ALT];
	int n, ret, xfer, alt, alt_idx;

	/* reset the streaming variables */
	pinect_dev->image = NULL;
	pinect_dev->image_len = 0;
	pinect_dev->last_packet_type = DISCARD_PACKET;
	pinect_dev->sequence = 0;

	pinect_dev->usb_err = 0;

	/* do the specific subdriver stuff before endpoint selection */
	intf = usb_ifnum_to_if(pinect_dev->dev, pinect_dev->iface);
	pinect_dev->alt = pinect_dev->cam.bulk ? intf->num_altsetting : 0;
	if (pinect_dev->sd_desc->isoc_init) {
		ret = pinect_dev->sd_desc->isoc_init(pinect_dev);
		if (ret < 0)
			return ret;
	}
	xfer = pinect_dev->cam.bulk ? USB_ENDPOINT_XFER_BULK
				   : USB_ENDPOINT_XFER_ISOC;

	/* if bulk or the subdriver forced an altsetting, get the endpoint */
	if (pinect_dev->alt != 0) {
		pinect_dev->alt--;	/* (previous version compatibility) */
		ep = alt_xfer(&intf->altsetting[pinect_dev->alt], xfer);
		if (ep == NULL) {
			pr_err("bad altsetting %d\n", pinect_dev->alt);
			return -EIO;
		}
		ep_tb[0].alt = pinect_dev->alt;
		alt_idx = 1;
	} else {

	/* else, compute the minimum bandwidth
	 * and build the endpoint table */
		alt_idx = build_isoc_ep_tb(pinect_dev, intf, ep_tb);
		if (alt_idx <= 0) {
			pr_err("no transfer endpoint found\n");
			return -EIO;
		}
	}

	/* set the highest alternate setting and
	 * loop until urb submit succeeds */
	pinect_input_destroy_urb(pinect_dev);

	pinect_dev->alt = ep_tb[--alt_idx].alt;
	alt = -1;
	for (;;) {
		if (alt != pinect_dev->alt) {
			alt = pinect_dev->alt;
			if (intf->num_altsetting > 1) {
				ret = usb_set_interface(pinect_dev->dev,
							pinect_dev->iface,
							alt);
				if (ret < 0) {
					if (ret == -ENOSPC)
						goto retry; /*fixme: ugly*/
					pr_err("set alt %d err %d\n", alt, ret);
					goto out;
				}
			}
		}
		if (!pinect_dev->cam.no_urb_create) {
			PDEBUG(D_STREAM, "init transfer alt %d", alt);
			ret = create_urbs(pinect_dev,
				alt_xfer(&intf->altsetting[alt], xfer));
			if (ret < 0) {
				destroy_urbs(pinect_dev);
				goto out;
			}
		}

		/* clear the bulk endpoint */
		if (pinect_dev->cam.bulk)
			usb_clear_halt(pinect_dev->dev,
					pinect_dev->urb[0]->pipe);

		/* start the cam */
		ret = pinect_dev->sd_desc->start(pinect_dev);
		if (ret < 0) {
			destroy_urbs(pinect_dev);
			goto out;
		}
		pinect_dev->streaming = 1;
		v4l2_ctrl_handler_setup(pinect_dev->vdev.ctrl_handler);

		/* some bulk transfers are started by the subdriver */
		if (pinect_dev->cam.bulk && pinect_dev->cam.bulk_nurbs == 0)
			break;

		/* submit the URBs */
		for (n = 0; n < MAX_NURBS; n++) {
			urb = pinect_dev->urb[n];
			if (urb == NULL)
				break;
			ret = usb_submit_urb(urb, GFP_KERNEL);
			if (ret < 0)
				break;
		}
		if (ret >= 0)
			break;			/* transfer is started */

		/* something when wrong
		 * stop the webcam and free the transfer resources */
		pinect_stream_off(pinect_dev);
		if (ret != -ENOSPC) {
			pr_err("usb_submit_urb alt %d err %d\n",
			       pinect_dev->alt, ret);
			goto out;
		}

		/* the bandwidth is not wide enough
		 * negotiate or try a lower alternate setting */
retry:
		PERR("alt %d - bandwidth not wide enough, trying again", alt);
		msleep(20);	/* wait for kill complete */
		if (pinect_dev->sd_desc->isoc_nego) {
			ret = pinect_dev->sd_desc->isoc_nego(pinect_dev);
			if (ret < 0)
				goto out;
		} else {
			if (alt_idx <= 0) {
				pr_err("no transfer endpoint found\n");
				ret = -EIO;
				goto out;
			}
			pinect_dev->alt = ep_tb[--alt_idx].alt;
		}
	}
out:
	pinect_input_create_urb(pinect_dev);
	return ret;
}

static void pinect_set_default_mode(struct pinect_dev *pinect_dev)
{
	int i;

	i = pinect_dev->cam.nmodes - 1;	/* take the highest mode */
	pinect_dev->curr_mode = i;
	pinect_dev->width = pinect_dev->cam.cam_mode[i].width;
	pinect_dev->height = pinect_dev->cam.cam_mode[i].height;
	pinect_dev->pixfmt = pinect_dev->cam.cam_mode[i].pixelformat;

	/* does nothing if ctrl_handler == NULL */
	v4l2_ctrl_handler_setup(pinect_dev->vdev.ctrl_handler);
}

static int wxh_to_mode(struct pinect_dev *pinect_dev,
			int width, int height)
{
	int i;

	for (i = pinect_dev->cam.nmodes; --i > 0; ) {
		if (width >= pinect_dev->cam.cam_mode[i].width
		    && height >= pinect_dev->cam.cam_mode[i].height)
			break;
	}
	return i;
}

/*
 * search a mode with the right pixel format
 */
static int pinect_get_mode(struct pinect_dev *pinect_dev,
			int mode,
			int pixfmt)
{
	int modeU, modeD;

	modeU = modeD = mode;
	while ((modeU < pinect_dev->cam.nmodes) || modeD >= 0) {
		if (--modeD >= 0) {
			if (pinect_dev->cam.cam_mode[modeD].pixelformat
								== pixfmt)
				return modeD;
		}
		if (++modeU < pinect_dev->cam.nmodes) {
			if (pinect_dev->cam.cam_mode[modeU].pixelformat
								== pixfmt)
				return modeU;
		}
	}
	return -EINVAL;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vidioc_g_chip_info(struct file *file, void *priv,
				struct v4l2_dbg_chip_info *chip)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	pinect_dev->usb_err = 0;
	if (pinect_dev->sd_desc->get_chip_info)
		return pinect_dev->sd_desc->get_chip_info(pinect_dev, chip);
	return chip->match.addr ? -EINVAL : 0;
}

static int vidioc_g_register(struct file *file, void *priv,
		struct v4l2_dbg_register *reg)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	pinect_dev->usb_err = 0;
	return pinect_dev->sd_desc->get_register(pinect_dev, reg);
}

static int vidioc_s_register(struct file *file, void *priv,
		const struct v4l2_dbg_register *reg)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	pinect_dev->usb_err = 0;
	return pinect_dev->sd_desc->set_register(pinect_dev, reg);
}
#endif

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
				struct v4l2_fmtdesc *fmtdesc)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	int i, j, index;
	__u32 fmt_tb[8];

	/* give an index to each format */
	index = 0;
	j = 0;
	for (i = pinect_dev->cam.nmodes; --i >= 0; ) {
		fmt_tb[index] = pinect_dev->cam.cam_mode[i].pixelformat;
		j = 0;
		for (;;) {
			if (fmt_tb[j] == fmt_tb[index])
				break;
			j++;
		}
		if (j == index) {
			if (fmtdesc->index == index)
				break;		/* new format */
			index++;
			if (index >= ARRAY_SIZE(fmt_tb))
				return -EINVAL;
		}
	}
	if (i < 0)
		return -EINVAL;		/* no more format */

	fmtdesc->pixelformat = fmt_tb[index];
	if (pinect_dev->cam.cam_mode[i].sizeimage <
			pinect_dev->cam.cam_mode[i].width *
				pinect_dev->cam.cam_mode[i].height)
		fmtdesc->flags = V4L2_FMT_FLAG_COMPRESSED;
	fmtdesc->description[0] = fmtdesc->pixelformat & 0xff;
	fmtdesc->description[1] = (fmtdesc->pixelformat >> 8) & 0xff;
	fmtdesc->description[2] = (fmtdesc->pixelformat >> 16) & 0xff;
	fmtdesc->description[3] = fmtdesc->pixelformat >> 24;
	fmtdesc->description[4] = '\0';
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
			    struct v4l2_format *fmt)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	int mode;

	mode = pinect_dev->curr_mode;
	fmt->fmt.pix = pinect_dev->cam.cam_mode[mode];
	/* some drivers use priv internally, zero it before giving it to
	   userspace */
	fmt->fmt.pix.priv = 0;
	return 0;
}

static int try_fmt_vid_cap(struct pinect_dev *pinect_dev,
			struct v4l2_format *fmt)
{
	int w, h, mode, mode2;

	w = fmt->fmt.pix.width;
	h = fmt->fmt.pix.height;

	PDEBUG_MODE(pinect_dev, D_CONF, "try fmt cap",
		    fmt->fmt.pix.pixelformat, w, h);

	/* search the closest mode for width and height */
	mode = wxh_to_mode(pinect_dev, w, h);

	/* OK if right palette */
	if (pinect_dev->cam.cam_mode[mode].pixelformat
						!= fmt->fmt.pix.pixelformat) {

		/* else, search the closest mode with the same pixel format */
		mode2 = pinect_get_mode(pinect_dev, mode,
					fmt->fmt.pix.pixelformat);
		if (mode2 >= 0)
			mode = mode2;
	}
	fmt->fmt.pix = pinect_dev->cam.cam_mode[mode];
	/* some drivers use priv internally, zero it before giving it to
	   userspace */
	fmt->fmt.pix.priv = 0;
	return mode;			/* used when s_fmt */
}

static int vidioc_try_fmt_vid_cap(struct file *file,
			      void *priv,
			      struct v4l2_format *fmt)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	int ret;

	ret = try_fmt_vid_cap(pinect_dev, fmt);
	if (ret < 0)
		return ret;
	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
			    struct v4l2_format *fmt)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	int ret;

	if (mutex_lock_interruptible(&pinect_dev->queue_lock))
		return -ERESTARTSYS;

	ret = try_fmt_vid_cap(pinect_dev, fmt);
	if (ret < 0)
		goto out;

	if (pinect_dev->nframes != 0
	    && fmt->fmt.pix.sizeimage > pinect_dev->frsz) {
		ret = -EINVAL;
		goto out;
	}

	if (ret == pinect_dev->curr_mode) {
		ret = 0;
		goto out;			/* same mode */
	}

	if (pinect_dev->streaming) {
		ret = -EBUSY;
		goto out;
	}
	pinect_dev->width = fmt->fmt.pix.width;
	pinect_dev->height = fmt->fmt.pix.height;
	pinect_dev->pixfmt = fmt->fmt.pix.pixelformat;
	pinect_dev->curr_mode = ret;

	ret = 0;
out:
	mutex_unlock(&pinect_dev->queue_lock);
	return ret;
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	int i;
	__u32 index = 0;

	for (i = 0; i < pinect_dev->cam.nmodes; i++) {
		if (fsize->pixel_format !=
				pinect_dev->cam.cam_mode[i].pixelformat)
			continue;

		if (fsize->index == index) {
			fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
			fsize->discrete.width =
				pinect_dev->cam.cam_mode[i].width;
			fsize->discrete.height =
				pinect_dev->cam.cam_mode[i].height;
			return 0;
		}
		index++;
	}

	return -EINVAL;
}

static int vidioc_enum_frameintervals(struct file *filp, void *priv,
				      struct v4l2_frmivalenum *fival)
{
	struct pinect_dev *pinect_dev = video_drvdata(filp);
	int mode = wxh_to_mode(pinect_dev, fival->width, fival->height);
	__u32 i;

	if (pinect_dev->cam.mode_framerates == NULL ||
			pinect_dev->cam.mode_framerates[mode].nrates == 0)
		return -EINVAL;

	if (fival->pixel_format !=
			pinect_dev->cam.cam_mode[mode].pixelformat)
		return -EINVAL;

	for (i = 0; i < pinect_dev->cam.mode_framerates[mode].nrates; i++) {
		if (fival->index == i) {
			fival->type = V4L2_FRMSIZE_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator =
				pinect_dev->cam.mode_framerates[mode].rates[i];
			return 0;
		}
	}

	return -EINVAL;
}

static void pinect_release(struct v4l2_device *v4l2_device)
{
	struct pinect_dev *pinect_dev =
		container_of(v4l2_device, struct pinect_dev, v4l2_dev);

	v4l2_ctrl_handler_free(pinect_dev->vdev.ctrl_handler);
	v4l2_device_unregister(&pinect_dev->v4l2_dev);
	kfree(pinect_dev->usb_buf);
	kfree(pinect_dev);
}

static int dev_open(struct file *file)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	PDEBUG(D_STREAM, "[%s] open", current->comm);

	/* protect the subdriver against rmmod */
	if (!try_module_get(pinect_dev->module))
		return -ENODEV;

	return v4l2_fh_open(file);
}

static int dev_close(struct file *file)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	PDEBUG(D_STREAM, "[%s] close", current->comm);

	/* Needed for pinect_stream_off, always lock before queue_lock! */
	if (mutex_lock_interruptible(&pinect_dev->usb_lock))
		return -ERESTARTSYS;

	if (mutex_lock_interruptible(&pinect_dev->queue_lock)) {
		mutex_unlock(&pinect_dev->usb_lock);
		return -ERESTARTSYS;
	}

	/* if the file did the capture, free the streaming resources */
	if (pinect_dev->capt_file == file) {
		if (pinect_dev->streaming)
			pinect_stream_off(pinect_dev);
		frame_free(pinect_dev);
	}
	module_put(pinect_dev->module);
	mutex_unlock(&pinect_dev->queue_lock);
	mutex_unlock(&pinect_dev->usb_lock);

	PDEBUG(D_STREAM, "close done");

	return v4l2_fh_release(file);
}

static int vidioc_querycap(struct file *file, void  *priv,
			   struct v4l2_capability *cap)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	strlcpy((char *) cap->driver, pinect_dev->sd_desc->name,
			sizeof cap->driver);
	if (pinect_dev->dev->product != NULL) {
		strlcpy((char *) cap->card, pinect_dev->dev->product,
			sizeof cap->card);
	} else {
		snprintf((char *) cap->card, sizeof cap->card,
			"USB Camera (%04x:%04x)",
			le16_to_cpu(pinect_dev->dev->descriptor.idVendor),
			le16_to_cpu(pinect_dev->dev->descriptor.idProduct));
	}
	usb_make_path(pinect_dev->dev, (char *) cap->bus_info,
			sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE
			  | V4L2_CAP_STREAMING
			  | V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vidioc_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	if (input->index != 0)
		return -EINVAL;
	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->status = pinect_dev->cam.input_flags;
	strlcpy(input->name, pinect_dev->sd_desc->name,
		sizeof input->name);
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;
	return (0);
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *rb)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	int i, ret = 0, streaming;

	i = rb->memory;			/* (avoid compilation warning) */
	switch (i) {
	case pinect_MEMORY_READ:			/* (internal call) */
	case V4L2_MEMORY_MMAP:
	case V4L2_MEMORY_USERPTR:
		break;
	default:
		return -EINVAL;
	}
	if (mutex_lock_interruptible(&pinect_dev->queue_lock))
		return -ERESTARTSYS;

	if (pinect_dev->memory != pinect_MEMORY_NO
	    && pinect_dev->memory != pinect_MEMORY_READ
	    && pinect_dev->memory != rb->memory) {
		ret = -EBUSY;
		goto out;
	}

	/* only one file may do the capture */
	if (pinect_dev->capt_file != NULL
	    && pinect_dev->capt_file != file) {
		ret = -EBUSY;
		goto out;
	}

	/* if allocated, the buffers must not be mapped */
	for (i = 0; i < pinect_dev->nframes; i++) {
		if (pinect_dev->frame[i].vma_use_count) {
			ret = -EBUSY;
			goto out;
		}
	}

	/* stop streaming */
	streaming = pinect_dev->streaming;
	if (streaming) {
		pinect_stream_off(pinect_dev);

		/* Don't restart the stream when switching from read
		 * to mmap mode */
		if (pinect_dev->memory == pinect_MEMORY_READ)
			streaming = 0;
	}

	/* free the previous allocated buffers, if any */
	if (pinect_dev->nframes != 0)
		frame_free(pinect_dev);
	if (rb->count == 0)			/* unrequest */
		goto out;
	ret = frame_alloc(pinect_dev, file, rb->memory, rb->count);
	if (ret == 0) {
		rb->count = pinect_dev->nframes;
		if (streaming)
			ret = pinect_init_transfer(pinect_dev);
	}
out:
	mutex_unlock(&pinect_dev->queue_lock);
	PDEBUG(D_STREAM, "reqbufs st:%d c:%d", ret, rb->count);
	return ret;
}

static int vidioc_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *v4l2_buf)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	struct pinect_frame *frame;

	if (v4l2_buf->index >= pinect_dev->nframes)
		return -EINVAL;

	frame = &pinect_dev->frame[v4l2_buf->index];
	memcpy(v4l2_buf, &frame->v4l2_buf, sizeof *v4l2_buf);
	return 0;
}

static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type buf_type)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	int ret;

	if (buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (mutex_lock_interruptible(&pinect_dev->queue_lock))
		return -ERESTARTSYS;

	/* check the capture file */
	if (pinect_dev->capt_file != file) {
		ret = -EBUSY;
		goto out;
	}

	if (pinect_dev->nframes == 0
	    || !(pinect_dev->frame[0].v4l2_buf.flags & V4L2_BUF_FLAG_QUEUED)) {
		ret = -EINVAL;
		goto out;
	}
	if (!pinect_dev->streaming) {
		ret = pinect_init_transfer(pinect_dev);
		if (ret < 0)
			goto out;
	}
	PDEBUG_MODE(pinect_dev, D_STREAM, "stream on OK", pinect_dev->pixfmt,
		    pinect_dev->width, pinect_dev->height);
	ret = 0;
out:
	mutex_unlock(&pinect_dev->queue_lock);
	return ret;
}

static int vidioc_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	int i, ret;

	if (buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (mutex_lock_interruptible(&pinect_dev->queue_lock))
		return -ERESTARTSYS;

	if (!pinect_dev->streaming) {
		ret = 0;
		goto out;
	}

	/* check the capture file */
	if (pinect_dev->capt_file != file) {
		ret = -EBUSY;
		goto out;
	}

	/* stop streaming */
	pinect_stream_off(pinect_dev);
	/* In case another thread is waiting in dqbuf */
	wake_up_interruptible(&pinect_dev->wq);

	/* empty the transfer queues */
	for (i = 0; i < pinect_dev->nframes; i++)
		pinect_dev->frame[i].v4l2_buf.flags &= ~BUF_ALL_FLAGS;
	atomic_set(&pinect_dev->fr_q, 0);
	atomic_set(&pinect_dev->fr_i, 0);
	pinect_dev->fr_o = 0;
	ret = 0;
out:
	mutex_unlock(&pinect_dev->queue_lock);
	return ret;
}

static int vidioc_g_jpegcomp(struct file *file, void *priv,
			struct v4l2_jpegcompression *jpegcomp)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	pinect_dev->usb_err = 0;
	return pinect_dev->sd_desc->get_jcomp(pinect_dev, jpegcomp);
}

static int vidioc_s_jpegcomp(struct file *file, void *priv,
			const struct v4l2_jpegcompression *jpegcomp)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);

	pinect_dev->usb_err = 0;
	return pinect_dev->sd_desc->set_jcomp(pinect_dev, jpegcomp);
}

static int vidioc_g_parm(struct file *filp, void *priv,
			struct v4l2_streamparm *parm)
{
	struct pinect_dev *pinect_dev = video_drvdata(filp);

	parm->parm.capture.readbuffers = pinect_dev->nbufread;

	if (pinect_dev->sd_desc->get_streamparm) {
		pinect_dev->usb_err = 0;
		pinect_dev->sd_desc->get_streamparm(pinect_dev, parm);
		return pinect_dev->usb_err;
	}
	return 0;
}

static int vidioc_s_parm(struct file *filp, void *priv,
			struct v4l2_streamparm *parm)
{
	struct pinect_dev *pinect_dev = video_drvdata(filp);
	int n;

	n = parm->parm.capture.readbuffers;
	if (n == 0 || n >= pinect_MAX_FRAMES)
		parm->parm.capture.readbuffers = pinect_dev->nbufread;
	else
		pinect_dev->nbufread = n;

	if (pinect_dev->sd_desc->set_streamparm) {
		pinect_dev->usb_err = 0;
		pinect_dev->sd_desc->set_streamparm(pinect_dev, parm);
		return pinect_dev->usb_err;
	}

	return 0;
}

static int dev_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	struct pinect_frame *frame;
	struct page *page;
	unsigned long addr, start, size;
	int i, ret;

	start = vma->vm_start;
	size = vma->vm_end - vma->vm_start;
	PDEBUG(D_STREAM, "mmap start:%08x size:%d", (int) start, (int) size);

	if (mutex_lock_interruptible(&pinect_dev->queue_lock))
		return -ERESTARTSYS;
	if (pinect_dev->capt_file != file) {
		ret = -EINVAL;
		goto out;
	}

	frame = NULL;
	for (i = 0; i < pinect_dev->nframes; ++i) {
		if (pinect_dev->frame[i].v4l2_buf.memory != V4L2_MEMORY_MMAP) {
			PDEBUG(D_STREAM, "mmap bad memory type");
			break;
		}
		if ((pinect_dev->frame[i].v4l2_buf.m.offset >> PAGE_SHIFT)
						== vma->vm_pgoff) {
			frame = &pinect_dev->frame[i];
			break;
		}
	}
	if (frame == NULL) {
		PDEBUG(D_STREAM, "mmap no frame buffer found");
		ret = -EINVAL;
		goto out;
	}
	if (size != frame->v4l2_buf.length) {
		PDEBUG(D_STREAM, "mmap bad size");
		ret = -EINVAL;
		goto out;
	}

	/*
	 * - VM_IO marks the area as being a mmaped region for I/O to a
	 *   device. It also prevents the region from being core dumped.
	 */
	vma->vm_flags |= VM_IO;

	addr = (unsigned long) frame->data;
	while (size > 0) {
		page = vmalloc_to_page((void *) addr);
		ret = vm_insert_page(vma, start, page);
		if (ret < 0)
			goto out;
		start += PAGE_SIZE;
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	vma->vm_ops = &pinect_vm_ops;
	vma->vm_private_data = frame;
	pinect_vm_open(vma);
	ret = 0;
out:
	mutex_unlock(&pinect_dev->queue_lock);
	return ret;
}

static int frame_ready_nolock(struct pinect_dev *pinect_dev, struct file *file,
				enum v4l2_memory memory)
{
	if (!pinect_dev->present)
		return -ENODEV;
	if (pinect_dev->capt_file != file || pinect_dev->memory != memory ||
			!pinect_dev->streaming)
		return -EINVAL;

	/* check if a frame is ready */
	return pinect_dev->fr_o != atomic_read(&pinect_dev->fr_i);
}

static int frame_ready(struct pinect_dev *pinect_dev, struct file *file,
			enum v4l2_memory memory)
{
	int ret;

	if (mutex_lock_interruptible(&pinect_dev->queue_lock))
		return -ERESTARTSYS;
	ret = frame_ready_nolock(pinect_dev, file, memory);
	mutex_unlock(&pinect_dev->queue_lock);
	return ret;
}

/*
 * dequeue a video buffer
 *
 * If nonblock_ing is false, block until a buffer is available.
 */
static int vidioc_dqbuf(struct file *file, void *priv,
			struct v4l2_buffer *v4l2_buf)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	struct pinect_frame *frame;
	int i, j, ret;

	PDEBUG(D_FRAM, "dqbuf");

	if (mutex_lock_interruptible(&pinect_dev->queue_lock))
		return -ERESTARTSYS;

	for (;;) {
		ret = frame_ready_nolock(pinect_dev, file, v4l2_buf->memory);
		if (ret < 0)
			goto out;
		if (ret > 0)
			break;

		mutex_unlock(&pinect_dev->queue_lock);

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		/* wait till a frame is ready */
		ret = wait_event_interruptible_timeout(pinect_dev->wq,
			frame_ready(pinect_dev, file, v4l2_buf->memory),
			msecs_to_jiffies(3000));
		if (ret < 0)
			return ret;
		if (ret == 0)
			return -EIO;

		if (mutex_lock_interruptible(&pinect_dev->queue_lock))
			return -ERESTARTSYS;
	}

	i = pinect_dev->fr_o;
	j = pinect_dev->fr_queue[i];
	frame = &pinect_dev->frame[j];

	pinect_dev->fr_o = (i + 1) % pinect_MAX_FRAMES;

	frame->v4l2_buf.flags &= ~V4L2_BUF_FLAG_DONE;
	memcpy(v4l2_buf, &frame->v4l2_buf, sizeof *v4l2_buf);
	PDEBUG(D_FRAM, "dqbuf %d", j);
	ret = 0;

	if (pinect_dev->memory == V4L2_MEMORY_USERPTR) {
		if (copy_to_user((__u8 __user *) frame->v4l2_buf.m.userptr,
				 frame->data,
				 frame->v4l2_buf.bytesused)) {
			PERR("dqbuf cp to user failed");
			ret = -EFAULT;
		}
	}
out:
	mutex_unlock(&pinect_dev->queue_lock);

	if (ret == 0 && pinect_dev->sd_desc->dq_callback) {
		mutex_lock(&pinect_dev->usb_lock);
		pinect_dev->usb_err = 0;
		if (pinect_dev->present)
			pinect_dev->sd_desc->dq_callback(pinect_dev);
		mutex_unlock(&pinect_dev->usb_lock);
	}

	return ret;
}

/*
 * queue a video buffer
 *
 * Attempting to queue a buffer that has already been
 * queued will return -EINVAL.
 */
static int vidioc_qbuf(struct file *file, void *priv,
			struct v4l2_buffer *v4l2_buf)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	struct pinect_frame *frame;
	int i, index, ret;

	PDEBUG(D_FRAM, "qbuf %d", v4l2_buf->index);

	if (mutex_lock_interruptible(&pinect_dev->queue_lock))
		return -ERESTARTSYS;

	index = v4l2_buf->index;
	if ((unsigned) index >= pinect_dev->nframes) {
		PDEBUG(D_FRAM,
			"qbuf idx %d >= %d", index, pinect_dev->nframes);
		ret = -EINVAL;
		goto out;
	}
	if (v4l2_buf->memory != pinect_dev->memory) {
		PDEBUG(D_FRAM, "qbuf bad memory type");
		ret = -EINVAL;
		goto out;
	}

	frame = &pinect_dev->frame[index];
	if (frame->v4l2_buf.flags & BUF_ALL_FLAGS) {
		PDEBUG(D_FRAM, "qbuf bad state");
		ret = -EINVAL;
		goto out;
	}

	frame->v4l2_buf.flags |= V4L2_BUF_FLAG_QUEUED;

	if (frame->v4l2_buf.memory == V4L2_MEMORY_USERPTR) {
		frame->v4l2_buf.m.userptr = v4l2_buf->m.userptr;
		frame->v4l2_buf.length = v4l2_buf->length;
	}

	/* put the buffer in the 'queued' queue */
	i = atomic_read(&pinect_dev->fr_q);
	pinect_dev->fr_queue[i] = index;
	atomic_set(&pinect_dev->fr_q, (i + 1) % pinect_MAX_FRAMES);

	v4l2_buf->flags |= V4L2_BUF_FLAG_QUEUED;
	v4l2_buf->flags &= ~V4L2_BUF_FLAG_DONE;
	ret = 0;
out:
	mutex_unlock(&pinect_dev->queue_lock);
	return ret;
}

/*
 * allocate the resources for read()
 */
static int read_alloc(struct pinect_dev *pinect_dev,
			struct file *file)
{
	struct v4l2_buffer v4l2_buf;
	int i, ret;

	PDEBUG(D_STREAM, "read alloc");

	if (mutex_lock_interruptible(&pinect_dev->usb_lock))
		return -ERESTARTSYS;

	if (pinect_dev->nframes == 0) {
		struct v4l2_requestbuffers rb;

		memset(&rb, 0, sizeof rb);
		rb.count = pinect_dev->nbufread;
		rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		rb.memory = pinect_MEMORY_READ;
		ret = vidioc_reqbufs(file, pinect_dev, &rb);
		if (ret != 0) {
			PDEBUG(D_STREAM, "read reqbuf err %d", ret);
			goto out;
		}
		memset(&v4l2_buf, 0, sizeof v4l2_buf);
		v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v4l2_buf.memory = pinect_MEMORY_READ;
		for (i = 0; i < pinect_dev->nbufread; i++) {
			v4l2_buf.index = i;
			ret = vidioc_qbuf(file, pinect_dev, &v4l2_buf);
			if (ret != 0) {
				PDEBUG(D_STREAM, "read qbuf err: %d", ret);
				goto out;
			}
		}
	}

	/* start streaming */
	ret = vidioc_streamon(file, pinect_dev, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (ret != 0)
		PDEBUG(D_STREAM, "read streamon err %d", ret);
out:
	mutex_unlock(&pinect_dev->usb_lock);
	return ret;
}

static unsigned int dev_poll(struct file *file, poll_table *wait)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	unsigned long req_events = poll_requested_events(wait);
	int ret = 0;

	PDEBUG(D_FRAM, "poll");

	if (req_events & POLLPRI)
		ret |= v4l2_ctrl_poll(file, wait);

	if (req_events & (POLLIN | POLLRDNORM)) {
		/* if reqbufs is not done, the user would use read() */
		if (pinect_dev->memory == pinect_MEMORY_NO) {
			if (read_alloc(pinect_dev, file) != 0) {
				ret |= POLLERR;
				goto out;
			}
		}

		poll_wait(file, &pinect_dev->wq, wait);

		/* check if an image has been received */
		if (mutex_lock_interruptible(&pinect_dev->queue_lock) != 0) {
			ret |= POLLERR;
			goto out;
		}
		if (pinect_dev->fr_o != atomic_read(&pinect_dev->fr_i))
			ret |= POLLIN | POLLRDNORM;
		mutex_unlock(&pinect_dev->queue_lock);
	}

out:
	if (!pinect_dev->present)
		ret |= POLLHUP;

	return ret;
}

static ssize_t dev_read(struct file *file, char __user *data,
		    size_t count, loff_t *ppos)
{
	struct pinect_dev *pinect_dev = video_drvdata(file);
	struct pinect_frame *frame;
	struct v4l2_buffer v4l2_buf;
	struct timeval timestamp;
	int n, ret, ret2;

	PDEBUG(D_FRAM, "read (%zd)", count);
	if (pinect_dev->memory == pinect_MEMORY_NO) { /* first time ? */
		ret = read_alloc(pinect_dev, file);
		if (ret != 0)
			return ret;
	}

	/* get a frame */
	timestamp = ktime_to_timeval(ktime_get());
	timestamp.tv_sec--;
	n = 2;
	for (;;) {
		memset(&v4l2_buf, 0, sizeof v4l2_buf);
		v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v4l2_buf.memory = pinect_MEMORY_READ;
		ret = vidioc_dqbuf(file, pinect_dev, &v4l2_buf);
		if (ret != 0) {
			PDEBUG(D_STREAM, "read dqbuf err %d", ret);
			return ret;
		}

		/* if the process slept for more than 1 second,
		 * get a newer frame */
		frame = &pinect_dev->frame[v4l2_buf.index];
		if (--n < 0)
			break;			/* avoid infinite loop */
		if (frame->v4l2_buf.timestamp.tv_sec >= timestamp.tv_sec)
			break;
		ret = vidioc_qbuf(file, pinect_dev, &v4l2_buf);
		if (ret != 0) {
			PDEBUG(D_STREAM, "read qbuf err %d", ret);
			return ret;
		}
	}

	/* copy the frame */
	if (count > frame->v4l2_buf.bytesused)
		count = frame->v4l2_buf.bytesused;
	ret = copy_to_user(data, frame->data, count);
	if (ret != 0) {
		PERR("read cp to user lack %d / %zd", ret, count);
		ret = -EFAULT;
		goto out;
	}
	ret = count;
out:
	/* in each case, requeue the buffer */
	ret2 = vidioc_qbuf(file, pinect_dev, &v4l2_buf);
	if (ret2 != 0)
		return ret2;
	return ret;
}

static struct v4l2_file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = dev_open,
	.release = dev_close,
	.read = dev_read,
	.mmap = dev_mmap,
	.unlocked_ioctl = video_ioctl2,
	.poll	= dev_poll,
};

static const struct v4l2_ioctl_ops dev_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,
	.vidioc_dqbuf		= vidioc_dqbuf,
	.vidioc_qbuf		= vidioc_qbuf,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vidioc_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vidioc_s_fmt_vid_cap,
	.vidioc_streamon	= vidioc_streamon,
	.vidioc_enum_input	= vidioc_enum_input,
	.vidioc_g_input		= vidioc_g_input,
	.vidioc_s_input		= vidioc_s_input,
	.vidioc_reqbufs		= vidioc_reqbufs,
	.vidioc_querybuf	= vidioc_querybuf,
	.vidioc_streamoff	= vidioc_streamoff,
	.vidioc_g_jpegcomp	= vidioc_g_jpegcomp,
	.vidioc_s_jpegcomp	= vidioc_s_jpegcomp,
	.vidioc_g_parm		= vidioc_g_parm,
	.vidioc_s_parm		= vidioc_s_parm,
	.vidioc_enum_framesizes = vidioc_enum_framesizes,
	.vidioc_enum_frameintervals = vidioc_enum_frameintervals,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_chip_info	= vidioc_g_chip_info,
	.vidioc_g_register	= vidioc_g_register,
	.vidioc_s_register	= vidioc_s_register,
#endif
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct video_device pinect_template = {
	.name = "gspca main driver",
	.fops = &dev_fops,
	.ioctl_ops = &dev_ioctl_ops,
	.release = video_device_release_empty, /* We use v4l2_dev.release */
};

/*
 * probe and create a new gspca device
 *
 * This function must be called by the sub-driver when it is
 * called for probing a new device.
 */
int pinect_dev_probe2(struct usb_interface *intf,
		const struct usb_device_id *id,
		const struct sd_desc *sd_desc,
		int dev_size,
		struct module *module, int alt)
{
	struct pinect_dev *pinect_dev;
	struct usb_device *dev = interface_to_usbdev(intf);
	int ret;

	pr_info("%s-" pinect_VERSION " probing %04x:%04x\n",
		sd_desc->name, id->idVendor, id->idProduct);

	/* create the device */
	if (dev_size < sizeof *pinect_dev)
		dev_size = sizeof *pinect_dev;
	pinect_dev = kzalloc(dev_size, GFP_KERNEL);
	if (!pinect_dev) {
		pr_err("couldn't kzalloc gspca struct\n");
		return -ENOMEM;
	}
	pinect_dev->usb_buf = kmalloc(USB_BUF_SZ, GFP_KERNEL);
	if (!pinect_dev->usb_buf) {
		pr_err("out of memory\n");
		ret = -ENOMEM;
		goto out;
	}
	pinect_dev->dev = dev;
	pinect_dev->iface = intf->cur_altsetting->desc.bInterfaceNumber;
	if(alt) usb_set_interface(pinect_dev->dev, pinect_dev->iface, 1);
	/* check if any audio device */
	if (dev->actconfig->desc.bNumInterfaces != 1) {
		int i;
		struct usb_interface *intf2;

		for (i = 0; i < dev->actconfig->desc.bNumInterfaces; i++) {
			intf2 = dev->actconfig->interface[i];
			if (intf2 != NULL
			 && intf2->altsetting != NULL
			 && intf2->altsetting->desc.bInterfaceClass ==
					 USB_CLASS_AUDIO) {
				pinect_dev->audio = 1;
				break;
			}
		}
	}

	pinect_dev->v4l2_dev.release = pinect_release;
	ret = v4l2_device_register(&intf->dev, &pinect_dev->v4l2_dev);
	if (ret)
		goto out;
	pinect_dev->sd_desc = sd_desc;
	pinect_dev->nbufread = 2;
	pinect_dev->empty_packet = -1;	/* don't check the empty packets */
	pinect_dev->vdev = pinect_template;
	pinect_dev->vdev.v4l2_dev = &pinect_dev->v4l2_dev;
	video_set_drvdata(&pinect_dev->vdev, pinect_dev);
	set_bit(V4L2_FL_USE_FH_PRIO, &pinect_dev->vdev.flags);
	pinect_dev->module = module;
	pinect_dev->present = 1;

	mutex_init(&pinect_dev->usb_lock);
	pinect_dev->vdev.lock = &pinect_dev->usb_lock;
	mutex_init(&pinect_dev->queue_lock);
	init_waitqueue_head(&pinect_dev->wq);

	/* configure the subdriver and initialize the USB device */
	ret = sd_desc->config(pinect_dev, id);
	if (ret < 0)
		goto out;
	ret = sd_desc->init(pinect_dev);
	if (ret < 0)
		goto out;
	if (sd_desc->init_controls)
		ret = sd_desc->init_controls(pinect_dev);
	if (ret < 0)
		goto out;
	pinect_set_default_mode(pinect_dev);

	ret = pinect_input_connect(pinect_dev);
	if (ret)
		goto out;

	/*
	 * Don't take usb_lock for these ioctls. This improves latency if
	 * usb_lock is taken for a long time, e.g. when changing a control
	 * value, and a new frame is ready to be dequeued.
	 */
	v4l2_disable_ioctl_locking(&pinect_dev->vdev, VIDIOC_DQBUF);
	v4l2_disable_ioctl_locking(&pinect_dev->vdev, VIDIOC_QBUF);
	v4l2_disable_ioctl_locking(&pinect_dev->vdev, VIDIOC_QUERYBUF);
#ifdef CONFIG_VIDEO_ADV_DEBUG
	if (!pinect_dev->sd_desc->get_register)
		v4l2_disable_ioctl(&pinect_dev->vdev, VIDIOC_DBG_G_REGISTER);
	if (!pinect_dev->sd_desc->set_register)
		v4l2_disable_ioctl(&pinect_dev->vdev, VIDIOC_DBG_S_REGISTER);
#endif
	if (!pinect_dev->sd_desc->get_jcomp)
		v4l2_disable_ioctl(&pinect_dev->vdev, VIDIOC_G_JPEGCOMP);
	if (!pinect_dev->sd_desc->set_jcomp)
		v4l2_disable_ioctl(&pinect_dev->vdev, VIDIOC_S_JPEGCOMP);

	/* init video stuff */
	ret = video_register_device(&pinect_dev->vdev,
				  VFL_TYPE_GRABBER,
				  -1);
	if (ret < 0) {
		pr_err("video_register_device err %d\n", ret);
		goto out;
	}

	usb_set_intfdata(intf, pinect_dev);
	PDEBUG(D_PROBE, "%s created", video_device_node_name(&pinect_dev->vdev));

	pinect_input_create_urb(pinect_dev);

	return 0;
out:
#if IS_ENABLED(CONFIG_INPUT)
	if (pinect_dev->input_dev)
		input_unregister_device(pinect_dev->input_dev);
#endif
	v4l2_ctrl_handler_free(pinect_dev->vdev.ctrl_handler);
	kfree(pinect_dev->usb_buf);
	kfree(pinect_dev);
	return ret;
}

/* same function as the previous one, but check the interface */
int pinect_dev_probe(struct usb_interface *intf,
		const struct usb_device_id *id,
		const struct sd_desc *sd_desc,
		int dev_size,
		struct module *module, int alt)
{
	struct usb_device *dev = interface_to_usbdev(intf);

	/* we don't handle multi-config cameras */
	if (dev->descriptor.bNumConfigurations != 1) {
		pr_err("%04x:%04x too many config\n",
		       id->idVendor, id->idProduct);
		return -ENODEV;
	}

	/* the USB video interface must be the first one */
	if (dev->actconfig->desc.bNumInterfaces != 1
	 && intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;

	return pinect_dev_probe2(intf, id, sd_desc, dev_size, module, alt);
}


/*
 * USB disconnection
 *
 * This function must be called by the sub-driver
 * when the device disconnects, after the specific resources are freed.
 */
void pinect_disconnect(struct usb_interface *intf)
{
	struct pinect_dev *pinect_dev = usb_get_intfdata(intf);
#if IS_ENABLED(CONFIG_INPUT)
	struct input_dev *input_dev;
#endif

	PDEBUG(D_PROBE, "%s disconnect",
		video_device_node_name(&pinect_dev->vdev));

	mutex_lock(&pinect_dev->usb_lock);

	pinect_dev->present = 0;
	destroy_urbs(pinect_dev);

#if IS_ENABLED(CONFIG_INPUT)
	pinect_input_destroy_urb(pinect_dev);
	input_dev = pinect_dev->input_dev;
	if (input_dev) {
		pinect_dev->input_dev = NULL;
		input_unregister_device(input_dev);
	}
#endif
	/* Free subdriver's streaming resources / stop sd workqueue(s) */
	if (pinect_dev->sd_desc->stop0 && pinect_dev->streaming)
		pinect_dev->sd_desc->stop0(pinect_dev);
	pinect_dev->streaming = 0;
	pinect_dev->dev = NULL;
	wake_up_interruptible(&pinect_dev->wq);

	v4l2_device_disconnect(&pinect_dev->v4l2_dev);
	video_unregister_device(&pinect_dev->vdev);

	mutex_unlock(&pinect_dev->usb_lock);

	/* (this will call pinect_release() immediately or on last close) */
	v4l2_device_put(&pinect_dev->v4l2_dev);
}


#ifdef CONFIG_PM
int pinect_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct pinect_dev *pinect_dev = usb_get_intfdata(intf);

	pinect_input_destroy_urb(pinect_dev);

	if (!pinect_dev->streaming)
		return 0;

	mutex_lock(&pinect_dev->usb_lock);
	pinect_dev->frozen = 1;		/* avoid urb error messages */
	pinect_dev->usb_err = 0;
	if (pinect_dev->sd_desc->stopN)
		pinect_dev->sd_desc->stopN(pinect_dev);
	destroy_urbs(pinect_dev);
	pinect_set_alt0(pinect_dev);
	if (pinect_dev->sd_desc->stop0)
		pinect_dev->sd_desc->stop0(pinect_dev);
	mutex_unlock(&pinect_dev->usb_lock);

	return 0;
}


int pinect_resume(struct usb_interface *intf)
{
	struct pinect_dev *pinect_dev = usb_get_intfdata(intf);
	int streaming, ret = 0;

	mutex_lock(&pinect_dev->usb_lock);
	pinect_dev->frozen = 0;
	pinect_dev->usb_err = 0;
	pinect_dev->sd_desc->init(pinect_dev);
	/*
	 * Most subdrivers send all ctrl values on sd_start and thus
	 * only write to the device registers on s_ctrl when streaming ->
	 * Clear streaming to avoid setting all ctrls twice.
	 */
	streaming = pinect_dev->streaming;
	pinect_dev->streaming = 0;
	if (streaming)
		ret = pinect_init_transfer(pinect_dev);
	else
		pinect_input_create_urb(pinect_dev);
	mutex_unlock(&pinect_dev->usb_lock);

	return ret;
}

#endif

