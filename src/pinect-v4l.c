/*
Pinect, a helper lib for interacting with depth cameras
Copyright (c) 2015, Yannis Gravezas,All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
     
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "pinect-internal.h"

#include <fcntl.h>              
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

static int xioctl(int fd, int request, void *arg)
{
    int r;
        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);
        return r;
}

pinect_dev *v4l_pinect_init(char *f){
	int fd,i;
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	enum v4l2_buf_type type;
	struct v4l2_requestbuffers req;
	pinect_dev *dev;
	
	fd = open((char *)f, O_RDWR  | O_NONBLOCK, 0);

	if (-1 == fd) {
		PUSHERRORNIL("notfound");	
	}
       
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		close(fd);
		if (EINVAL == errno) {
			PUSHERRORNIL("dev");
		} else {
			PUSHERRORNIL("caps");	
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		close(fd);
		PUSHERRORNIL("capture");
	}
	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		close(fd);
        PUSHERRORNIL("stream");
    }
    memset((void *)&fmt,0,sizeof(fmt));
	

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = 320;
	fmt.fmt.pix.height      = 240;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
	fmt.fmt.pix.field       = V4L2_FIELD_NONE;

	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
		PUSHERRORNIL("format");
	}
	memset((void *)&req,0,sizeof(req));
	
	req.count  = 3;
	req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
			if (EINVAL == errno) {
				PUSHERRORNIL("userp");
			} else {
				PUSHERRORNIL("req");
			}
	}

	dev = (pinect_dev *)malloc(sizeof(pinect_dev));
	memset(dev,0,sizeof(pinect_dev));
	dev->fd = fd;
	for (i=0; i < 3; i++) {
		dev->frames[i].data = malloc(320*240*2);
		struct v4l2_buffer buf;

		memset((void *)&buf,0,sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;
		buf.index = i;
		buf.m.userptr = (unsigned long)dev->frames[i].data;
		buf.length = 320*240*2;

		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
				do {
					free(dev->frames[i].data);
				} while(i--);
				free(dev);
				PUSHERRORNIL("dev");
		}
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
			do { free(dev->frames[i].data);} while(i--);
			free(dev);
			PUSHERRORNIL("stream");
	}
	dev->current=NULL;
    return dev;
}

void v4l_pinect_free(pinect_dev *dev){
	int i;
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(dev->fd, VIDIOC_STREAMOFF, &type);
	for(i=0;i!=3;i++) if(dev->frames[i].data)free(dev->frames[i].data);
	close(dev->fd);
}


static int v4l_pinect_release(pinect_dev *dev){
	struct v4l2_buffer buf;
	if(!dev->current) return 0;
	memset((void *)&buf,0,sizeof(buf));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.m.userptr = (unsigned long)dev->current;
	buf.length = 320*240*2;
	buf.index = dev->count;
	if (-1 == xioctl(dev->fd, VIDIOC_QBUF, &buf)){
		PUSHERROR("dev");
	}
	dev->current = NULL;
	return 0;
}

unsigned short *v4l_pinect_grab(pinect_dev *dev, int r){
	struct v4l2_buffer buf;
	fd_set fds;
	struct timeval tv;
	int rr;
	if(dev->current) v4l_pinect_release(dev);
	FD_ZERO(&fds);
	FD_SET(dev->fd, &fds);
	if(r >= 0){
		tv.tv_sec = r;
		tv.tv_usec = 0;
	}
	rr = select(dev->fd + 1, &fds, NULL, NULL, r > 0 ? &tv:NULL);

	if (-1 == rr && EINTR == errno) {
		PUSHERRORNIL("interrupted");
	}

	if (0 == rr && r > 0) {
		return NULL;
	}
	memset((void *)&buf,0,sizeof(buf));

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(dev->fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
				case EAGAIN:
						PUSHERRORNIL("none");
				case EIO: 
				default:
						PUSHERRORNIL("dev");
			}
	}
	dev->count = buf.index;
	dev->frames[0].data = (unsigned short *) buf.m.userptr;
	dev->current = dev->frames;
	return dev->current->data;
}
