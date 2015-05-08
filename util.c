#include "util.h"

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);

        return r;
}



static pinect_dev *pinect_new(unsigned char *f){
	size_t l;
	int fd,i;
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	enum v4l2_buf_type type;
	struct v4l2_requestbuffers req;
	void *buffers[3];
	pinect_dev *dev;
	
	fd = open(f, O_RDWR  | O_NONBLOCK, 0);

	if (-1 == fd) {
		PUSHERROR("notfound");	
	}
       
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		close(fd);
		if (EINVAL == errno) {
			PUSHERROR("dev");
		} else {
			PUSHERROR("caps");	
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		close(fd);
		PUSHERROR("capture");
	}
	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		close(fd);
        PUSHERROR("stream");
    }
    memset((void *)&fmt,0,sizeof(fmt));
	

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = 320;
	fmt.fmt.pix.height      = 240;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
	fmt.fmt.pix.field       = V4L2_FIELD_NONE;

	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
		PUSHERROR("format");
	}
	memset((void *)&req,0,sizeof(req));
	
	req.count  = 3;
	req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
			if (EINVAL == errno) {
				PUSHERROR("userp");
			} else {
				PUSHERROR("req");
			}
	}

	dev = (pinect_dev *)malloc(sizeof(pinect_dev));
	memset(dev,0,sizeof(pinect_dev));
	dev->fd = fd;
	for (i=0; i < 3; i++) {
		dev->buffers[i] = malloc(FRAMESIZE);
		struct v4l2_buffer buf;

		memset((void *)&buf,0,sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;
		buf.index = i;
		buf.m.userptr = (unsigned long)dev->buffers[i];
		buf.length = FRAMESIZE;

		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
				PUSHERROR("dev");
		}
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
			PUSHERROR("stream");
	}
	dev->current=NULL;
    return dev;
}

static int pinect_free(pinect_dev *dev){
	int i;
	enum v4l2_buf_type type;
	
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(dev->fd, VIDIOC_STREAMOFF, &type);
	for(i=0;i!=3;i++){
		free(dev->buffers[i]);
	}
	close(dev->fd);
	return 0;
}


static int pinect_capture(pinect_dev *dev, int r){
	struct v4l2_buffer buf;
	fd_set fds;
	struct timeval tv;
	int r,rr;
	void *ptr;
	FD_ZERO(&fds);
	FD_SET(dev->fd, &fds);
	if(r >= 0){
		tv.tv_sec = r;
		tv.tv_usec = 0;
	}
	rr = select(dev->fd + 1, &fds, NULL, NULL, r > 0 ? &tv:NULL);

	if (-1 == rr && EINTR == errno) {
		PUSHERROR("interrupted");
	}

	if (0 == rr && r > 0) {
		PUSHERROR("timeout");
	}
	memset((void *)&buf,0,sizeof(buf));

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(dev->fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
				case EAGAIN:
						PUSHERROR("none");
				case EIO: 
				default:
						PUSHERROR("dev");
			}
	}
	dev->index = buf.index;
	dev->current = (unsigned short *) buf.m.userptr;
	return 0;
}

int pinect_release(pinect_dev *dev){
	struct v4l2_buffer buf;
	if(!dev->current) return 0;
	memset((void *)&buf,0,sizeof(buf));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.m.userptr = (unsigned long)dev->current;
	buf.length = FRAMESIZE;
	buf.index = dev->index;
	if (-1 == xioctl(dev->fd, VIDIOC_QBUF, &buf)){
		PUSHERROR("dev");
	}
	dev->current = NULL;
	return 0;
}
