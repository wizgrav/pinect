/*
Pinect, a helper lib for depth cameras
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


pinect_dev *uvc_pinect_init(char *f){
    int i;
    pinect_dev *dev=(pinect_dev *)malloc(sizeof(pinect_dev));
    memset(dev,0,sizeof(pinect_dev));
    dev->count=0;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res;
    res = uvc_init(&dev->uctx, NULL);
    if (res < 0) {
	free(dev);
	return NULL;
    }
    res = uvc_find_device(dev->uctx, &dev->udev, 0x8086, 0x0a66, NULL);
    if (res < 0) {
	free(dev);
	return NULL;
    }
    res = uvc_open(dev->udev, &dev->udevh);
    if (res < 0) {
	free(dev);
	return NULL;
    }
    res = uvc_get_stream_ctrl_format_size( dev->udevh, &ctrl, UVC_FRAME_FORMAT_INVZ, 640, 480, 30);
    if (res < 0) {
	uvc_pinect_free(dev);
	return NULL;
    }
    return dev;
}

void uvc_pinect_free(pinect_dev *dev){
  uvc_close(dev->udevh);
  uvc_unref_device(dev->udev);
  uvc_exit(dev->uctx);  
  free(dev);
}


unsigned short *uvc_pinect_grab(pinect_dev *dev, int t){
    return NULL;
}
