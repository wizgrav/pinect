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


pinect_api pinect_apis[3] = {
#if defined linux
    {PINECT_TYPE_V4L,"v4l", v4l_pinect_init,v4l_pinect_free,v4l_pinect_grab},
#endif
#if defined USE_FREENECT
    {PINECT_TYPE_FNT,"fnt",fnt_pinect_init,fnt_pinect_free,fnt_pinect_grab},
#endif
    {0,NULL,NULL,NULL,NULL}
};

pinect_dev *pinect_init(char *f){
	pinect_api *api = pinect_apis;
	pinect_dev *dev=NULL;
	int l = f ? strlen(f):0;
	for(;api->newFn;api++){
		if(f && ((l > 3 && (f[3]==':')) || l==3)){
			if(strncmp(f,(const char *)api->prefix,3))
				continue;
			else
				f = l > 3:?f+4:NULL;
		}
		if((dev=api->newFn(f))){ 
			dev->api = api;
			break;
		}
	}
	return dev;
}

void pinect_free(pinect_dev *dev){
	dev->api->freeFn(dev);
}


unsigned short *pinect_grab(pinect_dev *dev, int t){
	return dev->api->grabFn(dev, t);
}

int pinect_attr(pinect_dev *dev, int attr){
	int ret;
	if(!dev) return -1; 
	switch(attr){
		case PINECT_ATTR_ERROR: ret=dev->error; break;
		case PINECT_ATTR_XRES: ret=dev->xres; break;
		case PINECT_ATTR_YRES: ret=dev->yres; break;
		case PINECT_ATTR_TYPE: ret=dev->api->id; break;
		case PINECT_ATTR_DELTA: ret=dev->delta; break;
		default: ret=-2; break;
	}
	return ret;
}

