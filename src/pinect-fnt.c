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

static void chunk_cb(void *buffer, void *pkt_data, int pkt_num, int pkt_size,void *ud)
{
	int n;
	uint8_t *raw = (uint8_t *) pkt_data;
	uint16_t *frame=(uint16_t *)buffer;
	uint16_t *mm = (uint16_t *) (((pinect_dev *)(ud))->depthtomm); 
	if(pkt_num == 73 || pkt_num == 146) return;
	if(pkt_num > 219){
		raw += (pkt_num-220) * 12;
		frame += 320 * (pkt_num-2);
	}else if(pkt_num > 146){
		raw += (pkt_num-147) * 12 + 4;
		frame += 320 * (pkt_num-2);
	}else if(pkt_num > 73){
		raw += (pkt_num-74) * 12 + 8;
		frame += 320 * (pkt_num-1);
	}else{
		raw += pkt_num * 12;
		frame += 320 * pkt_num;
	}
	for(n=40;n--;){
		frame[0] =  mm[(raw[0]<<3)  | (raw[1]>>5)];
        frame[1] = mm[((raw[2]<<9)  | (raw[3]<<1) | (raw[4]>>7) ) & 2047];
        frame[2] = mm[((raw[5]<<7)  | (raw[6]>>1) ) & 2047];
        frame[3] = mm[((raw[8]<<5)  | (raw[9]>>3) ) & 2047];
        frame[4] =  mm[(raw[11]<<3)  | (raw[12]>>5)];
        frame[5] = mm[((raw[13]<<9)  | (raw[14]<<1) | (raw[15]>>7) ) & 2047];
        frame[6] = mm[((raw[16]<<7)  | (raw[17]>>1) ) & 2047];
        frame[7] = mm[((raw[19]<<5)  | (raw[20]>>3) ) & 2047];
        frame+=8;
        raw+=22;
    }
}

static void depth_cb(freenect_device *fdev, void *v_depth, uint32_t timestamp)
{
  frame_t *frame;
  pinect_dev *dev = (pinect_dev *)freenect_get_user(fdev);
  pthread_mutex_lock(&dev->mutex);
  do{dev->count++;}while(!dev->count);
  dev->working->count = dev->count;
  frame = dev->working;
  dev->working = dev->spare;
  dev->spare=frame;
  pthread_cond_broadcast(&dev->condition);
  pthread_mutex_unlock(&dev->mutex);
  freenect_set_depth_buffer(fdev,dev->working->data);
}


static void *thread_func(void *arg){
  int i;
  pinect_dev *dev = (pinect_dev *) arg;
 
  while(dev->active){
	i=freenect_process_events(dev->ctx);
	if(i < 0) dev->active=0;
  }
  if(dev->dev){
    freenect_stop_depth(dev->dev);
    freenect_close_device(dev->dev);
    dev->dev=NULL;
  }
  
  if(dev->ctx){
    freenect_shutdown(dev->ctx);
    dev->ctx=NULL;
  }
  return 0;
}


pinect_dev *fnt_pinect_init(char *f){
    int i;
    freenect_registration reg;
    pinect_dev *dev=(pinect_dev *)malloc(sizeof(pinect_dev));
    memset(dev,0,sizeof(pinect_dev));
    dev->working = &dev->frames[0];
    dev->spare = &dev->frames[1];
    dev->current = &dev->frames[2];
    dev->current->count = 0;
    dev->count=0;
    if (freenect_init(&dev->ctx, NULL) < 0){ 
        free(dev);
        return NULL;
    }
    freenect_set_log_level(dev->ctx, FREENECT_LOG_FATAL);
    freenect_select_subdevices(dev->ctx, (freenect_device_flags)(FREENECT_DEVICE_CAMERA));
    i = freenect_open_device(dev->ctx, &dev->dev, 0);
    if(i >= 0){
        
	for (i=0; i < 3; i++) dev->frames[i].data = malloc(320*240*2);
	
	freenect_set_user(dev->dev,(void *)dev);
	freenect_set_depth_buffer(dev->dev,dev->working->data);
        freenect_set_depth_mode(dev->dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT_PACKED));
        freenect_set_video_mode(dev->dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_BAYER));
        freenect_set_depth_callback(dev->dev, depth_cb);
        freenect_set_depth_chunk_callback(dev->dev, chunk_cb);
	
        freenect_start_depth(dev->dev);
        reg=freenect_copy_registration(dev->dev);
        dev->depthtomm[0]=0;
        for(i=1; i!=2048; i++) dev->depthtomm[i] = MIN(reg.raw_to_mm_shift[i],8191);
        freenect_destroy_registration(&reg);
        dev->active=1;
	pthread_mutex_init(&dev->mutex,NULL);
	pthread_cond_init(&dev->condition,NULL);
	thread_start(dev);
    } else {
        free(dev);
        dev=NULL;
    }
    return dev;
}

void fnt_pinect_free(pinect_dev *dev){
  int i;
  dev->active=0;
  thread_stop(dev->thread);
  
  for(i=0;i!=3;i++) free(dev->frames[i].data);
  
  pthread_mutex_destroy(&dev->mutex);
  pthread_cond_destroy(&dev->condition);
  free(dev);
}


unsigned short *fnt_pinect_grab(pinect_dev *dev, int t){
    if(!dev->active){
        thread_stop(dev->thread);
	dev->error=1;
        return NULL;
    }
    frame_t *frame;
    pthread_mutex_lock(&dev->mutex);
    while(1){
	if(dev->current->count != dev->spare->count){
            frame = dev->current;
            dev->current = dev->spare;
            dev->spare = frame;
	    dev->spare->count = dev->current->count;
            break;
        }
	if(t) pthread_cond_wait(&dev->condition, &dev->mutex); else break;
    }
    pthread_mutex_unlock(&dev->mutex);

    return dev->current ? dev->current->data:NULL;
}
