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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdint.h>

#if defined _WIN32
#include <windows.h>
typedef CRITICAL_SECTION pthread_mutex_t;
#define PTHREAD_MUTEX_INITIALIZER {(void *)-1,-1,0,0,0,0}
#define pthread_mutex_init(m,a) InitializeCriticalSection(m)
#define pthread_mutex_destroy(m) DeleteCriticalSection(m)
#define pthread_mutex_lock(m) EnterCriticalSection(m)
#define pthread_mutex_unlock(m) LeaveCriticalSection(m)
typedef CONDITION_VARIABLE pthread_cond_t;
#define PTHREAD_COND_INITIALIZER {0}
#define pthread_cond_init(c,a) InitializeConditionVariable(c)
#define pthread_cond_destroy(c) (void)c
#define pthread_cond_wait(c,m) SleepConditionVariableCS(c,m,INFINITE)
#define pthread_cond_broadcast(c) WakeAllConditionVariable(c)
#define thread_t HANDLE
#define thread_start(dev) dev->thread = CreateThread(NULL,65536,(LPTHREAD_START_ROUTINE) & thread_func,dev,0,NULL)
#define thread_stop(dev) dev->active=0; WaitForSingleObject(dev->thread, INFINITE);

#else

#include <pthread.h>

#define thread_t pthread_t
#define thread_start(dev)\
	size_t stacksize=65536;\
	pthread_attr_t attr;\
	pthread_attr_init(&attr);\
	pthread_attr_setstacksize (&attr, stacksize);\
	pthread_create(&dev->thread, &attr, &thread_func, dev);\
	pthread_attr_destroy(&attr)
#define thread_stop(thread) pthread_join(thread,NULL);

#endif

#include "../deps/libfreenect/include/libfreenect.h"
#include "../deps/libfreenect/include/libfreenect_registration.h"
#include "../deps/libuvc/include/libuvc/libuvc.h"


#include "pinect.h"

pinect_dev *fnt_pinect_init(char *f);
void fnt_pinect_free(pinect_dev *dev);
unsigned short *fnt_pinect_grab(pinect_dev *dev, int t);

pinect_dev *uvc_pinect_init(char *f);
void uvc_pinect_free(pinect_dev *dev);
unsigned short *uvc_pinect_grab(pinect_dev *dev, int t);

#if defined linux
pinect_dev *v4l_pinect_init(char *f);
void v4l_pinect_free(pinect_dev *dev);
unsigned short *v4l_pinect_grab(pinect_dev *dev, int t);
#endif

#define MIN(a,b) (a < b? a:b)


typedef struct pinect_dev pinect_dev;

typedef pinect_dev *(*pinect_new_fn)(char *f);
typedef unsigned short *(*pinect_grab_fn)(pinect_dev *dev, int t);
typedef void (*pinect_free_fn)(pinect_dev *dev);

typedef struct pinect_api{
  int id;
  char *prefix;
  pinect_new_fn newFn;
  pinect_free_fn freeFn;
  pinect_grab_fn grabFn;
} pinect_api;

extern pinect_api pinect_apis[3];

typedef struct frame_t {
	uint32_t count;
	uint64_t time;
	uint16_t *data;
} frame_t;

typedef struct pinect_dev {
  pinect_api *api;
  int xres, yres, error;
  uint64_t latest, delta;
  frame_t frames[3], *current, *working,*spare;
  uint32_t wait,active,count;
  uint16_t depthtomm[2048];
  pthread_cond_t condition;
  pthread_mutex_t mutex;
  freenect_context *ctx;
  freenect_device *dev;
  uvc_device_handle_t *udevh;
  uvc_context_t *uctx;
  uvc_device_t *udev;
  thread_t thread;
#if defined linux
  int fd;
#endif
} pinect_dev;

#define PUSHERROR(err) \
	printf("ERR:%s\n",err);\
	return -1
#define PUSHERRORNIL(err) \
	printf("ERR:%s\n",err);\
	return NULL
