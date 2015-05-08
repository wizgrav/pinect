#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             

#include <fcntl.h>              
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>


#include <linux/videodev2.h>
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
	
#define FRAMEPIXELS 320*240
#define FRAMESIZE FRAMEPIXELS*2
#define PUSHERROR(err) \
	printf("ERR:%s\n",err);\
	return -1
	
typedef struct {
	int fd,index;
	void *buffers[3];
	unsigned short *current;
} pinect_dev;
