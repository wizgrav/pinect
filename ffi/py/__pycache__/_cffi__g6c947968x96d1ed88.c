
#include <stdio.h>
#include <stddef.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/types.h>   /* XXX for ssize_t on some platforms */

#ifdef _WIN32
#  include <Windows.h>
#  define snprintf _snprintf
typedef __int8 int8_t;
typedef __int16 int16_t;
typedef __int32 int32_t;
typedef __int64 int64_t;
typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
typedef SSIZE_T ssize_t;
typedef unsigned char _Bool;
#else
#  include <stdint.h>
#endif


	#include <SDL2/SDL.h>
	
	SDL_Window *window;
	SDL_Surface *surface;
	
	int xy[2];
	
	unsigned int *window_init(char *title, int w, int h){
		        SDL_Init(SDL_INIT_VIDEO);
		window = SDL_CreateWindow(title,
                                SDL_WINDOWPOS_CENTERED,
                                SDL_WINDOWPOS_CENTERED,
                                w,
                                h,
                                SDL_WINDOW_SHOWN);
                if(!window) return NULL;
                surface = SDL_GetWindowSurface(window);
		if(!surface) return NULL;
		return surface->pixels;
	}
	
	int window_update(unsigned char *title){
		SDL_Event event;
		while( SDL_PollEvent(&event) != 0 ){
			if(event.type == SDL_QUIT){
				return -1;
			}
		}
		if(title){
		        SDL_SetWindowTitle(window,title);
		}
		SDL_UpdateWindowSurface(window);
		return 0;
	}
	
	int window_rect(int x, int y,int w, int h,int c){
		SDL_Rect rect;
		rect.x = x;
		rect.y = y;
		rect.w = w;
		rect.h = h;
		SDL_FillRect(surface,&rect,c);
		return 0;
	}
	int *window_inspect(void){
	        int b;
	        b=SDL_GetMouseState(&xy[0],&xy[1]);
	        if(b) return xy; else return NULL;
	}
	void window_destroy(void){
		SDL_DestroyWindow(window);
		SDL_Quit();		
	}

void _cffi_f_window_destroy(void)
{
  window_destroy();
}

unsigned int * _cffi_f_window_init(unsigned char * x0, int x1, int x2)
{
  return window_init(x0, x1, x2);
}

int * _cffi_f_window_inspect(void)
{
  return window_inspect();
}

int _cffi_f_window_rect(int x0, int x1, int x2, int x3, int x4)
{
  return window_rect(x0, x1, x2, x3, x4);
}

int _cffi_f_window_update(unsigned char * x0)
{
  return window_update(x0);
}

