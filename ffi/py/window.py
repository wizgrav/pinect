from cffi import FFI
ffi = FFI()
ffi.cdef("""
	unsigned int *window_init(unsigned char *title, int w, int h);
	int window_update(unsigned char *title);
	int *window_inspect(void);
	int window_rect(int x, int y, int w, int h, int c);
	void window_destroy(void);
""")

lib=ffi.verify("""
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
""", libraries=["SDL2"])
def init(title,w,h): return lib.window_init(title,w,h);
def update(title): return lib.window_update(ffi.new("unsigned char[]",title));
def rect(x,y,w,h,c=255): return lib.window_rect(x,y,w,h,c);
def inspect(): return lib.window_inspect();
def destroy(): return lib.window_destroy();
