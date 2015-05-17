from cffi import FFI
ffi = FFI()

ffi.cdef("""
    void *pinect_init(unsigned char *f);
    unsigned short *pinect_grab(void *dev, int t);
    int pinect_attr(char *key);
    void pinect_free(void *dev);
""")


lib = ffi.dlopen("/usr/local/lib/libpinect.so") 

def init(s="/dev/video0"):
    return ffi.gc(lib.pinect_init(s),lib.pinect_free)

def grab(d,r=0):
    return lib.pinect_grab(d,r)
    
def attr(d,s):
    return lib.pinect_attr(d,s)
