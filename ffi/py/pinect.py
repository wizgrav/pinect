from cffi import FFI
ffi = FFI()

ffi.cdef("""
    void *pinect_new(unsigned char *f);
    unsigned short *pinect_capture(void *dev, int r);
    int pinect_release(void *dev);
    int pinect_free(void *dev);
""")


lib = ffi.dlopen("/usr/local/lib/libpinect.so") 

def new(s="/dev/video0"):
    return ffi.gc(lib.pinect_new(s),lib.pinect_free)

def capture(d,r=0):
    return lib.pinect_capture(d,r)
    
def release(d):
    return lib.pinect_release(d)
