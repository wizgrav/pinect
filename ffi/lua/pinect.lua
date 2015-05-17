local ffi = require("ffi")

ffi.cdef[[
    void *pinect_new(unsigned char *f);
    unsigned short * pinect_capture(void *dev, int r);
    int pinect_release(void *dev);
    int pinect_free(void *dev);
]]

local lib = ffi.load("/usr/local/lib/libpinect.so")

return {
    new = function(s) return ffi.gc(lib.pinect_new(s),lib.pinect_free) end,
    capture = lib.pinect_capture,
    release = lib.pinect_release,
    free = lib.pinect_free    
}
