import window
import pinect

#dev is the handle we'll use with the pinect functions. None on error
dev=pinect.new()

if not dev:
        print("no device found")
        quit()

#init returns a cffi array of chars representing the window pixels
pixels=window.init("Click for depth measurement",320,240)
if not pixels:
        print("can't create window")
        quit()
c=0
while True:
        frame=pinect.capture(dev)
        c = c+1;
        for i in xrange(0,320*240):
                
                # We'll convert depth to a more pleasant form
                # A lookup table is an obvious optimisation
                d=frame[i]
                if d > 8191:
                        pixels[i]=(255 << 24)
                else:
                        v=0
                        if d < 2048: v = v | ((255-d/8)<<16)
                        if d < 4096: v = v | ((255-d/16) << 8)
                        if d > 2048: v = v | ((d-2048)/24)
                        v = v | (255 << 24)
                        pixels[i] = v
        i = window.inspect()
        if i:
                t = "x: "+str(i[0])+", y: "+str(i[1])+", depth: "+str(frame[320*i[1]+i[0]])+"mm";
        else:
                t= "Click for depth #"+ str(c)
                     
        if window.update(t) == -1: break;
                
window.destroy() 
