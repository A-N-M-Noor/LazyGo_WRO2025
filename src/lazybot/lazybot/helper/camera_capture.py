import cv2, time
from threading import Thread

class Camera:
    def __init__(self, src:int, width:int=640, height:int=480):
        self.width = width
        self.height = height
        
        self.cap=cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        self.frame = None
        
        self.running = False
    
    def start(self):
        if(not self.running):
            print("Starting Camera Capture")
            self.running = True
            self.thread=Thread(target=self.update,args=())
            self.thread.daemon=True
            self.thread.start()
        
    def stop(self):
        if(self.running):
            self.cap.release()
            self.running = False
            print("Stopping Camera Capture")
    
    def update(self):
        while self.running:
            _,self.frame = self.cap.read()
            
    def getFrame(self, blr:int = 0):
        if(not self.running or self.frame is None):
            return None
            
        blr = int(blr)            
        if(blr > 0):
            if(blr%2 == 0):
                blr = blr+1
            _frm = self.frame.copy()
            _frm = cv2.GaussianBlur(_frm, (blr,blr), cv2.BORDER_DEFAULT)
            return _frm
        
        if(self.frame is not None):
            return self.frame.copy()
        return None

if(__name__ == "__main__"):
    cam = Camera(0)
    cam.start()
    
    while(True):
        frame = cam.getFrame()
        if(frame is not None):
            cv2.imshow("cap", frame)
        
        if cv2.waitKey(1)==ord('q'):
            cam.stop()
            cv2.destroyAllWindows()
            exit(1)
            break