import cv2, time
from threading import Thread
import subprocess

class Camera:
    def __init__(self, src, width:int=640, height:int=480):
        self.width = width
        self.height = height
        
        self.cap=cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        self.cap.read()
        
        time.sleep(0.2)
        self.set_camera_controls(device = src, exposure=180, gain=120, wb_temp=4500, saturation=50)
        
        self.frame = None
        
        self.running = False

    def set_camera_controls(self, device="/dev/video2", exposure=166, gain=123, wb_temp=4000, saturation=32):
        """
        Lock exposure and white balance on Logitech C270.
        """
        print(f"Setting exposure to {exposure}")
        
        commands = [
            ["--set-ctrl=auto_exposure=1"],                     # 1 = manual mode (set first)
            ["--set-ctrl=exposure_dynamic_framerate=0"],        # disable dynamic framerate
            [f"--set-ctrl=exposure_time_absolute={exposure}"],  # exposure value (set after manual mode)
            [f"--set-ctrl=gain={gain}"],                             # Lock gain to prevent auto-adjustment
            # [f"--set-ctrl=saturation={saturation}"],            # set saturation
            ["--set-ctrl=white_balance_automatic=0"],           # disable auto WB
            [f"--set-ctrl=white_balance_temperature={wb_temp}"] # set WB manually
        ]

        for ctrl in commands:
            try:
                result = subprocess.run(
                    ["v4l2-ctl", "-d", device] + ctrl,
                    check=True,
                    capture_output=True,
                    text=True
                )
                print(f"✅ {' '.join(ctrl)}")
                if result.stdout.strip():
                    print(result.stdout.strip())
                if result.stderr.strip():
                    print(f"⚠️ {result.stderr.strip()}")
                time.sleep(0.1)
            except subprocess.CalledProcessError as e:
                print(f"❌ Failed: {e.stderr.strip() if e.stderr else e}")

        # Wait and verify the settings stuck
        time.sleep(0.5)
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", device, "--get-ctrl=auto_exposure,exposure_time_absolute,gain"],
                capture_output=True,
                text=True
            )
            print(f"📋 Final settings: \n{result.stdout.strip()}")
        except Exception as e:
            print(f"❌ Verification failed: {e}")

        print("✅ C270 camera controls applied!")
        
    
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
            
    def getFrame(self, blr:int = 0, rotate_180:bool = False):
        if(not self.running or self.frame is None):
            return None
            
        _frm = self.frame.copy()
        
        # Rotate by 180 degrees
        if rotate_180:
            _frm = cv2.rotate(_frm, cv2.ROTATE_180)
        
        blr = int(blr)            
        if(blr > 0):
            if(blr%2 == 0):
                blr = blr+1
            _frm = cv2.GaussianBlur(_frm, (blr,blr), cv2.BORDER_DEFAULT)
            
        return _frm

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