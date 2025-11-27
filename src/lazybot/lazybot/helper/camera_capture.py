import cv2, time
from threading import Thread
import subprocess

class Camera:
    """
    Threaded Camera Capture Class.
    
    Why Threading?
    OpenCV's `cap.read()` is a blocking operation. If run in the main loop, 
    it limits the processing FPS to the camera's hardware FPS. 
    By running capture in a separate thread, the main loop can process the 
    latest available frame as fast as possible without waiting for the hardware.
    """
    def __init__(self, src: int | str, width:int=640, height:int=480):
        self.width = width
        self.height = height
        
        # Initialize VideoCapture
        # 'src' can be an integer (index) or a path string (e.g., /dev/video0)
        self.cap=cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # Read one frame to ensure connection is established
        self.cap.read()
        
        # Wait for camera to warm up before applying settings
        time.sleep(0.2)
        
        # Apply manual exposure/gain settings using v4l2-ctl
        # This is crucial for computer vision to prevent auto-exposure flickering
        self.set_camera_controls(device = src, exposure=185, gain=120, wb_temp=4500, saturation=None)
        
        self.frame = None
        self.running = False

    def set_camera_controls(self, device="/dev/video2", exposure=None, gain=None, wb_temp=None, saturation=None):
        """
        Configures camera hardware settings using the Linux `v4l2-ctl` utility.
        
        Target Hardware: Logitech C270 (or similar UVC cameras).
        Goal: Lock exposure and white balance to ensure consistent color detection
              regardless of ambient lighting changes.
        """
        print(f"Setting exposure to {exposure}")
        
        # Sequence of commands to disable auto-features and set manual values
        commands = []
        if exposure is not None: 
            commands.append(["--set-ctrl=auto_exposure=1"])                         # 1 = Manual Mode (Must be set first)
            commands.append(["--set-ctrl=exposure_dynamic_framerate=0"])            # Disable dynamic framerate (prevents motion blur)
            commands.append([f"--set-ctrl=exposure_time_absolute={exposure}"])      # Set absolute exposure time
        if gain is not None:
            commands.append([f"--set-ctrl=gain={gain}"])                            # Lock gain to reduce noise
        if saturation is not None:
            commands.append([f"--set-ctrl=saturation={saturation}"])                # Set saturation (Optional)
        if wb_temp is not None:
            commands.append(["--set-ctrl=white_balance_automatic=0"])               # Disable Auto White Balance
            commands.append([f"--set-ctrl=white_balance_temperature={wb_temp}"])    # Set fixed Color Temperature
        

        for ctrl in commands:
            try:
                # Execute shell command
                result = subprocess.run(
                    ["v4l2-ctl", "-d", str(device)] + ctrl,
                    check=True,
                    capture_output=True,
                    text=True
                )
                print(f"{' '.join(ctrl)}")
                if result.stdout.strip():
                    print(result.stdout.strip())
                if result.stderr.strip():
                    print(f"[!] {result.stderr.strip()}")
                time.sleep(0.1) # Small delay to allow hardware to register command
            except subprocess.CalledProcessError as e:
                print(f"[X] Failed: {e.stderr.strip() if e.stderr else e}")

        # Verify settings were applied
        time.sleep(0.5)
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", str(device), "--get-ctrl=auto_exposure,exposure_time_absolute,gain"],
                capture_output=True,
                text=True
            )
            print(f"📋 Final settings: \n{result.stdout.strip()}")
        except Exception as e:
            print(f"❌ Verification failed: {e}")

        print("✅ Camera controls applied!")
        
    
    def start(self):
        """Starts the background capture thread."""
        if(not self.running):
            print("Starting Camera Capture")
            self.running = True
            self.thread=Thread(target=self.update,args=())
            self.thread.daemon=True # Daemon threads exit when the main program exits
            self.thread.start()
        
    def stop(self):
        """Stops the background thread and releases the camera resource."""
        if(self.running):
            self.cap.release()
            self.running = False
            print("Stopping Camera Capture")
    
    def update(self):
        """
        Background Loop:
        Continuously reads frames from the buffer.
        This keeps the buffer empty so `self.frame` is always the newest reality.
        """
        while self.running:
            _,self.frame = self.cap.read()
            # Print the exposure value from the camera
            exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            gain = self.cap.get(cv2.CAP_PROP_GAIN)
            white_balance = self.cap.get(cv2.CAP_PROP_TEMPERATURE)
            saturation = self.cap.get(cv2.CAP_PROP_SATURATION)
            # print(f"Current Exposure: {exposure}, Gain: {gain}, White Balance: {white_balance}, Saturation: {saturation}")
            # print(f"Current Exposure: {exposure}")
            
    def getFrame(self, blr:int = 0, rotate_180:bool = False):
        """
        Returns the latest available frame with optional preprocessing.
        
        Args:
            blr (int): Kernel size for Gaussian Blur (must be odd).
            rotate_180 (bool): Flips the image if camera is mounted upside down.
        """
        if(not self.running or self.frame is None):
            return None
            
        _frm = self.frame.copy()
        
        # Rotate by 180 degrees
        if rotate_180:
            _frm = cv2.rotate(_frm, cv2.ROTATE_180)
        
        # Apply Gaussian Blur if requested
        blr = int(blr)            
        if(blr > 0):
            if(blr%2 == 0):
                blr = blr+1 # Kernel size must be odd
            _frm = cv2.GaussianBlur(_frm, (blr,blr), cv2.BORDER_DEFAULT)
            
        return _frm

# Simple test script to verify camera functionality
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