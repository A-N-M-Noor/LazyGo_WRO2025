import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray

import cv2, math, sys, time
import numpy as np
import customtkinter as ctk

from threading import Thread
import lazybot.helper.util as util
from lazybot.helper.camera_capture import Camera
from lazybot.helper.collapsible import CollapsiblePane


ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("green")  # Themes: "blue" (standard), "green", "dark-blue"
window = ctk.CTk()
window.title("HSV Calibration Tool")
# window.attributes('-topmost',True)

# Configuring the grid system
window.grid_columnconfigure(0, weight=1)


CAMSRC = "/dev/v4l/by-id/usb-046d_081b_61C8A860-video-index0"
RANGES = [[0,0,0],[179,255,255]]
# RANGES = [[0,0,0],[255,255,255]]

COLORSPACE = ["Hue", "Saturation", "Value"]
# COLORSPACE = ["L", "A", "B"]

color_spaces = {
    "HSV": {
        "code": cv2.COLOR_BGR2HSV,
        "ranges": [[0,0,0],[179,255,255]],
        "labels": ["Hue", "Saturation", "Value"]
    },
    "HLS": {
        "code": cv2.COLOR_BGR2HLS,
        "ranges": [[0,0,0],[179,255,255]],
        "labels": ["Hue", "Lightness", "Saturation"]
    },
    "LAB": {
        "code": cv2.COLOR_BGR2Lab,
        "ranges": [[0,0,0],[255,255,255]],
        "labels": ["L", "A", "B"]
    },
    "LUV": {
        "code": cv2.COLOR_BGR2Luv,
        "ranges": [[0,0,0],[255,255,255]],
        "labels": ["L", "U", "V"]
    },
    "YCrCb": {
        "code": cv2.COLOR_BGR2YCrCb,
        "ranges": [[0,0,0],[255,255,255]],
        "labels": ["Y", "Cr", "Cb"]
    }
}

def btn_press(cmd):
    global first
    print(f"{cmd} happened")
    
    if(cmd == "reset"):
        setSliders(RANGES, 0)
        first = True
    
    if(cmd == "save"):
        util.save_range_data(
            cSpace.get(),
            colors.get(), 
            [
                [int(H_min.get()), int(S_min.get()), int(V_min.get())], 
                [int(H_max.get()), int(S_max.get()), int(V_max.get())]
            ],
            blurSlider.get(),
            maskBlurSlider.get(),
            gammaSlider.get()
        )
        

def setLabel(label, initial, value, isInt:bool = True):
    if(isInt):
        label.configure(text=f"{initial}: {int(value)}")
    else:
        label.configure(text=f"{initial}: {value:.2f}")

def createFrame(parent, _row, stick="nsew", columns = 1, pack = True):
    frm = ctk.CTkFrame(parent, border_width=1, height = 0, border_color="gray28")
    if(pack):
        frm.grid(row=_row, column=0, padx=20, pady=(0,20), sticky=stick)
    
    for i in range(columns):
        frm.grid_columnconfigure(i, weight=1)
    return frm

def createSwitch(parent, label, _row, _column, default = False, show = False):
    switch = ctk.CTkSwitch(parent, text=label)
    if(default):
        switch.select()
    if(show):
        switch.grid(row=_row, column=_column, padx=5, pady=5, stick="nsew")
    return switch

def createSlider(parent, label, range, _row, _column = 0, default = 0, _res = 1.0):
    _row = _row*2
    txt = ctk.CTkLabel(parent, text=f"{label}: {default}")
    txt.grid(row=_row, column=_column, pady=(2,0))
    sld = ctk.CTkSlider(
        parent,
        from_=range[0], to=range[1], number_of_steps=(range[1]-range[0])/_res,
        command = lambda v: setLabel(txt, label, v, isInt=(isinstance(_res, int) or _res.is_integer())),
        )
    sld.set(default)
    sld.grid(row=_row+1, column=_column, padx=10, pady=(0, 20), sticky="ew")
    return txt, sld

def color_callback(choice):
    setSliders(util.get_range_data(choice), 0)

def cSpace_callback(choice):
    global COLORSPACE, RANGES
    cs = color_spaces[choice]
    COLORSPACE = cs['labels']
    RANGES = cs['ranges']
    
    H_min_lbl.configure(text=f"{COLORSPACE[0]} Min")
    H_max_lbl.configure(text=f"{COLORSPACE[0]} Max")
    S_min_lbl.configure(text=f"{COLORSPACE[1]} Min")
    S_max_lbl.configure(text=f"{COLORSPACE[1]} Max")
    V_min_lbl.configure(text=f"{COLORSPACE[2]} Min")
    V_max_lbl.configure(text=f"{COLORSPACE[2]} Max")
    
    H_min.configure(from_=RANGES[0][0], to=RANGES[1][0])
    H_max.configure(from_=RANGES[0][0], to=RANGES[1][0])
    S_min.configure(from_=RANGES[0][1], to=RANGES[1][1])
    S_max.configure(from_=RANGES[0][1], to=RANGES[1][1])
    V_min.configure(from_=RANGES[0][2], to=RANGES[1][2])
    V_max.configure(from_=RANGES[0][2], to=RANGES[1][2])
    
    util.set_space(choice)

# Adding the widgets
ctk.CTkLabel(
    window, 
    text="Use the dropdown to select a color and use the sliders to find the range."
    ).grid(row=0, column=0, padx=20, pady=20, sticky="ew")



frmFile = createFrame(window, 1, "ew", columns=2)
frmFile.configure(fg_color="transparent", border_width=0)

cSpace = ctk.CTkOptionMenu(frmFile, values=list(color_spaces.keys()), command=cSpace_callback)
cSpace.set(util.color_calib_data['color_space'])
cSpace.grid(row=0, column=0, columnspan=1, sticky="ew", padx=5, pady=5)

colors = ctk.CTkOptionMenu(frmFile, values=util.get_colors(), command=color_callback)
colors.grid(row=0, column=1, columnspan=1, sticky="ew", padx=5, pady=5)

saveBtn = ctk.CTkButton(frmFile, text="Save", command=lambda: btn_press("save"))
saveBtn.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
resetBtn = ctk.CTkButton(frmFile, text="Reset", fg_color="firebrick3", hover_color="firebrick4", command=lambda: btn_press("reset"))
resetBtn.grid(row=1, column=1, sticky="ew", padx=5, pady=5)

pane = CollapsiblePane(window, expanded_text="<< Hide Options", collapsed_text=">> Show Options", default_state=False)
pane.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=20, pady=(0,20))

frmMenu = createFrame(pane, 2, "ew", columns=3, pack=False)
swContour = createSwitch(frmMenu, "Contour", 0, 0, True, True)
swMaskBlr = createSwitch(frmMenu, "Mask", 0, 1, False, True)
swCollage = createSwitch(frmMenu, "Collage", 0, 2, False, True)


frmOptions = createFrame(pane, 3, "ew", columns=2, pack=False)
threshLbl,      threshSlider =      createSlider(frmOptions, "Selection Threshold", (0, 80), _row = 0, _column = 0, default = 10)
blurLbl,        blurSlider =        createSlider(frmOptions, "Blur Amount",         (0, 21), _row = 0, _column = 1, default=util.get_main_blur_val())
maskBlurLbl,    maskBlurSlider =    createSlider(frmOptions, "Mask Blur Amount",    (0, 21), _row = 1, _column = 0, default = util.get_mask_blur_val())
gammaLbl,       gammaSlider =       createSlider(frmOptions, "Gamma Correction",    (0.1,  3), _row = 1, _column = 1, default = util.get_gamma_val(), _res = 0.01)

pane.updateContent([frmMenu, frmOptions])

RANGES = color_spaces[cSpace.get()]['ranges']
COLORSPACE = color_spaces[cSpace.get()]['labels']

frmRngs = createFrame(window, 3, "ew", True)
H_min_lbl, H_min = createSlider(frmRngs, f"{COLORSPACE[0]} Min", (RANGES[0][0], RANGES[1][0]), _row = 0, _column = 0)
H_max_lbl, H_max = createSlider(frmRngs, f"{COLORSPACE[0]} Max", (RANGES[0][0], RANGES[1][0]), _row = 0, _column = 1, default = 179)
S_min_lbl, S_min = createSlider(frmRngs, f"{COLORSPACE[1]} Min", (RANGES[0][1], RANGES[1][1]), _row = 1, _column = 0)
S_max_lbl, S_max = createSlider(frmRngs, f"{COLORSPACE[1]} Max", (RANGES[0][1], RANGES[1][1]), _row = 1, _column = 1, default = 255)
V_min_lbl, V_min = createSlider(frmRngs, f"{COLORSPACE[2]} Min", (RANGES[0][2], RANGES[1][2]), _row = 2, _column = 0)
V_max_lbl, V_max = createSlider(frmRngs, f"{COLORSPACE[2]} Max", (RANGES[0][2], RANGES[1][2]), _row = 2, _column = 1, default = 255)

# Just started the program or not
first = False

def setSliders(rng, thr):
    H_min.set(util.clamp(rng[0][0] - thr, 0, 179))
    setLabel(H_min_lbl, f"{COLORSPACE[0]} Min", H_min.get())
    H_max.set(util.clamp(rng[1][0] + thr, 0, 179))
    setLabel(H_max_lbl, f"{COLORSPACE[0]} Max", H_max.get())
    S_min.set(util.clamp(rng[0][1] - thr, 0, 255))
    setLabel(S_min_lbl, f"{COLORSPACE[1]} Min", S_min.get())
    S_max.set(util.clamp(rng[1][1] + thr, 0, 255))
    setLabel(S_max_lbl, f"{COLORSPACE[1]} Max", S_max.get())
    V_min.set(util.clamp(rng[0][2] - thr, 0, 255))
    setLabel(V_min_lbl, f"{COLORSPACE[2]} Min", V_min.get())
    V_max.set(util.clamp(rng[1][2] + thr, 0, 255))
    setLabel(V_max_lbl, f"{COLORSPACE[2]} Max", V_max.get())

setSliders(util.get_range_data(colors.get()), 0)
cSpace_callback(cSpace.get())

class CameraNode(Node):
    def __init__(self):
        super().__init__('color_calibration_node')
        self.declare_parameter('topic', '')
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.compressed = True
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3
        )
        
        self.create_subscription(
            CompressedImage if self.compressed else Image,
            'camera/image_raw'+ ("/compressed" if self.compressed else ""),
            self.camera_callback,
            qos_profile)
        self.create_subscription(
            Float32MultiArray,
            'obj_data',
            self.obj_callback,
            10)
        
        # self.create_timer(1/60, self.show_view)        
        self.lastImgTime = None
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.image.fill(255)
        
        self.imsg = None
        self.fps = 0.0
        
        self.objs = []

        cv2.namedWindow('Camera Image', cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow('Camera Image', 640, 480)
        cv2.setMouseCallback('Camera Image', self.mouseClick)

        if(self.topic == ''):
            self.cam = Camera(CAMSRC)
            self.cam.start()
            self.create_timer(1/60, self.getCamFrame)

        self.get_logger().info('CameraNode started, waiting for images...')

    def mouseClick(self, event, x, y, flags, param):
        global first
        if event == cv2.EVENT_LBUTTONUP:
            
            # hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
            hsv = util.process_mask(self.image)[0]
            h, s, v = hsv[y, x, :]
            
            self.get_logger().info(f"Clicked Color: {h}, {s}, {v}")
            
            _min = [min(h, H_min.get()), min(s, S_min.get()), min(v, V_min.get())]
            _max = [max(h, H_max.get()), max(s, S_max.get()), max(v, V_max.get())]
            
            threshold = threshSlider.get()
            
            if first:
                _min = [h - threshold, s - threshold, v - threshold]
                _max = [h + threshold, s + threshold, v + threshold]
                first = False
            setSliders([_min,_max], threshold)

    def camera_callback(self, msg):
        self.imsg = msg
        self.image = None
        if(self.lastImgTime is None):
            self.lastImgTime = time.time()
        else:
            dt = time.time() - self.lastImgTime
            self.lastImgTime = time.time()
            if(dt > 0):
                fps = 1 / dt
                self.fps = self.fps + (fps - self.fps) * 0.1
    
    def getCamFrame(self):
        frame = self.cam.getFrame()
        if(frame is None):
            self.get_logger().error("No Frame")
            return
        self.image = frame
    
    def obj_callback(self, msg: Float32MultiArray):
        if len(msg.data) > 0:
            self.objs = msg.data
    
    def decode(self, msg):
        try:
            if isinstance(msg, CompressedImage):
                np_arr = np.frombuffer(msg.data, np.uint8)
                self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                return True
            elif isinstance(msg, Image):
                image = None
                if msg.encoding == 'rgb8':
                    image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                elif msg.encoding == 'bgr8':
                    image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                elif msg.encoding == 'mono8':
                    image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                self.image = image
                return True
        except Exception as e:
            self.get_logger().error(f'Camera error: {str(e)}')
        return False
    
    def getInRange(self, chn, minVal, maxVal, _max = 179):
        if(minVal <= maxVal):
            return cv2.inRange(chn, minVal, maxVal)
        
        msk = cv2.bitwise_or(cv2.inRange(chn, minVal, _max), cv2.inRange(chn, 0, maxVal))
        return msk
    
    def getCollage(self, frame, cnt):
        _frm = util.process_mask(frame)[0]
        if _frm is None:
            return None
        h, w = _frm.shape[:2]

        cpFrm = frame.copy()
        cv2.drawContours(cpFrm, cnt, -1, (0,255,0), 2)

        # Resize the frame to half its width and height
        main = cv2.resize(cpFrm, (w // 2, h // 2))

        view = cv2.resize(_frm, (w // 2, h // 2))
                
        ch1, ch2, ch3 = cv2.split(view)
        
        msk1 = self.getInRange(ch1, H_min.get(), H_max.get())
        msk2 = self.getInRange(ch2, S_min.get(), S_max.get())
        msk3 = self.getInRange(ch3, V_min.get(), V_max.get())
        

        # Create a blank canvas of the original size
        collage = np.zeros_like(_frm)

        # Place the small images in each quadrant
        collage[0:h//2, 0:w//2] = main
        # Top-right: Channel 1 (e.g., Hue/L) as grayscale
        collage[0:h//2, w//2:w] = cv2.bitwise_and(cv2.merge([msk1, msk1, msk1]), cv2.merge([msk1, msk1, msk1]), mask=msk1)
        # Bottom-left: Channel 2 (e.g., Saturation/A) as grayscale
        collage[h//2:h, 0:w//2] = cv2.bitwise_and(cv2.merge([ch2, ch2, ch2]), cv2.merge([ch2, ch2, ch2]), mask=msk2)
        # Bottom-right: Channel 3 (e.g., Value/B) as grayscale
        collage[h//2:h, w//2:w] = cv2.bitwise_and(cv2.merge([ch3, ch3, ch3]), cv2.merge([ch3, ch3, ch3]), mask=msk3)

        return collage
    
    
    def show_view(self):
        if not running:
            return
        frame = self.image.copy() if self.image is not None else None
        if frame is not None:
            frame = util.preprocess_image(frame, gamma = gammaSlider.get(), blr = blurSlider.get())
            hsv, mask, thresh = util.process_mask(
                frame, 
                [
                    [H_min.get(), S_min.get(), V_min.get()], 
                    [H_max.get(), S_max.get(), V_max.get()]
                ],
                maskBlurSlider.get()
                )
                                
            if(swMaskBlr.get() == 1):
                cv2.imshow("Processed Mask", thresh)
            elif(cv2.getWindowProperty("Processed Mask", cv2.WND_PROP_VISIBLE) == 1):
                cv2.destroyWindow("Processed Mask")
            
            cnt = util.get_contours(thresh)

            if(swContour.get() == 1):
                cv2.drawContours(frame, cnt, -1, (0,255,0), 2)
            
            if(swCollage.get() == 1):
                collage = self.getCollage(frame, cnt)
                if(collage is not None):
                    cv2.imshow("Collage", collage)
            elif(cv2.getWindowProperty("Collage", cv2.WND_PROP_VISIBLE) == 1):
                cv2.destroyWindow("Collage")

            
            cv2.imshow("Camera Image", frame)

            self.imsg = None
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
                exit()
            return
        
        if self.imsg is not None:
            msg = self.imsg
        else:
            self.get_logger().warn('No image message received yet.')
            return
        if self.decode(msg):
            self.show_view()

running = True
def startTk(window: ctk.CTk, node: CameraNode):
    global running
    running = True
    window.protocol("WM_DELETE_WINDOW", lambda: (cv2.destroyAllWindows(), rclpy.shutdown(), window.destroy()))
    
    def keep_running():
        if running:
            node.show_view()
            window.after(int(1000/60), keep_running)
    window.after(100, lambda: keep_running())
    window.mainloop()
    running = False
    
def start_view(node):
    while running:
        node.show_view()
        time.sleep(1/60)

def startNode(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    node_thr = Thread(target=startNode, args=(node,))
    node_thr.daemon = True
    node_thr.start()
    
    # tkinter_thr = Thread(target=startTk, args=(window,))
    # tkinter_thr.daemon = True
    # tkinter_thr.start()

    # view_thr = Thread(target=start_view, args=(node,))
    # view_thr.daemon = True
    # view_thr.start()
    
    startTk(window, node)
    # start_view(node)

if __name__ == '__main__':
    main()