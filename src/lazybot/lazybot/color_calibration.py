import cv2
import customtkinter as ctk
from helper.camera_capture import Camera
from threading import Thread
import helper.util as util

# Create a window
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("green")  # Themes: "blue" (standard), "green", "dark-blue"
window = ctk.CTk()
window.title("HSV Calibration Tool")
window.attributes('-topmost',True)

# Configuring the grid system
window.grid_columnconfigure(0, weight=1)

def btn_press(cmd):
    global first
    print(f"{cmd} happened")
    
    if(cmd == "reset"):
        setSliders([[0,0,0],[179,255,255]], 0)
        first = True
    
    if(cmd == "save"):
        util.save_range_data(
            colors.get(), 
            [
                [int(H_min.get()), int(S_min.get()), int(V_min.get())], 
                [int(H_max.get()), int(S_max.get()), int(V_max.get())]
            ],
            blurSlider.get(),
            maskBlurSlider.get()
        )
        

def setLabel(label, initial, value):
    label.configure(text=f"{initial}: {int(value)}")

def createFrame(parent, _row, stick="nsew", column2 = False):
    frm = ctk.CTkFrame(parent, border_width=1, height = 0, border_color="gray28")
    frm.grid(row=_row, column=0, padx=20, pady=(0,20), sticky=stick)
    frm.grid_columnconfigure(0, weight=1)
    if(column2):
        frm.grid_columnconfigure(1, weight=1)
    return frm

def createSwitch(parent, label, _row, _column, default = False):
    switch = ctk.CTkSwitch(parent, text=label)
    if(default):
        switch.select()
    switch.grid(row=_row, column=_column, padx=5, pady=5, stick="nsew")
    return switch

def createSlider(parent, label, range, _row, _column = 0, default = 0):
    _row = _row*2
    txt = ctk.CTkLabel(parent, text=f"{label}: {default}")
    txt.grid(row=_row, column=_column, pady=(2,0))
    sld = ctk.CTkSlider(
        parent,
        from_=range[0], to=range[1], number_of_steps=range[1]-range[0],
        command = lambda v: setLabel(txt, label, v)
        )
    sld.set(default)
    sld.grid(row=_row+1, column=_column, padx=10, pady=(0, 20), sticky="ew")
    return txt, sld

def color_callback(choice):
    setSliders(util.get_range_data(choice), 0)

# Adding the widgets
ctk.CTkLabel(
    window, 
    text="Use the dropdown to select a color and use the sliders to find the range."
    ).grid(row=0, column=0, padx=20, pady=20, sticky="ew")

frmMenu = createFrame(window, 1, "ew", True)
frmMenu.configure(fg_color="transparent", border_width=0)
colors = ctk.CTkOptionMenu(frmMenu, values=util.get_colors(), command=color_callback)
colors.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

saveBtn = ctk.CTkButton(frmMenu, text="Save", command=lambda: btn_press("save"))
saveBtn.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
resetBtn = ctk.CTkButton(frmMenu, text="Reset", fg_color="firebrick3", hover_color="firebrick4", command=lambda: btn_press("reset"))
resetBtn.grid(row=1, column=1, sticky="ew", padx=5, pady=5)

swMask = createSwitch(frmMenu, "Show Mask", 2, 0)
swMaskBlr = createSwitch(frmMenu, "Show Processed Mask", 2, 1, True)
swContour = createSwitch(frmMenu, "Draw Contour", 3, 0)
swBox = createSwitch(frmMenu, "Draw Bounding Box", 3, 1, True)
swMinRect = createSwitch(frmMenu, "Draw MinAreaRect", 4, 0)
swLine = createSwitch(frmMenu, "Draw Fitted Line", 4, 1)


frmOptions = createFrame(window, 2, "ew")
threshLbl, threshSlider = createSlider(frmOptions, "Selection Threshold", (0, 80), 0, default = 10)
blurLbl, blurSlider = createSlider(frmOptions, "Blur Amount", (0, 21), 1, default=util.get_main_blur_val())
maskBlurLbl, maskBlurSlider = createSlider(frmOptions, "Mask Blur Amount", (0, 21), 2, default = util.get_mask_blur_val())

frmRngs = createFrame(window, 3, "ew", True)
H_min_lbl, H_min = createSlider(frmRngs, "HUE Min", (0, 179), _row = 0, _column = 0)
H_max_lbl, H_max = createSlider(frmRngs, "HUE Max", (0, 179), _row = 0, _column = 1, default = 179)
S_min_lbl, S_min = createSlider(frmRngs, "Saturation Min", (0, 255), _row = 1, _column = 0)
S_max_lbl, S_max = createSlider(frmRngs, "Saturation Max", (0, 255), _row = 1, _column = 1, default = 255)
V_min_lbl, V_min = createSlider(frmRngs, "Value Min", (0, 255), _row = 2, _column = 0)
V_max_lbl, V_max = createSlider(frmRngs, "Value Max", (0, 255), _row = 2, _column = 1, default = 255)


# Just started the program or not
first = True

# Start capturing the camera
cam = Camera(0)
cam.start()

running = True

def setSliders(rng, thr):
    H_min.set(util.clamp(rng[0][0] - thr, 0, 179))
    setLabel(H_min_lbl, "HUE Min", H_min.get())
    H_max.set(util.clamp(rng[1][0] + thr, 0, 179))
    setLabel(H_max_lbl, "HUE Max", H_max.get())
    S_min.set(util.clamp(rng[0][1] - thr, 0, 255))
    setLabel(S_min_lbl, "Saturation Min", S_min.get())
    S_max.set(util.clamp(rng[1][1] + thr, 0, 255))
    setLabel(S_max_lbl, "Saturation Max", S_max.get())
    V_min.set(util.clamp(rng[0][2] - thr, 0, 255))
    setLabel(V_min_lbl, "Value Min", V_min.get())
    V_max.set(util.clamp(rng[1][2] + thr, 0, 255))
    setLabel(V_max_lbl, "Value Max", V_max.get())

def on_mouse_click(event, x, y, flags, param):
    global first
    if event == cv2.EVENT_LBUTTONUP:
        # Get pixel value at (x, y)
        frame = cam.getFrame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[y, x, :]
        
        _min = [min(h, H_min.get()), min(s, S_min.get()), min(v, V_min.get())]
        _max = [max(h, H_max.get()), max(s, S_max.get()), max(v, V_max.get())]
        
        threshold = threshSlider.get()
        
        if first:
            _min = [h, s - threshold*6, v - threshold*6]
            _max = [h, s + threshold*6, v + threshold*6]
            first = False
        setSliders([_min,_max], threshold)

def viewing():
    while(running):
        frame = cam.getFrame(blr=blurSlider.get())
        if(frame is not None):
            cv2.imshow("cap", frame)
            hsv, mask, thresh = util.process_mask(
                frame, 
                [
                    [H_min.get(), S_min.get(), V_min.get()], 
                    [H_max.get(), S_max.get(), V_max.get()]
                ],
                maskBlurSlider.get()
                )
                
            if(swMask.get() == 1):
                cv2.imshow("Mask", mask)
            elif(cv2.getWindowProperty("Mask", cv2.WND_PROP_VISIBLE) == 1):
                cv2.destroyWindow("Mask")
                
            if(swMaskBlr.get() == 1):
                cv2.imshow("Processed Mask", thresh)
            elif(cv2.getWindowProperty("Processed Mask", cv2.WND_PROP_VISIBLE) == 1):
                cv2.destroyWindow("Processed Mask")
                
            cv2.setMouseCallback("cap", on_mouse_click)
            
        key = cv2.waitKey(1)
        if key==ord('q'):
            cam.stop()
            cv2.destroyAllWindows()
            window.destroy()
            exit(1)
            break

viewThread=Thread(target=viewing,args=())
viewThread.daemon=True
viewThread.start()

# Run the mainloop for CTk
window.mainloop()
running = False
cam.stop()