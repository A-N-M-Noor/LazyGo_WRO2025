import time
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray, String
import cv2, math
import numpy as np

from lazy_interface.msg import DetectionTowerInfo, DetectionTowerList

import threading
import lazybot.helper.util as util
from lazybot.helper.camera_capture import Camera

class Detect(Node):
    """
    Camera Detection Node.
    Captures frames from the USB camera, processes them to find colored objects (towers),
    and publishes the results to the control system.
    """
    def __init__(self):
        super().__init__('detect_cam')
        self.display = False # Set to True to enable debug window
        
        self.compressed = True
        
        # --- Publishers & Subscribers ---
        # Command interface for system status checks
        self.cmd_pub = self.create_publisher(String, 'cmd', 10)
        self.cmd_sub = self.create_subscription(String, 'cmd', self.cmd_callback, 10)
        
        # Publishes list of all detected towers
        self.obj_pub = self.create_publisher(DetectionTowerList, 'lazy_towers', 3)
        
        # Publishes only the closest object (used for immediate decision making)
        self.closest_pub = self.create_publisher(String, 'closest_obj', 3)
        
        # --- Camera Setup ---
        # Initialize threaded camera capture
        self.cam = Camera("/dev/v4l/by-id/usb-046d_081b_61C8A860-video-index0")
        
        # --- State Variables ---
        self.frame = None
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.image.fill(255)
        self.sent_img_conf = False # Flag to confirm camera is active

        self.imsg = None
        self.lastImgTime = None
        self.lastProcTime = None
        self.fps_recv = 0.0
        self.fps_proc = 0.0
        
        self.objs = []
        self.region = [60, 440] # Vertical ROI (Region of Interest) to ignore floor/ceiling
        self.closest = None
        
        self.received_objs = []
        
        # Color map for debug display (BGR format)
        self.hud_map = {
            "R": (0, 255, 0),   # Red objects shown in Green box (contrast)
            "G": (0, 0, 255),   # Green objects shown in Red box
            "P": (0, 255, 255)  # Purple objects shown in Yellow box
        }

        if(self.display):
            cv2.namedWindow('Camera Image', cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Camera Image', 640, 480)
            cv2.setMouseCallback('Camera Image', self.mouseClick)

        self.cam.start()

        self.get_logger().info('DetectionNode started, waiting for images...')

    def cmd_callback(self, msg: String):
        """Resets confirmation flag if requested by system."""
        if(msg.data == "CONF_CAM"):
            self.sent_img_conf = False
    
    def mouseClick(self, event, x, y, flags, param):
        """Debug helper: Prints coordinates of clicks."""
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.objs) > 0:
                self.get_logger().info(f"{math.degrees(self.objs[0])}, {x-320} , {y}")

    def detect(self):
        """
        Main Detection Loop.
        1. Fetches latest frame.
        2. Applies color thresholding for Green, Red, and Purple.
        3. Finds contours and bounding boxes.
        4. Identifies the closest object based on height (larger = closer).
        5. Publishes results.
        """
        frame = self.cam.getFrame()
        if(frame is None):
            self.get_logger().error("No Frame")
            return

        # Send "CAM_OK" once to signal system that camera is live
        if(not self.sent_img_conf):
            self.cmd_pub.publish(String(data="CAM_OK"))
            self.sent_img_conf = True

        self.frame = frame
        self.objs = []
        
        # --- 1. Detect Green Objects ---
        hsv, mask, thresh = util.process_mask(
            frame, 
            util.get_range_data('green'), 
            util.get_mask_blur_val(),
            crop = self.region # Crop to ROI
        )
        
        green_cont = util.get_contours(thresh)
        
        for cnt in green_cont:
            x, y, w, h = cv2.boundingRect(cnt)
            # Store as tuple: (Color, CenterX, CenterY, Width, Height)
            # Note: Y is adjusted by self.region[0] because of cropping
            self.objs.append(('G', x+w//2, y+h//2+self.region[0], w, h))

        # --- 2. Detect Red Objects ---
        # Reuse HSV image to save computation time
        hsv, mask, thresh = util.process_mask(
            frame, 
            util.get_range_data('red'), 
            util.get_mask_blur_val(),
            hsv = hsv
        )
        
        red_cont = util.get_contours(thresh)
        
        for cnt in red_cont:
            x, y, w, h = cv2.boundingRect(cnt)
            self.objs.append(('R', x+w//2, y+h//2+self.region[0], w, h))
        
        # --- 3. Detect Purple Objects ---
        hsv, mask, thresh = util.process_mask(
            frame, 
            util.get_range_data('purple'), 
            util.get_mask_blur_val(),
            hsv = hsv
        )

        purple_cont = util.get_contours(thresh)
        
        for cnt in purple_cont:
            x, y, w, h = cv2.boundingRect(cnt)
            self.objs.append(('P', x+w//2, y+h//2+self.region[0], w, h))

        # --- 4. Find Closest Object ---
        self.closest = None
        msg = String()
        msg.data = "N" # Default: None
        for obj in self.objs:
            if(obj[0] == 'P'):
                continue # Ignore purple (start/finish markers) for collision logic
            
            # Heuristic: Larger height = Closer object
            # Filter small noise (height > 30)
            if (self.closest is None or obj[4] > self.closest[4]) and obj[4] > 30:
                self.closest = obj
                msg.data = self.closest[0]
        self.closest_pub.publish(msg)
        
        # --- 5. Publish All Objects ---
        t_infos = DetectionTowerList()
        
        for obj in self.objs:
            t_info = DetectionTowerInfo()
            t_info.color = obj[0]
            t_info.x = obj[1]
            t_info.y = obj[2]
            t_info.width = obj[3]
            t_info.height = obj[4]
            t_infos.towers.append(t_info)
        self.obj_pub.publish(t_infos)

    def show_view(self):
        """
        Debug Visualization.
        Draws bounding boxes and FPS stats on the image.
        Only runs if self.display is True.
        """
        if(not self.display):
            return
        if self.frame is not None:
            self.image = self.frame.copy()
            
            # Calculate Processing FPS
            if(self.lastProcTime is None):
                self.lastProcTime = time.time()
            else:
                dt = time.time() - self.lastProcTime
                self.lastProcTime = time.time()
                if(dt > 0):
                    fps = 1 / dt
                    self.fps_proc = self.fps_proc + (fps - self.fps_proc) * 0.1

            # Draw ROI lines
            cv2.line(self.image, (0, self.region[0]), (self.image.shape[1], self.region[0]), (255, 255, 0), 2)
            cv2.line(self.image, (0, self.region[1]), (self.image.shape[1], self.region[1]), (255, 255, 0), 2)

            # Draw bounding boxes
            for obj in self.objs:
                color = self.hud_map.get(obj[0], (255, 0, 0))
                x, y, w, h = obj[1], obj[2], obj[3], obj[4]
                cv2.rectangle(self.image, (x-w//2, y-h//2), (x + w//2, y + h//2), color, 1)
            
            # Highlight closest object
            if(self.closest is not None):
                color = (0, 255, 0) if self.closest[0] == 'R' else (0, 0, 255)
                cv2.circle(self.image, (self.closest[1], self.closest[2]), 5, color, 1, cv2.LINE_AA)

            # Draw FPS
            cv2.putText(self.image, f'V: {self.fps_recv:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(self.image, f'P: {self.fps_proc:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            cv2.imshow('Camera Image', self.image)
            self.imsg = None
            
            # Handle Quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
                exit()

def startNode(node):
    """Runs ROS 2 spinning in a background thread."""
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    node = Detect()
    
    # Start ROS communication in a separate thread
    thr = threading.Thread(target=startNode, args=(node,))
    thr.daemon = True
    thr.start()
    
    # Main loop for detection and visualization
    while True:
        node.detect()
        node.show_view()
        time.sleep(1/60) # Cap at ~60 FPS

if __name__ == '__main__':
    main()