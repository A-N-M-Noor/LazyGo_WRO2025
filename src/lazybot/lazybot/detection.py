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

class Detect(Node):
    def __init__(self):
        super().__init__('detect')
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
        
        self.obj_pub = self.create_publisher(DetectionTowerList, 'lazy_towers', 3)
        
        self.closest_pub = self.create_publisher(String, 'closest_obj', 3)
        
        self.frame = None
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.image.fill(255)
        
        self.imsg = None
        self.lastImgTime = None
        self.lastProcTime = None
        self.fps_recv = 0.0
        self.fps_proc = 0.0
        
        self.objs = []
        self.region = [160, 340]
        self.closest = None
        
        self.received_objs = []
        
        self.hud_map = {
            "R": (0, 255, 0),
            "G": (0, 0, 255),
            "P": (0, 255, 255)
        }

        cv2.namedWindow('Camera Image', cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow('Camera Image', 640, 480)
        cv2.setMouseCallback('Camera Image', self.mouseClick)
        self.get_logger().info('DetectionNode started, waiting for images...')

    def mouseClick(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.objs) > 0:
                self.get_logger().info(f"{math.degrees(self.objs[0])}, {x-320} , {y}")

    def camera_callback(self, msg):
        self.imsg = msg
        self.frame = None
        if(self.lastImgTime is None):
            self.lastImgTime = time.time()
        else:
            dt = time.time() - self.lastImgTime
            self.lastImgTime = time.time()
            if(dt > 0):
                fps = 1 / dt
                self.fps_recv = self.fps_recv + (fps - self.fps_recv) * 0.1
    
    def obj_callback(self, msg: Float32MultiArray):
        if len(msg.data) > 0:
            self.received_objs = msg.data
    
    def decode(self):
        if(self.imsg is not None):
            msg = self.imsg
        else:
            return
        try:
            if isinstance(msg, CompressedImage):
                np_arr = np.frombuffer(msg.data, np.uint8)
                self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                return True
            elif isinstance(msg, Image):
                frame = None
                if msg.encoding == 'rgb8':
                    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif msg.encoding == 'bgr8':
                    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                elif msg.encoding == 'mono8':
                    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                self.frame = frame
                return True

        except Exception as e:
            self.get_logger().error(f'Camera error: {str(e)}')
        return False

    def detect(self):
        frame = None
        try:
            frame = self.frame.copy()
        except Exception as e:
            return
        if(frame is None):
            return
        self.objs = []
        hsv, mask, thresh = util.process_mask(
            frame, 
            util.get_range_data('green'), 
            util.get_mask_blur_val(),
            crop = self.region
        )
        
        green_cont = util.get_contours(thresh)
        
        for cnt in green_cont:
            x, y, w, h = cv2.boundingRect(cnt)
            self.objs.append(('G', x+w//2, y+h//2+self.region[0], w, h))

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

        self.closest = None
        msg = String()
        msg.data = "N"
        for obj in self.objs:
            if(obj[0] == 'P'):
                continue
            if (self.closest is None or obj[4] > self.closest[4]) and obj[4] > 12:
                self.closest = obj
                msg.data = self.closest[0]
        self.closest_pub.publish(msg)
        
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
        if self.frame is not None:
            self.image = self.frame.copy()
            
            if(self.lastProcTime is None):
                self.lastProcTime = time.time()
            else:
                dt = time.time() - self.lastProcTime
                self.lastProcTime = time.time()
                if(dt > 0):
                    fps = 1 / dt
                    self.fps_proc = self.fps_proc + (fps - self.fps_proc) * 0.1

            cv2.line(self.image, (0, self.region[0]), (self.image.shape[1], self.region[0]), (255, 255, 0), 2)
            cv2.line(self.image, (0, self.region[1]), (self.image.shape[1], self.region[1]), (255, 255, 0), 2)

            for obj in self.objs:
                color = self.hud_map.get(obj[0], (255, 0, 0))
                x, y, w, h = obj[1], obj[2], obj[3], obj[4]
                cv2.rectangle(self.image, (x-w//2, y-h//2), (x + w//2, y + h//2), color, 1)
            if(self.closest is not None):
                color = (0, 255, 0) if self.closest[0] == 'R' else (0, 0, 255)
                cv2.circle(self.image, (self.closest[1], self.closest[2]), 5, color, 1, cv2.LINE_AA)

            cv2.putText(self.image, f'V: {self.fps_recv:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(self.image, f'P: {self.fps_proc:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow('Camera Image', self.image)
            self.imsg = None
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
                exit()

def startNode(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    node = Detect()
    thr = threading.Thread(target=startNode, args=(node,))
    thr.daemon = True
    thr.start()
    
    while True:
        decoded = node.decode()
        if(decoded):
            node.detect()
        node.show_view()
        time.sleep(1/60)

if __name__ == '__main__':
    main()