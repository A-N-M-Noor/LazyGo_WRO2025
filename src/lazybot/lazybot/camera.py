import time
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray
import cv2, math, sys
import numpy as np

import threading

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
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
        
        self.lastImgTime = None
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.image.fill(255)
        
        self.imsg = None
        self.fps = 0.0
        
        self.objs = []

        cv2.namedWindow('Camera Image', cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow('Camera Image', 640, 480)
        cv2.setMouseCallback('Camera Image', self.mouseClick)
        
        # self.create_timer(1/30, self.show_view)
        self.get_logger().info('CameraNode started, waiting for images...')

    def mouseClick(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.objs) > 0:
                self.get_logger().info(f"{math.degrees(self.objs[0])}, {x-320} , {y}")

    def camera_callback(self, msg):
        # if(self.lastImgTime is not None and time.time() - self.lastImgTime < 1/30):
        #     return
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
                # self.get_logger().info(f"Camera FPS: {self.fps:.2f}")
    
    def obj_callback(self, msg: Float32MultiArray):
        if len(msg.data) > 0:
            self.objs = msg.data
            
    def show_view(self):
        if self.image is not None:
            cv2.putText(self.image, f'FPS: {self.fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow('Camera Image', self.image)
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
        try:
            if isinstance(msg, CompressedImage):
                np_arr = np.frombuffer(msg.data, np.uint8)
                self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
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
            self.show_view()

        except Exception as e:
            self.get_logger().error(f'Camera error: {str(e)}')

def startNode(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    thr = threading.Thread(target=startNode, args=(node,))
    thr.daemon = True
    thr.start()
    
    while True:
        node.show_view()
        time.sleep(1/60)

if __name__ == '__main__':
    main()