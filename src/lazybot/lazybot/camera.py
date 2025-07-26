import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
import cv2, math
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            self.camera_callback,
            10)
        self.create_subscription(
            Float32MultiArray,
            'obj_data',
            self.obj_callback,
            10)
        
        self.objs = []

        cv2.namedWindow('Camera Image')
        cv2.setMouseCallback('Camera Image', self.mouseClick)
        
        self.get_logger().info('CameraNode started, waiting for images...')

    def mouseClick(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.objs) > 0:
                self.get_logger().info(f"{math.degrees(self.objs[0])}, {x-320} , {y}")

    def camera_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            cv2.imshow('Camera Image', image)
            _key = cv2.waitKey(1) & 0xFF
            if _key == ord('q'):
                self.get_logger().info('Exiting CameraNode...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
        else:
            self.get_logger().warn('Failed to decode image')
    
    def obj_callback(self, msg: Float32MultiArray):
        if len(msg.data) > 0:
            self.objs = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()