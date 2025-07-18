import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial, time
import threading

class SerNode(Node):
    """Node to handle serial communication with an ESP32."""
    def __init__(self):
        super().__init__('ser')

        self.subscription1 = self.create_subscription(Float32, '/throttle', self.thr_callback, 1)
        self.subscription2 = self.create_subscription(Float32, '/steer', self.steer_callback, 1)

        self.thr = 0.0
        self.str = 0.0
        self.ser = None
        
        self.establish_ser()
            
        if self.ser is not None:
            self.get_logger().info('Serial connection established.')
            self.timer = self.create_timer(0.2, self.timer_callback)

            self.readThr = threading.Thread(target=self.ser_read)
            self.readThr.daemon = True
            self.readThr.start()
        else:
            self.get_logger().error('Failed to establish serial connection to ESP32')

    def establish_ser(self):
        if self.ser is None:
            ports_to_try = [f'/dev/ttyACM{i}' for i in range(10)]
            
            for port in ports_to_try:
                try:
                    self.ser = serial.Serial(port=port, baudrate=115200)
                    self.get_logger().info(f'Successfully connected to serial port: {port}')
                    break
                except serial.SerialException:
                    self.ser = None
                    self.get_logger().warn(f'Failed to connect to {port}, trying next port...')

    def thr_callback(self, msg):
        self.thr = msg.data

    def steer_callback(self, msg):
        self.str = msg.data
    
    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Serial connection is not available.')
            return

        v = int(self.thr * 50) + 100
        self.ser.write(v.to_bytes(1, 'little'))

        v = int(self.str * 50) + 200
        self.ser.write(v.to_bytes(1, 'little'))

    def ser_read(self):
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                self.get_logger().error('Serial connection is not available.')
                self.establish_ser()
                continue
            if(self.ser.in_waiting > 0):
                try:
                    data = self.ser.readline()
                    self.get_logger().info(f'Received data: {data.decode("utf-8")}')
                except Exception as e:
                    self.get_logger().error(f'Error reading from serial: {e}')

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial connection closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()