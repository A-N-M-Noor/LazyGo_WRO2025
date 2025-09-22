import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3

from math import radians

import serial, time
from serial.tools import list_ports
import threading

class SerNode(Node):
    """Node to handle serial communication with an ESP32."""
    def __init__(self):
        super().__init__('ser')

        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        self.pos_pub = self.create_publisher(Vector3, '/lazypos', 10)
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

    def printSerialDetails(self, port):
        print(f'Port device: {port.device}')
        print(f'Port name: {port.name}')
        print(f'Port description: {port.description}')
        print(f'Port hwid: {port.hwid}')
        print(f'Port vid: {port.vid}')
        print(f'Port pid: {port.pid}')
        print(f'Port serial_number: {port.serial_number}')
        print(f'Port location: {port.location}')
        print(f'Port manufacturer: {port.manufacturer}')
        print(f'Port product: {port.product}')
        print(f'Port interface: {port.interface}')

    def establish_ser(self):
        if self.ser is None:
            ports_to_try = list_ports.comports()
            
            for port in ports_to_try:
                if port.device.startswith("/dev/ttyUSB") or port.device.startswith("/dev/ttyACM"):
                    if port.product and not port.product.startswith("CP2102N"):
                        try:
                            self.ser = serial.Serial(port=port.device, baudrate=115200)
                            self.get_logger().info(f'Successfully connected to serial port: {port}')
                            break
                        except serial.SerialException:
                            self.ser = None
                            self.get_logger().warn(f'Failed to connect to {port}, trying next port...')
                    else:
                        self.get_logger().info(f'Skipping port (not target device): {port.device} - Product: {port.product}')
                else:
                    self.get_logger().info(f'Skipping non-USB port: {port.device}')

    def thr_callback(self, msg):
        self.thr = msg.data

    def steer_callback(self, msg):
        self.str = msg.data
    def pub_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

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
                    data = self.ser.readline().decode('utf-8').strip()
                    if data:
                        if data == "Start":
                            self.get_logger().info('Starting Bot')
                            self.pub_cmd("start")
                        
                        if data.startswith('[') and data.endswith(']'):
                            try:
                                parts = data[1:-1].split(',')
                                if len(parts) == 3:
                                    x = float(parts[0])
                                    y = float(parts[1])
                                    theta = float(parts[2])
                                    pos_msg = Vector3()
                                    pos_msg.x = x
                                    pos_msg.y = y
                                    pos_msg.z = radians(theta)
                                    self.pos_pub.publish(pos_msg)
                                else:
                                    self.get_logger().warn(f'Unexpected position format: {data}')
                            except ValueError as ve:
                                self.get_logger().error(f'Error parsing position data: {ve}')

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