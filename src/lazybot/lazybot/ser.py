import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int8
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
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 1)
        self.pos_pub = self.create_publisher(Vector3, '/lazypos', 10)
        self.thr_sub = self.create_subscription(Float32, '/throttle', self.thr_callback, 1)
        self.str_sub = self.create_subscription(Float32, '/steer', self.steer_callback, 1)
        self.cam_sub = self.create_subscription(Int8, '/cam_servo', self.cam_callback, 1)
        
        self.esp_cmd_sub = self.create_subscription(Int8, '/esp_cmd', self.esp_cmd_callback, 1)

        self.thr = 0.0
        self.str = 0.0
        self.cam = 0.0
        self.ser = None
        self.isDone = False
        
        self.establish_ser()
            
        if self.ser is not None:
            self.get_logger().info('Serial connection established.')
            self.timer = self.create_timer(0.2, self.timer_callback)

            self.readThr = threading.Thread(target=self.ser_read)
            self.readThr.daemon = True
            self.readThr.start()
        else:
            self.get_logger().error('Failed to establish serial connection to ESP32')

    
    def cmd_callback(self, msg: String):
        if(msg.data == "completeR"):
            self.send_val(6, 100)
            self.isDone = True
        elif(msg.data == "completeL"):
            self.send_val(7, 100)
            self.isDone = True
        elif(msg.data == "start_open"):
            self.cmd_pub.publish(String(data="start"))
            self.send_val(5, 100)
            self.get_logger().info('Sent start command to ESP32')
            self.isDone = True
    
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
                self.printSerialDetails(port)
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

    def send_val(self, k, v):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Serial connection is not available.')
            return
        try:
            self.ser.write(k.to_bytes(1, 'little'))
            self.ser.write(v.to_bytes(1, 'little'))
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')
    
    def esp_cmd_callback(self, msg: Int8):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Serial connection is not available.')
            return
        try:
            v = int(msg.data)
            if(v >= 0 and v <= 49):
                self.ser.write(msg.data.to_bytes(1, 'little'))
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')
    
    def thr_callback(self, msg):
        self.thr = msg.data
    
    def cam_callback(self, msg):
        self.cam = msg.data
        self.send_val(17, int(self.cam)+140)

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
        try:
            if(not self.isDone):
                v = 10
                self.ser.write(v.to_bytes(1, 'little'))
                return
        
            self.send_val(15, int(self.thr * 100)+150)
            self.send_val(16, int(self.str * 100)+150)
            
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')

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
                            self.isDone = True
                        if data == "Boot":
                            self.get_logger().info('Boot Bot')
                            self.pub_cmd("Boot")
                            self.isDone = False
                        
                        if data == "Turned":
                            self.get_logger().info('Turned Bot')
                            self.pub_cmd("Turned")
                        
                        if data == "Done":
                            self.get_logger().info('Done Bot')
                            self.pub_cmd("Done")
                            self.isDone = True
                        
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