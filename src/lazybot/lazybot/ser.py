import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int8, Int16MultiArray
from geometry_msgs.msg import Vector3

from math import radians

import serial, time
from serial.tools import list_ports
import threading

class SerNode(Node):
    """
    Serial Communication Node.
    Acts as a bridge between the ROS 2 high-level logic (running on PC/Pi) 
    and the low-level hardware controller (ESP32).
    
    Responsibilities:
    1. Sends motor commands (Throttle, Steering) to ESP32.
    2. Sends servo angles (Camera mount) to ESP32.
    3. Receives Odometry data (X, Y, Theta) from ESP32.
    4. Handles handshake and state synchronization commands.
    """
    def __init__(self):
        super().__init__('ser')

        # --- Publishers & Subscribers ---
        
        # High-level system commands
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 1)
        
        # Odometry output (received from ESP32)
        self.pos_pub = self.create_publisher(Vector3, '/lazypos', 10)
        
        # Actuator inputs
        self.thr_sub = self.create_subscription(Float32, '/throttle', self.thr_callback, 1)
        self.str_sub = self.create_subscription(Float32, '/steer', self.steer_callback, 1)
        self.cam_sub = self.create_subscription(Int8, '/cam_servo', self.cam_callback, 1)
        
        # Calibration & Parking inputs
        self.offs_sub = self.create_subscription(Vector3, '/initial_offset', self.offs_callback, 10)
        self.dir_dst_sub = self.create_subscription(Int16MultiArray, '/dir_dst', self.dir_dst_callback, 1)
        self.esp_cmd_sub = self.create_subscription(Int8, '/esp_cmd', self.esp_cmd_callback, 1)

        # --- State Variables ---
        self.thr = 0.0
        self.str = 0.0
        self.cam = 0.0
        self.ser = None
        self.isDone = False # Flag to stop sending drive commands during handshake/parking

        self.offx = 0.0
        self.offy = 0.0
        
        # Initialize Serial Connection
        self.establish_ser()
            
        if self.ser is not None:
            self.get_logger().info('Serial connection established.')
            
            # Timer for sending periodic motor updates (5Hz)
            self.timer = self.create_timer(0.2, self.timer_callback)

            # Background thread for reading incoming serial data
            self.readThr = threading.Thread(target=self.ser_read)
            self.readThr.daemon = True
            self.readThr.start()
        else:
            self.get_logger().error('Failed to establish serial connection to ESP32')

    def offs_callback(self, msg: Vector3):
        """Updates the coordinate offset to align ESP32 odometry with the map."""
        self.offx = msg.x
        self.offy = msg.y
        self.get_logger().info(f'Received initial offset: x={self.offx}, y={self.offy}')
    
    def cmd_callback(self, msg: String):
        """
        Translates high-level ROS string commands into byte codes for the ESP32.
        Protocol: Key (Byte) -> [Optional Value (Byte)]
        """
        if(msg.data == "RunEnd"):
            self.send_val(30) # Code 30: Stop/End Run
            self.isDone = True
        elif(msg.data == "NormalizeIMU"):
            self.send_val(30) # Code 30: Reset IMU/Stop
            self.get_logger().info('Sent NormalizeIMU command to ESP32')
        elif(msg.data == "start_open"):
            self.cmd_pub.publish(String(data="start"))
            self.send_val(5) # Code 5: Start Challenge Mode
            self.get_logger().info('Sent start command to ESP32')
            self.isDone = True
        elif(msg.data == "Cornering"):
            self.send_val(47) # Code 47: Enable Cornering Indication
        elif(msg.data == "CAM_OK"):
            self.get_logger().info('Received CAM_OK command')
            self.send_val(49) # Code 49: Camera Handshake OK
        
        elif(msg.data.startswith("MOVE:")):
            # Handle precise movement commands (e.g., "MOVE:0.5")
            try:
                distance = float(msg.data.split(":")[1])
                move_val = int(distance * 100)  # Convert m to cm
                
                # Clamp value to byte range logic
                if(move_val < -100): move_val = -100
                if(move_val > 100): move_val = 100
                move_val = move_val + 150 
                
                self.send_val(7, move_val) # Code 7: Move Distance
                self.get_logger().info(f'Sent MOVE command with distance: {move_val}')
            except ValueError:
                self.get_logger().error(f'Invalid MOVE command format: {msg.data}')

    def dir_dst_callback(self, msg: Int16MultiArray):
        """Sends LiDAR distances to ESP32 for parking alignment."""
        if len(msg.data) == 5:
            # Clamp distances to 200cm max
            l  = min(msg.data[0], 200)
            fl = min(msg.data[1], 200)
            f  = min(msg.data[2], 200)
            fr = min(msg.data[3], 200)
            r  = min(msg.data[4], 200)
            
            # Send with offset +50 (Protocol specific)
            self.send_val(31, l+50)
            self.send_val(32, fl+50)
            self.send_val(33, f+50)
            self.send_val(34, fr+50)
            self.send_val(35, r+50)
    
    def printSerialDetails(self, port):
        """Debug helper to identify USB devices."""
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
        """
        Auto-detects and connects to the ESP32.
        Filters ports to avoid connecting to the LiDAR or Camera by mistake.
        """
        if self.ser is None:
            ports_to_try = list_ports.comports()
            
            for port in ports_to_try:
                self.printSerialDetails(port)
                if port.device.startswith("/dev/ttyUSB") or port.device.startswith("/dev/ttyACM"):
                    # Filter out CP2102N (Used by LiDAR)
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

    def send_val(self, k, v = None):
        """
        Low-level serial write.
        Args:
            k (int): Key/Command byte.
            v (int, optional): Value byte.
        """
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Serial connection is not available.')
            return
        try:
            self.ser.write(k.to_bytes(1, 'little'))
            if v is not None:
                self.ser.write(v.to_bytes(1, 'little'))
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')
    
    def esp_cmd_callback(self, msg: Int8):
        """Direct pass-through for integer commands."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Serial connection is not available.')
            return
        try:
            v = int(msg.data)
            if(v >= 0 and v <= 49):
                self.ser.write(msg.data.to_bytes(1, 'little'))
                self.get_logger().info(f"Sent byte {v} to ESP Side")
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')
    
    def thr_callback(self, msg):
        self.thr = msg.data
    
    def cam_callback(self, msg):
        """Updates camera servo angle immediately."""
        self.cam = msg.data
        # Map angle to byte range
        self.send_val(17, int(self.cam)+140)

    def steer_callback(self, msg):
        self.str = msg.data
        
    def pub_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def timer_callback(self):
        """
        Periodic loop to send drive commands.
        Prevents flooding the serial bus by sending at a fixed rate.
        """
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Serial connection is not available.')
            return
        try:
            # If in handshake/parking mode, send keep-alive/idle code
            if(not self.isDone):
                self.send_val(48)
                return
        
            # Send Throttle (Key 15) and Steer (Key 16)
            self.send_val(15, int(self.thr * 100)+150)
            self.send_val(16, int(self.str * 100)+150)
            
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')

    def ser_read(self):
        """
        Background thread for reading serial data.
        Parses status strings and odometry arrays.
        """
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                self.get_logger().error('Serial connection is not available.')
                self.establish_ser()
                continue
            
            if(self.ser.in_waiting > 0):
                try:
                    data = self.ser.readline().decode('utf-8').strip()
                    if data:
                        # --- Handle Status Messages ---
                        if data == "Start":
                            self.get_logger().info('Starting Bot')
                            self.pub_cmd("start")
                            self.isDone = True
                        if data == "Boot":
                            self.get_logger().info('Boot Bot')
                            self.pub_cmd("Boot")
                            self.isDone = False
                        if data == "Turned":
                            self.pub_cmd("Turned")
                        if data == "Done":
                            self.pub_cmd("Done")
                            self.isDone = True
                        if data == "Oriented":
                            self.pub_cmd("Oriented")
                        if data == "ParkReady":
                            self.pub_cmd("ParkReady")
                        if data == "Inside":
                            self.pub_cmd("Inside")
                        if data == "CONF_CAM":
                            self.pub_cmd("CONF_CAM")
                        
                        # Debug prints from ESP32
                        if data.startswith('>'):
                            self.get_logger().info("Got Print: \n" + data)
                        
                        # --- Handle Odometry Data [x, y, theta] ---
                        if data.startswith('[') and data.endswith(']'):
                            try:
                                parts = data[1:-1].split(',')
                                if len(parts) == 3:
                                    x = float(parts[0])
                                    y = float(parts[1])
                                    theta = float(parts[2])
                                    
                                    # Apply offset and publish
                                    pos_msg = Vector3()
                                    pos_msg.x = x + self.offx
                                    pos_msg.y = y + self.offy
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