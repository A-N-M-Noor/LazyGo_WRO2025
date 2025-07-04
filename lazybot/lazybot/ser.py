import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial, time

class SerNode(Node):
    def __init__(self):
        super().__init__('ser')
        self.subscription1 = self.create_subscription(Float32, '/throttle', self.thr_callback, 10)
        self.subscription2 = self.create_subscription(Float32, '/steer', self.steer_callback, 10)

        self.thr = 0.0
        self.str = -1.0
        self.ser = None
        
        if self.ser is None:
            # Try both ttyACM and ttyUSB ports
            ports_to_try = [f'/dev/ttyACM{i}' for i in range(10)]
            
            for port in ports_to_try:
                try:
                    self.ser = serial.Serial(port=port, baudrate=115200)
                    self.get_logger().info(f'Successfully connected to serial port: {port}')
                    break
                except serial.SerialException:
                    self.ser = None
                    self.get_logger().warn(f'Failed to connect to {port}, trying next port...')
            
        if self.ser is not None:
            self.get_logger().info('Serial connection established.')
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            self.get_logger().error('Failed to establish serial connection to ESP32')


    def thr_callback(self, msg):
        self.thr = msg.data

    def steer_callback(self, msg):
        self.str = msg.data
    
    def timer_callback(self):
        if self.ser is not None and self.ser.is_open:
            try:
                command = f'{int(self.thr*100)},{int(self.str*100)}\n'
                self.ser.write(command.encode('utf-8'))
                self.ser.flush()
                
                # Optional: Log the sent command for debugging
                self.get_logger().info(f'Sent command: {command.strip()}')
                
            except Exception as e:
                self.get_logger().error(f'Error writing to serial: {e}')
                # Try to reconnect if write fails
                self.ser.close()
                self.ser = None
                return

            # Read response from ESP32
            if(self.ser.in_waiting > 0):
                data = ''
                t = time.time()
                while time.time() - t < 1:
                    try:
                        c = self.ser.read()
                        data = data + c.decode('utf-8')
                        if c == b'\n':  # Assuming the response ends with a newline
                            break
                    except Exception as e:
                        self.get_logger().error(f'Error reading from serial: {e}')
                if data:
                    self.get_logger().info(f'Received data: {data}')
        else:
            self.get_logger().warn('Serial connection not available')

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