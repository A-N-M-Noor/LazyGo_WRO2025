import rclpy
from rclpy.node import Node

from threading import Thread
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from lazy_interface.msg import BotDebugInfo, LidarTowerInfo

from std_msgs.msg import String, Int8, Int16MultiArray
from math import pi, radians, degrees, sin, cos, inf
import time
from lazybot.helper.util import clamp, remap, lerp, norm_ang

class Parking(Node):
    """
    Parking & Initialization Node.
    
    Responsibilities:
    1. Handles the "Start" sequence: Determines orientation, checks for parking markers, 
       and calculates initial position offsets.
    2. Handles the "End" sequence: Manages the final parking maneuver after the run.
    3. Communicates with an ESP32 (via /esp_cmd) for specific pre-programmed maneuvers 
       (like precise turns for parking moves).
    """
    def __init__(self):
        super().__init__('parking')
        self.get_logger().info('Parking node has been started.')
        
        # --- Communication with Low-Level Controller (ESP32) ---
        self.esp_cmd_pub = self.create_publisher(Int8, '/esp_cmd', 10)
        
        # --- High-Level Command Interface ---
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 1)
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        
        # --- Sensors ---
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)
        self.obj_sub = self.create_subscription(String, '/closest_obj', self.obj_callback, 1)
        self.debug_sub = self.create_subscription(BotDebugInfo, '/lazybot/debug', self.debug_callback, 10)
        
        # --- Outputs ---
        # Publishes calculated start offset to correct odometry
        self.offs_pub = self.create_publisher(Vector3, '/initial_offset', 10)
        # Publishes LiDAR distances for the ESP32 to use during final parking
        self.dir_dst_pub = self.create_publisher(Int16MultiArray, '/dir_dst', 1)
        
        # --- State Variables ---
        self.obj = "N" # Closest object color
        self.objs = []
        
        # LiDAR properties
        self.angMin = -pi
        self.angMax = pi
        self.angInc = radians(0.5)
        self.ranges = []
        self.ints = []
        
        self.pos = Vector3()
        
        # State Machine
        self.state = "Idle"
        self.park_dir = "_" # "L" or "R" (Direction of the parking lot relative to start)
        self.parking_offs_rng = [0.95, 1.45]
        self.parking_offs = self.parking_offs_rng[0]
        
        # Run logic in a separate thread to avoid blocking ROS callbacks
        self.parking_thread = Thread(target=self.parking_loop)
        self.parking_thread.daemon = True
        self.parking_thread.start()
    
    def parking_loop(self):
        """
        Main State Machine Loop.
        Handles the sequence of events from Boot -> Orientation -> Marker Check -> Run Start -> Run End -> Park.
        """
        while rclpy.ok():
            
            # --- State: Determine Orientation ---
            # Robot starts facing a wall. It needs to turn 90 deg to face the track.
            # It checks which side has more space to decide turn direction.
            if(self.state == "toTurn"):
                l = self.get_dst(80)
                r = self.get_dst(-80)
                
                if(l > r):
                    self.park_dir = "R"   # Open space is Left, so Parking Lot is on the Right
                    self.cmd_pub.publish(String(data="ccw")) # Set run direction
                else:
                    self.park_dir = "L"  # Open space is Right, so Parking Lot is on the Left
                    self.cmd_pub.publish(String(data="cw"))
                
                # Send command to ESP32 to execute the 90-degree turn
                if(self.park_dir == "L"):
                    self.send_c(1) # Command 1: Turn Right (to face track)
                    self.state = "TurningRight"
                else:
                    self.send_c(2) # Command 2: Turn Left (to face track)
                    self.state = "TurningLeft"
            
            # --- State: Check Parking Marker ---
            # After turning, robot faces the track. It checks the marker color 
            # to decide which parking spot to use later.
            elif(self.state == "Turned"):
                time.sleep(0.5)
                clr = "N"
                # Check detected objects from camera
                if(len(self.objs) > 0):
                    clr = self.objs[0]["color"]
                    # Ignore objects that are too far away (likely noise or next section)
                    if(self.objs[0]["dst"] > 0.5):
                        clr = "N"
                    self.obj = clr
                
                # Decide parking strategy based on marker color and direction
                self.set_state_table(self.obj)
            
            # --- State: Calculate Initial Offset ---
            # Uses LiDAR to measure distance to walls to calibrate starting position (x, y).
            elif(self.state == "Offset_Calc"):
                time.sleep(1.0)
                l = self.get_dst(90)
                r = self.get_dst(-90)
                f = self.get_dst(0)
                offx = 0.0
                offy = 0.0

                # Calculate X offset (lateral position in lane)
                if(self.park_dir == "R"):
                    offx = r - 0.5 # Distance from right wall
                else:
                    offx = 0.5 - l # Distance from left wall
                
                # Calculate Y offset (longitudinal position relative to start wall)
                offy = 1.5 - f

                self.state = "Idle"          
                self.send_c(5) # Command 5: Signal ESP32 that setup is done
                self.cmd_pub.publish(String(data="start")) # Start the main control node
                
                # Publish offset to reset odometry
                self.initial_offset = Vector3()
                self.initial_offset.x = offx
                self.initial_offset.y = offy
                self.initial_offset.z = 0.0
                self.offs_pub.publish(self.initial_offset)
                self.get_logger().info(f"Initial Offset set to: x={offx:.2f}, y={offy:.2f}, f={f:.2f}")
            

            # --- State: Final Parking ---
            # Triggered when the run is complete.
            elif(self.state == "RunEnd"):
                if(self.park_dir == "R"):
                    self.send_c(6) # Command 6: Park into Right spot
                else:
                    self.send_c(8) # Command 8: Park into Left spot
                self.state = "ParkingInit"
            
            time.sleep(0.1)
    
    def set_state_table(self, obj):
        """
        Logic Table for Parking Strategy.
        Combines Parking Direction (L/R) and Marker Color (R/G/None) 
        to send the correct parking command to ESP32.
        """

        if(obj == "G" and self.park_dir == "L"):
            self.send_c(3)  # Just Move Out
            self.state = "Moving"
        elif(obj == "G" and self.park_dir == "R"):
            self.send_c(4)  # Cross Object First
            self.state = "Moving"
        elif(obj == "R" and self.park_dir == "L"):
            self.send_c(4)  # Cross Object First
            self.state = "Moving"
        elif(obj == "R" and self.park_dir == "R"):
            self.send_c(3)  # Just Move Out
            self.state = "Moving"
        elif(obj == "N"):
            self.send_c(3)  # Default behavior
            self.state = "Moving"
    
    def send_c(self, c:int):
        """Helper to send integer commands to ESP32."""
        msg = Int8()
        msg.data = c
        self.esp_cmd_pub.publish(msg)
        
    def cmd_callback(self, msg: String):
        """Handles state transitions triggered by external nodes or ESP32 feedback."""
        self.get_logger().info(f'Received command: {msg.data}')
        
        if(msg.data == "Boot"):
            self.state = "toTurn" # Start initialization sequence
        
        if(msg.data == "Turned"):
            self.state = "Turned" # ESP32 finished turning
            
        if(msg.data == "Done"):
            self.state = "Offset_Calc" # ESP32 finished parking setup move
            
        if(msg.data == "RunEnd"):
            self.state = "RunEnd" # Main control node finished the run
    
    def lidar_callback(self, msg: LaserScan):
        """
        Updates LiDAR data.
        If in 'ParkingInit' state (final parking), it publishes distance data 
        to the ESP32 to help it center in the parking box.
        """
        self.angMin = msg.angle_min
        self.angMax = msg.angle_max
        self.angInc = msg.angle_increment
        self.ranges = msg.ranges
        self.ints = msg.intensities

        if(self.state == "ParkingInit"):
            # Get distances at key angles for parking alignment
            l = self.get_dst(90)
            fl = self.get_dst(45)
            f = self.get_dst(0)
            fr = self.get_dst(-45)
            r = self.get_dst(-90)

            # Send as integer array (cm) to ESP32
            msg = Int16MultiArray()
            msg.data = [int(l*100), int(fl*100), int(f*100), int(fr*100), int(r*100)]
            self.dir_dst_pub.publish(msg)
    
    def debug_callback(self, msg: BotDebugInfo):
        """Updates detected towers from the debug topic."""
        self.objs = []
        for tower in msg.towers:
            self.objs.append({
                "color": tower.color,
                "index": tower.index,
                "ang": tower.ang,
                "dst": tower.dst,
                "x": tower.x,
                "y": tower.y
            })
    
    def pos_callback(self, msg: Vector3):
        self.pos = msg
        
    def obj_callback(self, msg: String):
        self.obj = msg.data
    
    # --- LiDAR Helper Functions ---

    def get_dst(self, ang):
        """Returns distance at a specific angle (degrees)."""
        i = self.a2i(radians(ang))
        return self.fix_missing(i)
    
    def fix_missing(self, i):
        """Handles invalid LiDAR readings (inf or 0)."""
        if(i < 0 or i >= len(self.ints)):
            return 0
        # Check intensity and range validity
        if(self.ints[i] > 0.05 and self.ranges[i] <= 3.0 and self.ranges[i] != inf):
            return self.ranges[i]
        
        if(self.ranges[i] == inf):
            return 0
        return self.ranges[i]

    def i2a(self, i, deg = False):
        """Index to Angle."""
        if deg:
            return degrees(self.angMin + self.angInc*i)
        return self.angMin + self.angInc*i
    
    def indRng(self, Min, Max):
        """Angle range to Index range."""
        return [self.a2i(Min), self.a2i(Max)]

    def a2i(self, ang):
        """Angle to Index."""
        return round((ang -self.angMin) / self.angInc)

def main(args=None):
    rclpy.init(args=args)
    node = Parking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()