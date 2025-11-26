import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

from std_msgs.msg import Float32, String
from math import pi, radians, degrees, sqrt, inf
import time

class OpenNode(Node):
    """
    Control Node for the Open Challenge.
    
    Strategy:
    1. Drive straight maintaining a specific heading (0, 90, 180, 270 degrees).
    2. Use LiDAR to detect when a wall is directly in front.
    3. When a wall is detected, determine turn direction (Left vs Right) based on available space.
    4. Update target heading by +/- 90 degrees and execute turn.
    5. Count laps based on starting position.
    """
    def __init__(self):
        super().__init__('open_control')

        # --- Parameters ---
        self.declare_parameter('IS_SIM', False)
        self.IS_SIM = self.get_parameter('IS_SIM').get_parameter_value().bool_value
        
        # --- Publishers & Subscribers ---
        # Command interface
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        
        # Sensors
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)

        # Actuators
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 1)
        self.steer_pub = self.create_publisher(Float32, '/steer', 1)
        
        # --- Timers ---
        self.create_timer(0.025, self.odom_loop)    # 40Hz: Lap counting
        self.create_timer(0.025, self.control_loop) # 40Hz: Navigation logic

        # --- State Variables ---
        self.dir = 0 # 0: Unknown, 1: CCW (Left turns), -1: CW (Right turns)
        
        # LiDAR properties (updated in callback)
        self.angMin = -pi
        self.angMax = pi
        self.angInc = radians(0.5)
        self.ranges = []
        self.ints = []

        # Navigation Params
        self.maxSpeed : float = 0.8
        self.speed : float = 0.0
        self.strAngle : float = 0.0
        self.strRange = 1.0 # Max steering value
        
        self.front_dist_thresh = 0.7 # Distance to wall to trigger a turn

        self.new_lidar_val = False

        # Odometry & Mission State
        self.pos = Vector3()
        self.sectionAngle = 0.0 # Current target heading (aligned to track axes)
        self.turning = False
        self.gotWallD = False
        self.reached = True
        self.lapCount = 0
        self.targetLap = 3
        self.running = False
        self.endOffset = [0.0, 0.0] # Y-coordinates of start/finish box

        self.get_logger().info(f'Open Control node has been started - with IS_SIM={self.IS_SIM}')
        self.lastTime = time.time()

        # Debounce variable to prevent double-triggering turns
        self.lastTurnSpot = Vector3(x=100.0, y=100.0, z=0.0)
    
    def control_loop(self):
        """
        Main Navigation Loop.
        Calculates steering based on heading error and wall distances.
        """
        if not self.running:
            self.pubDrive(disable=True)
            return
        
        # 1. Heading Correction (P-Controller)
        # Calculate error between current yaw and target section angle
        err = self.sectionAngle - self.pos.z
        
        # If error is small, we are done turning
        if(abs(err) < radians(20)):
            self.turning = False
        

        turningKP = 1.0
        # Calculate base steering to align with target heading
        sA = self.remap(err, -radians(90), radians(90), -self.strRange/turningKP, self.strRange/turningKP)
        
        # Aggressive steering multiplier during turns
        if(self.turning):
            sA *= 20
        self.strAngle = self.clamp(sA, -self.strRange, self.strRange)
        
        # 2. Wall Detection
        frontDist = self.get_dst(0)
        leftDist = self.get_dst(90)
        rightDist = self.get_dst(-90)

        if(leftDist == inf):
            leftDist = 10
        if(rightDist == inf):
            rightDist = 10

        # Distance from last turn to prevent immediate re-triggering
        checkPointDist =  sqrt((self.pos.x - self.lastTurnSpot.x)**2 + (self.pos.y - self.lastTurnSpot.y)**2)

        # 3. Corner Logic
        # If wall is close in front AND we haven't just turned
        if(frontDist < self.front_dist_thresh and not self.turning and checkPointDist > 1.0):
            self.get_logger().info(f"Front: {frontDist:.2f}, Left: {leftDist:.2f}, Right: {rightDist:.2f}")
            
            # Determine direction on first turn (Auto-detect CW vs CCW)
            if(self.dir == 0):                
                if(leftDist > rightDist):
                    self.dir = 1 # More space on left -> CCW
                    self.get_logger().info("Setting direction to CCW.")
                else:
                    self.dir = -1 # More space on right -> CW
                    self.get_logger().info("Setting direction to CW.")
                
            # Update target heading for the next section
            if(self.dir == 1):
                self.sectionAngle = self.sectionAngle + pi/2 # Turn Left (+90 deg)
            else:
                self.sectionAngle = self.sectionAngle - pi/2 # Turn Right (-90 deg)
            
            self.turning = True
            self.get_logger().info(f"Section angle changed: {degrees(self.sectionAngle)}")
            
            # Record turn location
            self.lastTurnSpot.x = self.pos.x
            self.lastTurnSpot.y = self.pos.y
            
        # 4. Wall Centering (when driving straight)
        # If in a corridor (walls on both sides), center the robot
        elif(leftDist + rightDist < 1.1):            
            pos = leftDist - rightDist
            # Add small steering adjustment to stay in middle
            kP2 = 15
            self.strAngle = self.clamp(
                val = self.remap(pos, -0.5, 0.5, -self.strRange/kP2, self.strRange/kP2), 
                mini = -self.strRange/2,
                maxi = self.strRange/2
            )
        
        # 5. Speed Control
        # Slow down when steering angle is high (cornering)
        self.speed = self.remap(
            old_val = abs(self.strAngle), 
            old_min = 0, 
            old_max = self.strRange,
            new_min = self.maxSpeed,
            new_max = self.maxSpeed * 0.5
        )
        
        self.pubDrive()
        
    def odom_loop(self):
        """
        Lap Counting Logic.
        Uses the initial wall distance to define a start/finish box.
        """
        if not self.running or not self.gotWallD:
            self.pubDrive(disable=True)
            return
        
        # Check if robot is inside the start/finish box
        isInside = abs(self.pos.x) < 0.75 and self.pos.y > self.endOffset[0] and self.pos.y < self.endOffset[1]
        
        # Logic to detect entering the box after leaving it
        if not self.reached:
            if isInside:
                self.reached = True
                self.lapCount += 1
                self.get_logger().info(f"Lap {self.lapCount} completed!")
        
        # Logic to detect leaving the box
        if not isInside and self.reached:
            self.get_logger().info("Moving on...")
            self.reached = False
        
        # Stop after target laps
        if self.lapCount >= self.targetLap:            
            self.running = False
            self.get_logger().info("Reached the destination, stopping the robot.")
            self.pubDrive(disable=True)
            return
        
    def pos_callback(self, msg: Vector3):
        """Updates internal position state."""
        self.pos.x = msg.x
        self.pos.y = msg.y
        self.pos.z = msg.z
        
    def cmd_callback(self, msg: String):
        """Handles external commands."""
        command = msg.data.strip().lower()
        if command == "boot":
            self.lapCount = 0
            self.dir = 0
            self.turning = False
            self.sectionAngle = 0.0
            self.reached = True
            self.running = True
            self.gotWallD = False
            self.get_logger().info("Starting the robot.")
            self.cmd_pub.publish(String(data="start_open"))
        elif command == "stop":
            self.running = False
            self.get_logger().info("Stopping the robot.")
            self.dir = 0
    
    def lidar_callback(self, msg: LaserScan):
        """Updates LiDAR data and initializes start/finish box."""
        self.angMin = msg.angle_min
        self.angMax = msg.angle_max
        self.angInc = msg.angle_increment

        self.ranges = msg.ranges
        self.ints = msg.intensities

        # print(msg.ranges) # Debug print
        
        if not self.running:
            return
        
        # Initialize Start/Finish Box based on rear wall distance
        if(not self.gotWallD):
            wi = self.a2i(0.0) # Index for 0 degrees (Front)
            
            if(self.IS_SIM): self.ints[wi] = 1.0
            
            if(self.ints[wi] <= 0.1 or self.ranges[wi] > 3.0):
                self.ranges[wi] = self.fix_missing(wi)
                self.ints[wi] = 1.0
            
            # Define box relative to detected wall
            self.endOffset[0] = self.ranges[wi] - 1.5
            self.endOffset[1] = self.ranges[wi] - 0.5
            
            self.get_logger().info(f"Wall Distance: {self.ranges[wi]:.2f}, End Offset: {self.endOffset}")
            
            self.gotWallD = True
            self.sectionAngle = self.pos.z # Lock initial heading
            
            # Check if we are starting inside the valid area
            isInside = abs(self.pos.x) < 0.75 and self.pos.y > self.endOffset[0] and self.pos.y < self.endOffset[1]
            
            if not isInside:
                self.lapCount = -1
                self.get_logger().info(f"Robot is outside the area, setting lap count to {self.lapCount}.")

    
    def fix_missing(self, i):
        """Interpolates missing LiDAR rays."""
        if(i < 0 or i >= len(self.ints)):
            return 0
        if( self.IS_SIM or self.ints[i] > 0.05 and self.ranges[i] <= 3.0 ):
            return self.ranges[i]
        
        # Find nearest valid neighbors
        first = i
        last = i
        while(first > 0 and (self.ints[first] == 0.0 or self.ranges[first] > 3.0)):
            first -= 1
            if(first < 0):
                first = 0
                break
        while(last < len(self.ints) and (self.ints[last] == 0.0 or self.ranges[last] > 3.0)):
            last += 1
            if(last >= len(self.ints)):
                last = len(self.ints) - 1
                break
        
        if(self.ints[first] == 0.0 or self.ints[last] == 0.0):
            return 0
        if(first == last):
            return self.ranges[first]
        
        return self.lerp(self.ranges[first], self.ranges[last], (i-first)/(last-first))

    def pubDrive(self, disable = False):
        """Publishes motor commands."""
        if disable:
            self.speed = 0.0
            self.strAngle = 0.0
        throttle_msg = Float32()
        throttle_msg.data = self.speed
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()
        steer_msg.data = self.strAngle
        self.steer_pub.publish(steer_msg)
        
    def get_dst(self, ang):
        """Gets distance at specific angle (degrees)."""
        i = self.a2i(radians(ang))
        if(self.ints[i] <= 0.05 or self.ranges[i] > 3.0):
            self.ranges[i] = self.fix_missing(i)
            self.ints[i] = 1.0
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

    def clamp(self, val, mini, maxi):
        """Constrains value."""
        tMin = mini
        tMax = maxi
        if (mini > maxi):
            tMin = maxi
            tMax = mini
        if (val < tMin):
            return tMin
        if (val > tMax):
            return tMax
        return val
            
    def remap(self, old_val, old_min, old_max, new_min, new_max):
        """Maps value from one range to another."""
        newVal = (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min
        return self.clamp(newVal, new_min, new_max)

    def lerp(self, a, b, t):
        """Linear Interpolation."""
        return self.clamp(a + (b - a) * t, a, b)
    
    def norm_ang(self, a):
        """Normalizes angle to [-pi, pi]."""
        return (a + pi) % (2 * pi) - pi
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = OpenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()