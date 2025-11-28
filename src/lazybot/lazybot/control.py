import rclpy
from threading import Thread
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

from std_msgs.msg import Float32, Float32MultiArray, String, Int8MultiArray, Int8
from lazy_interface.msg import BotDebugInfo, LidarTowerInfo
from math import pi, radians, degrees, sin, cos
import time

from lazybot.helper.util import LidarHandler, clamp, remap, lerp, norm_ang

class ControlNode(Node):
    """
    Main Control Node for LazyBot.
    Handles sensor fusion (LiDAR, Odometry, Camera), path planning (Disparity Extender),
    obstacle avoidance, and motor control.
    """
    def __init__(self):
    
        super().__init__('control')

        # --- Parameters ---
        self.declare_parameter('IS_SIM', False)
        self.IS_SIM = self.get_parameter('IS_SIM').get_parameter_value().bool_value

        # --- Publishers & Subscribers ---
        
        # Command interface (Start/Stop/Direction)
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        
        # Sensor inputs
        self.sensor_sub = self.create_subscription(Int8MultiArray, '/lazybot/sensors', self.sensor_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.obj_sub = self.create_subscription(String, '/closest_obj', self.obj_callback, 1) # From Camera
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)   # From Odometry

        # Data outputs
        self.objPub = self.create_publisher(Float32MultiArray, '/obj_data', 1)
        
        # Actuator outputs
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 1)
        self.steer_pub = self.create_publisher(Float32, '/steer', 1)
        self.cam_pub = self.create_publisher(Int8, '/cam_servo', 1) # Servo to point camera at towers

        # Debugging
        self.pubDebug = self.create_publisher(BotDebugInfo, '/lazybot/debug', 10)
        
        # --- Timers ---
        self.create_timer(0.025, self.odom_loop)    # 40Hz: Position & Lap Logic
        self.create_timer(0.025, self.control_loop) # 40Hz: Motor Command Publishing

        # --- State Variables ---
        self.IS_OPEN = False # False = Obstacle Challenge, True = Open Challenge
        self.dir = 1 # 1 = CCW (Counter-Clockwise), -1 = CW (Clockwise)
        
        self.lidar = LidarHandler() # Helper class to manage LiDAR arrays

        # --- Navigation Parameters ---
        
        # Speed Boosting (Go fast on straights)
        self.speedBoost = 1.0
        self.boostMax = 1.35 #1.25
        self.boostAngleThresh = 7.5 # Only boost if steering angle is within +/- 7.5 deg
        self.boostDistThresh = 1.10 # Only boost if clear path > 1.1m

        # Speed Limits
        self.maxSpeed : float = 0.60
        self.speedCap : float = 0.40
        self.targetCap : float = 0.40
        self.speedCapRng = [0.30, 0.45] # [Corner Speed, Straight Speed]
        self.speed : float = 0.0
        
        # Steering Parameters
        self.strAngle : float = 0.0
        self.strRange = 1.0 # Max steering value (normalized -1 to 1)
        self.str_ang_thresh = 60.0 # Max physical steering angle in degrees
        self.strSpd = 0.65 # Steering smoothing factor
        
        # LiDAR Processing
        self.prec = 81 # Precision window for marching algorithm
        self.skip1 = 2 # Skip factor for initial scan
        self.skip2 = 1 # Skip factor for fine checks

        # Safety
        self.dangerDist = 0.22 # Emergency avoidance distance
        self.dangerAng = [25.0, 90.0] # Angles to check for side collisions

        # Disparity Extender / Gap Follower Params
        self.new_lidar_val = False
        self.castRange = [0.13, 0.16] # Dynamic robot width (expands with speed)
        self.castR = 0.25 # Current cast radius
        self.lookRng = radians(80.0) # Field of view for path planning
        self.lookRngS = radians(130.0)
        self.targetAng = 0.0
        self.targetD = 0.0
        
        # --- Background Thread ---
        # Runs heavy LiDAR calculations separately from ROS callbacks
        self.calc_thr = Thread(target=self.calc_lidar)
        self.calc_thr.daemon = True
        self.calc_thr.start()

        # --- Odometry & Lap Counting ---
        self.pos = Vector3()
        self.sectionAngle = 0.0
        self.gotWallD = False
        self.reached = True
        self.lapCount = 0
        self.targetLap = 2
        self.running = False
        self.endOffset = [-0.5, 1.5] # Y-coordinates for Start/Finish area
        # Coordinates of track corners for special handling
        self.cornerPOS = [(0.0, 1.0), (0.0, -1.0), (2.0, 1.0), (2.0, -1.0), (-2.0, 1.0), (-2.0, -1.0)]

        self.objs = [] # Detected towers
        self.cont_stack = [] # Stack for edge detection
        
        self.closest = "N" # Closest object color: "R"ed, "G"reen, "N"one

        self.get_logger().info('Control node has been started.')
        self.lastTime = time.time()

    
    def sensor_callback(self, msg: Int8MultiArray):
        """Handles hardware limit switches or line sensors."""
        if len(msg.data) < 4:
            self.get_logger().warn("Received sensor data with insufficient length.")
            return
        
        # Hard steering override if sensors trigger
        if msg.data[0] == 1:
            self.strAngle = -self.strRange
        if msg.data[1] == 1:
            self.strAngle = self.strRange

    def control_loop(self):
        """Publishes motor commands at a fixed rate."""
        if not self.running:
            return
        self.pubDrive()
        
    def odom_loop(self):
        """
        Handles Lap Counting and Mission Status.
        Checks if robot is inside the start/finish box to increment laps.
        Stops robot after target laps are completed.
        """
        if not self.running or not self.gotWallD:
            self.pubDrive(disable=True)
            return
        
        # Check if robot is inside the Start/Finish area
        isInside = abs(self.pos.x) < 0.6 and self.pos.y > self.endOffset[0] and self.pos.y < self.endOffset[1]
        
        if not self.reached:
            if isInside:
                self.reached = True
                self.lapCount += 1
                self.get_logger().info(f"Lap {self.lapCount} completed!")
        
        if not isInside and self.reached:
            self.get_logger().info("Moving on...")
            self.reached = False
        
        # Mission Complete Logic
        if self.lapCount >= self.targetLap:
            self.speedCap = 0.35 # Slow down for parking
            
            # Simple parking logic: Stop when Y > 0.07 inside the box
            if(self.pos.y > 0.07):
                self.running = False
                self.get_logger().info("Reached the destination, stopping the robot.")
                self.pubDrive(disable=True)
                self.cmd_pub.publish(String(data="NormalizeIMU"))
                # time.sleep(5.0)
                self.cmd_pub.publish(String(data="RunEnd"))
                return
        
        # Section tracking (simplified IMU/Odometry heading check)
        if abs(self.pos.z - self.sectionAngle) > radians(80):
            if(self.pos.z > self.sectionAngle):
                self.sectionAngle = self.sectionAngle + pi/2
            else:
                self.sectionAngle = self.sectionAngle - pi/2
            self.get_logger().info(f"Section angle changed: {degrees(self.sectionAngle)}")
    
    def cmd_callback(self, msg: String):
        """Handles external commands (e.g., from UI or Bluetooth)."""
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {command}")
        if command == "start":
            self.lapCount = 0
            self.reached = True
            self.running = True
            self.gotWallD = False
            self.get_logger().info("Starting the robot.")
        elif command == "stop":
            self.running = False
            self.get_logger().info("Stopping the robot.")
        elif command == "cw":
            self.dir = -1
            self.get_logger().info("Setting direction to clockwise.")
        elif command == "ccw":
            self.dir = 1
            self.get_logger().info("Setting direction to counter-clockwise.")
            
    def obj_callback(self, msg: String):
        """Receives color of the closest object from the camera node."""
        self.closest = msg.data
        
    def lidar_callback(self, msg: LaserScan):
        """Updates LiDAR data and handles initial startup checks."""
        self.new_lidar_val = True
        self.lidar.set_laser_vals(msg)
        
        if not self.running:
            return
        
        # Wait for valid wall distance before starting logic
        if(not self.gotWallD):
            wi = self.lidar.a2i(0.0) # Index at 0 degrees
            if(self.IS_SIM): self.lidar.ints[wi] = 1.0
            
            if(self.lidar.ints[wi] <= 0.1 or self.lidar.ranges[wi] > 3.0):
                self.lidar.ranges[wi] = self.lidar.fix_missing(wi)
                self.lidar.ints[wi] = 1.0
            
            self.get_logger().info(f"Wall Distance: {self.lidar.ranges[wi]:.2f}, End Offset: {self.endOffset}")
            self.gotWallD = True
            
            # Check if robot started outside the box
            isInside = abs(self.pos.x) < 0.75 and self.pos.y > self.endOffset[0] and self.pos.y < self.endOffset[1]
            if not isInside and self.pos.y < 0.5:
                self.lapCount = -1
                self.get_logger().info(f"Robot is outside the area, setting lap count to {self.lapCount}.")

    def calc_lidar(self):
        """
        Main Navigation Algorithm (Background Thread).
        1. Calculates optimal heading based on furthest available path (Gap Follower).
        2. Detects towers and adjusts heading to avoid them.
        3. Handles cornering logic.
        4. Calculates speed based on path straightness.
        """
        while True:
            if(not self.new_lidar_val):
                time.sleep(0.01)
                continue

            # Calculate delta time for smoothing
            self.dt = time.time() - self.lastTime
            self.lastTime = time.time()

            # Dynamic Cast Radius: Robot "width" increases with speed
            self.castR = remap(self.speed/self.maxSpeed, 0.45, 1, self.castRange[0], self.castRange[1])
            
            self.fix_all_missing()
            self.objs = []
            self.cont_stack = []
            self.findOBJs()

            # Sort detected objects by distance
            self.objs.sort(key=lambda x: x["dst"], reverse=False)

#             indx = 0
#             for obj in self.objs:
#                 indx += 1
#                 self.get_logger().info(f"""
# {indx} - {obj['dst']} - {self.closest}""")

            self.pubObjData()

            # --- Core Path Planning ---
            # Find the direction with the maximum distance (deepest gap)
            maxD, tA = self.getMaxDOBJ()
            

            # Point Camera Servo at the nearest object within 70 degrees
            for obj in self.objs:
                if(abs(obj['ang']) < radians(70)):
                    msg = Int8()
                    msg.data = int(degrees(obj['ang']))
                    self.cam_pub.publish(msg)
                    break
            
            # Smooth the target angle (Low-pass filter)
            delta = abs(tA-self.targetAng)
            self.targetAng = tA if(delta > 0.5) else lerp(self.targetAng, tA, 35*self.dt)
            self.targetAng = lerp(self.targetAng, tA, 0.1)
            self.targetD = maxD
            
            # --- Corner Handling ---
            corner = False
            if(self.isInCorner(0.75)):
                corner = self.corner_handling(self.objs[0] if self.objs else None)

            # --- Speed Profiling ---
            # Slow down in corners, speed up in straights
            self.targetCap = self.speedCapRng[1]
            if(self.isInCorner(0.15)):
                self.targetCap = self.speedCapRng[0]
            
            # Smooth speed transitions
            if(self.targetCap < self.speedCap):
                self.speedCap = self.targetCap # Decelerate instantly
            else:
                self.speedCap = lerp(self.speedCap, self.targetCap, min(self.dt*5, 1.0)) # Accelerate smoothly

            self.targetAng = degrees(self.targetAng)
            
            # Override heading if in corner (and not in Open Challenge mode)
            if(corner and not self.IS_OPEN):
                self.targetAng = corner

            # Emergency side collision avoidance
            self.targetAng = self.dangerSense(self.targetAng)

            # --- Speed Boosting ---
            self.speedBoost = 1.0
            # Check if the path directly ahead is clear using ray marching
            marchingHit = self.marching(self.lidar.a2i(self.targetAng, degrees=True), R=self.castR/2)
            hitD = marchingHit['dst']
            
            # If steering is straight and path is long, enable boost
            if(abs(self.targetAng)  < self.boostAngleThresh and hitD > self.boostDistThresh):
                self.speedBoost = self.boostMax
                # self.get_logger().info("Boosting!")
            
            # Map target angle to steering servo range (-1 to 1)
            sAng = remap(self.targetAng, -self.str_ang_thresh, self.str_ang_thresh, -self.strRange, self.strRange)
            self.strAngle = lerp(self.strAngle, sAng, min(self.dt*5, 1.0)) if self.IS_SIM else sAng
            
            # Calculate final speed based on distance to wall
            mult2 = remap(maxD, 1.0, 2.0, 0.65, 1.0)
            self.speed = self.maxSpeed * mult2 * self.speedBoost
            
            self.pubDebugPoint()
            self.new_lidar_val = False

    def fix_all_missing(self):
        chkRng = self.lidar.indRng(-self.lookRng, self.lookRng)
        for i in range(chkRng[0]-50, chkRng[1]+50):
            self.lidar.ranges[i] = self.lidar.fix_missing(i)
    
    def corner_handling(self, obj):
        """
        Special logic for corners.
        If an object (tower) is detected, orbit around it.
        If no object, turn 90 degrees based on direction.
        """
        if(obj is None):
            # If wall is close, force a 90 degree turn
            if(self.lidar.get_dst(0) < 0.75):
                return self.dir*90.0
        else:
            # Orbit logic: Calculate tangent angle to the tower
            # ang = self.castR/obj['dst'] * (1.0 if(self.closest == "G") else -1.0)
            ang = self.castR/obj['dst']
            if(self.closest == "G"):
                return degrees(obj['ang'] + ang)
            elif(self.closest == "R"):
                return degrees(obj['ang'] - ang)
            # return degrees(obj['ang'] + ang)
        
        return False

    def isInCorner(self, bleeding = 0.0):
        """Checks if robot position matches known corner coordinates."""
        for corner in self.cornerPOS:
            isIn = abs(self.pos.x - corner[0]) < 0.50 + bleeding and abs(self.pos.y - corner[1]) < 0.50 + bleeding
            if isIn:
                return True
        return False
    
    def pos_callback(self, msg: Vector3):
        """Updates internal position state."""
        self.pos.x = msg.x
        self.pos.y = msg.y
        self.pos.z = msg.z
    
    def getMaxDOBJ(self):
        """
        Scans the LiDAR array to find the 'deepest' path (furthest distance).
        Also detects towers and filters them based on color (Red/Green).
        """
        _max = {"dst": 0, "ang": 0}
        
        # chkRng = self.lidar.indRng(-self.lookRng, self.lookRng)

        # self.objs = []
        # self.cont_stack = []

        # # Determine which side to ignore based on detected color
        # remove = "None" 
        # if self.closest == "G":
        #     remove = "Right" # Ignore Right if Green (Pass Left)
        # elif self.closest == "R":
        #     remove = "Left"  # Ignore Left if Red (Pass Right)

        chkRng = self.lidar.indRng(-self.lookRng, self.lookRng)

        remove = "None" 
        if self.closest == "G":
            remove = "Right"
            chkRng[0] = self.objs[0]["index"] if len(self.objs) > 0 else chkRng[0]
        elif self.closest == "R":
            remove = "Left"
            chkRng[1] = self.objs[0]["index"] if len(self.objs) > 0 else chkRng[1]
        
        for i in range(chkRng[0], chkRng[1], self.skip1):
            # Simulation hack: force intensity high
            if(self.IS_SIM): self.lidar.ints[i] = 1.0
            
            # Filter bad data
            if(self.lidar.ints[i] <= 0.1 or self.lidar.ranges[i] > 3.0):
                self.lidar.ranges[i] = self.lidar.fix_missing(i)
                self.lidar.ints[i] = 1.0

            if(i < 1 or i >= len(self.lidar.ranges) or self.lidar.ints[i] <= 0.05 or self.lidar.ranges[i] > 3.0):
                continue
            
            # Calculate effective distance accounting for robot width (Marching)
            dt = self.marching(i)
            
            if(dt["dst"] > _max["dst"]):
                _max = dt
                
            # Detect towers/obstacles
            # objectFound = self.detectContrast(i)
            
            # # Logic to ignore the "best path" if it leads to a forbidden side of a tower
            # if objectFound:
            #     if remove == "Right":
            #         _max = dt # Force path to current (likely left)
            #     if remove == "Left":
            #         return _max["dst"], _max["ang"] # Return immediately (likely right)
        return _max["dst"], _max["ang"]

    def findOBJs(self):
        chkRng = self.lidar.indRng(-self.lookRng, self.lookRng)
        for i in range(chkRng[0], chkRng[1]):
            self.detectContrast(i)


    def detectContrast(self, i):
        """
        Detects objects based on sudden changes (edges) in LiDAR range.
        Uses a stack to track falling edges and rising edges.
        """
        if(self.IS_OPEN):
            return False
        
        slope = (self.lidar.ranges[i] - self.lidar.ranges[i-self.skip1])
        
        # Falling edge (Object starts)
        if(slope < -0.25):          # it was -0.15
            self.cont_stack.append(i)
        # Rising edge (Object ends)
        elif(slope > 0.25):         # it was 0.15
            if(len(self.cont_stack) > 0):
                pop = self.cont_stack.pop()
                mid = (i + pop) // 2
                # Calculate physical width of the object
                sz = self.lidar.ranges[mid] * abs(i-pop)*self.lidar.angInc
                
                # Filter by size (approx. tower width)
                if( sz > 0.02 and sz < 0.1):
                    ang = self.lidar.i2a(mid)
                    obj = {
                        "index": mid,
                        "ang": ang,
                        "dst": self.lidar.ranges[mid],
                        "x": self.lidar.ranges[mid] * cos(ang),
                        "y": self.lidar.ranges[mid] * sin(ang),
                        "intensity": self.lidar.ints[mid]
                    }
                    self.objs.append(obj)
                    return obj
        return False

    def marching(self, indx, R = None):
        """
        'Circle Casting' / Ray Marching check.
        Checks if a circle of radius R (robot width) centered on the ray 
        at 'indx' would collide with any neighboring points.
        Returns the distance to the first collision, or the ray distance if clear.
        """
        if R is None:
            R = self.castR

        rng = [indx-self.prec*self.skip2, indx+self.prec*self.skip2]

        targetRay = {
            "dst": self.lidar.ranges[indx],
            "ang": self.lidar.i2a(indx)
        }
        
        _min = {"dst": 1000, "ang": 0}
        for i in range(rng[0], rng[1], self.skip2):
            if(self.IS_SIM): self.lidar.ints[i] = 1.0
            if(i >= 0 and i < len(self.lidar.ranges)):
                ray = {
                    "dst": self.lidar.ranges[i],
                    "ang": self.lidar.i2a(i)
                }

                # Check geometric intersection
                hit = self.hitC(targetRay, ray, R)
                if(hit):
                    if(hit < _min["dst"]):
                        _min = {
                            "dst": ray["dst"],
                            "ang": targetRay["ang"]
                        }
        if _min["dst"] >= 1000:
            return {"dst": targetRay["dst"], "ang": targetRay["ang"]}
        else:
            return {"dst": _min["dst"], "ang": _min["ang"]}

    def hitC(self, rayPnt, checkPnt, R):
        """Geometric check: Does a point fall within the cast radius of the target ray?"""
        dTheta = abs(rayPnt["ang"] - checkPnt["ang"])     
        if(checkPnt["dst"] <= 0.0):
            return False   
        collAng = R/checkPnt["dst"]
        if(dTheta <= collAng):
            return checkPnt["dst"]
        
        return False
    
    def clearance_ang(self, d):
        return self.castR/d
    
    def dangerSense(self, tAng):
        """
        Emergency override.
        If an object is detected very close (dangerDist) on the sides,
        steer away drastically.
        """
        found_obst = False
        obsAng = 0.0
        for i in range(len(self.lidar.ranges)):
            if(self.lidar.ints[i] <= 0.05 or self.lidar.ranges[i] > 3.0):
                continue
            if(self.lidar.ranges[i] < self.dangerDist):
                ang = degrees(self.lidar.i2a(i))
                # Check left and right danger zones
                if(ang > self.dangerAng[0] and ang < self.dangerAng[1]):
                    found_obst = True
                    obsAng = ang
                    break
                if(ang > -self.dangerAng[1] and ang < -self.dangerAng[0]):
                    found_obst = True
                    obsAng = ang
                    break
        if(found_obst):
            # If target angle is towards the obstacle, reduce it significantly
            if(tAng*obsAng > 0):
                return tAng/3
        return tAng

    def pubDrive(self, disable = False):
        """Publishes throttle and steering commands."""
        if disable:
            self.speed = 0.0
            self.strAngle = 0.0
        throttle_msg = Float32()
        throttle_msg.data = clamp(self.speed, -self.speedCap*self.speedBoost, self.speedCap*self.speedBoost)
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()
        steer_msg.data = self.strAngle
        self.steer_pub.publish(steer_msg)

    def pubObjData(self):
        """Publishes angles of detected objects."""
        obj_data = Float32MultiArray()
        obj_data.data = []
        for obj in self.objs:
            obj_data.data.append(obj["ang"])
        
        self.objPub.publish(obj_data)

    def pubDebugPoint(self):
        """Publishes visualization data for RVIZ."""
        msg = BotDebugInfo()
        msg.target_angle = self.targetAng
        msg.target_distance = self.targetD
        msg.cast_radius = self.castR
        msg.danger_distance = self.dangerDist
        msg.danger_angles = self.dangerAng
        
        msg.towers = []
        for obj in self.objs:
            tower = LidarTowerInfo()
            tower.color = self.closest
            tower.index = obj["index"]
            tower.ang = obj["ang"]
            tower.dst = obj["dst"]
            tower.x = obj["x"]
            tower.y = obj["y"]
            msg.towers.append(tower)
        self.pubDebug.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()