import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from math import pi, radians, degrees, sin, cos
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)

        self.throttle_pub = self.create_publisher(Float32, '/throttle', 1)
        self.steer_pub = self.create_publisher(Float32, '/steer', 1)

        self.pubDebug = self.create_publisher(PointCloud, "/target_point", 10)

        self.datass = []

        self.maxSpeed : float = 0.65
        self.speed : float = 0.0
        self.strAngle : float = 0.0
        self.strRange = 1.0
        self.str_ang_thresh = 90.0
        self.strSpd = 0.65
        
        self.prec = 61
        self.skip1 = 2
        self.skip2 = 1

        self.dangerDist = 0.3
        self.dangerAng = [25.0, 90.0]

        self.castRange = [0.25, 0.28]
        self.castR = 0.25
        self.lookRng = radians(90.0)
        self.lookRngS = radians(110.0)
        self.targetAng = 0.0
        self.targetD = 0.0

        self.get_logger().info('Control node has been started.')
        self.lastTime = time.time()

    def lidar_callback(self, msg: LaserScan):
        # self.get_logger().info(f'LIDAR data received: {degrees(msg.angle_min)} to {degrees(msg.angle_max)}, with {degrees(msg.angle_increment)} increment.')

        self.dt = time.time() - self.lastTime
        self.lastTime = time.time()

        angMin = msg.angle_min
        angMax = msg.angle_max
        angInc = msg.angle_increment

        angDats = msg.ranges
        angInts = msg.intensities
        self.castR = self.remap(self.speed/self.maxSpeed, 0.45, 1, self.castRange[0], self.castRange[1])

        maxD, tA = self.getMaxD(angMin, angInc, angDats, angInts)
        dS = self.dangerSense(angDats, angMin, angInc)

        # if(dS):
        #     tA -= dS * self.strRange * 1.0

        delta = abs(tA-self.targetAng)
        self.targetAng = tA if(delta > 0.5) else self.lerp(self.targetAng, tA, 35*self.dt)
        self.targetAng = self.lerp(self.targetAng, tA, 0.1)
        self.targetAng = degrees(self.targetAng)
        self.targetD = maxD

        self.get_logger().info(f"Target Angle: {self.targetAng}, Max Dst: {maxD}")
        self.strAngle = self.remap(self.targetAng, -self.str_ang_thresh, self.str_ang_thresh, -self.strRange, self.strRange)

        mult2 = self.remap(maxD, 1.0, 2.0, 0.65, 1.0)

        trgSpd = self.maxSpeed * mult2
        if(trgSpd < self.speed):
            self.speed = self.lerp(self.speed, trgSpd, 30*self.dt)
        else:
            self.speed = self.lerp(self.speed, trgSpd, 6*self.dt)
        self.datass = []
        self.pubDrive()
        try:
            self.pubDebugPoint()
        except Exception as e:
            self.get_logger().error(f"Error publishing debug point: {e}")

    def getMaxD(self, angMin, _inc, _dats, _ints):
        _max = {"dst": 0, "ang": 0}
        chkRng = self.indRng(-self.lookRng, self.lookRng, angMin, _inc)

        mid = (chkRng[0] + chkRng[1]) // 2
        for i in range(0, mid - chkRng[0], self.skip1):
            if(_ints[mid-i] <= 0.1 or _dats[mid-i] > 3.0):
                _dats[mid-i] = self.fix_missing(_dats, _ints, mid-i)
                _ints[mid-i] = 1.0  # Mark as fixed
            if(_ints[mid+i] <= 0.1 or _dats[mid+i] > 3.0):
                _dats[mid+i] = self.fix_missing(_dats, _ints, mid+i)
                _ints[mid+i] = 1.0  # Mark as fixed

            if(mid - i < 0 or mid + i >= len(_dats) or _ints[mid - i] == 0.0 or _ints[mid + i] == 0.0):
                continue
            dt = self.marching(mid - i, _dats, _ints, angMin,_inc)
            if(dt["dst"] > _max["dst"]):
                _max = dt
            
            dt = self.marching(mid + i, _dats, _ints, angMin,_inc)
            if(dt["dst"] > _max["dst"]):
                _max = dt
            # print(dt)
            if(mid+i < chkRng[1]-10 and _max["dst"] < 15):
                chkRng = self.indRng(-self.lookRngS, self.lookRngS, angMin, _inc)
        return _max["dst"], _max["ang"]

    def marching(self, indx, _dats, _ints, angMin, _inc):
        rng = [indx-self.prec*self.skip2, indx+self.prec*self.skip2]
        # if(rng[0] < self.a2i(-self.lookRng, angMin, _inc)):
        #     rng[0] = self.a2i(-self.lookRng, angMin, _inc)
        # if(rng[1] > self.a2i(self.lookRng, angMin, _inc)):
        #     rng[1] = self.a2i(self.lookRng, angMin, _inc)

        self.datass.append([_dats[indx]*cos(self.i2a(indx, angMin, _inc)), _ints[indx]*sin(self.i2a(indx, angMin, _inc)), _ints[indx] ])


        targetRay = {
            "dst": _dats[indx],
            "ang": self.i2a(indx, angMin, _inc)
        }
        
        _min = {"dst": 1000, "ang": 0}
        for i in range(rng[0], rng[1], self.skip2):
            if(i >= 0 and i < len(_dats)):
                ray = {
                    "dst": _dats[i],
                    "ang": self.i2a(i, angMin, _inc)
                }

                hit = self.hitC(targetRay, ray, self.castR)
                if(hit):
                    if(hit < _min["dst"]):
                        _min = {
                            "dst": ray["dst"],
                            "ang": targetRay["ang"]
                        }
        if _min["dst"] >= 1000:
            # No obstacles found, return the target ray distance
            return {"dst": targetRay["dst"], "ang": targetRay["ang"]}
        else:
            # Obstacle found, return the closest one
            return {"dst": _min["dst"], "ang": _min["ang"]}
    
    def fix_missing(self, _dats, _ints, i):

        first = i
        last = i
        while(first > 0 and (_ints[first] == 0.0 or _dats[first] > 3.0)):
            first -= 1
            if(first < 0):
                first = 0
                break
        while(last < len(_ints) and (_ints[last] == 0.0 or _dats[last] > 3.0)):
            last += 1
            if(last >= len(_ints)):
                last = len(_ints) - 1
                break
        
        if(_ints[first] == 0.0 or _ints[last] == 0.0):
            return 0
        if(first == last):
            return _dats[first]
        
        return self.lerp(_dats[first], _dats[last], (i-first)/(last-first))

    def hitC(self, rayPnt, checkPnt, R):
        dTheta = abs(rayPnt["ang"] - checkPnt["ang"])     
        if(checkPnt["dst"] <= 0.0):
            return False   
        collAng = R/checkPnt["dst"]
        if(dTheta <= collAng):
            return checkPnt["dst"]
        
        return False
    
    def dangerSense(self, _dats, angMin, _inc):
        # print(len(_dats))
        for i in range(len(_dats)):
            if(_dats[i] < self.dangerDist):
                ang = degrees(self.i2a(i, angMin, _inc))
                # print(ang)
                if(ang > self.dangerAng[0] and ang < self.dangerAng[1]):
                    return self.remap(_dats[i], 0, self.dangerDist, 1.0, 0.0)
                if(ang > -self.dangerAng[1] and ang < -self.dangerAng[0]):
                    return -self.remap(_dats[i], 0, self.dangerDist, 1.0, 0.0)
        return False

    def pubDrive(self):
        throttle_msg = Float32()
        throttle_msg.data = self.speed
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()
        steer_msg.data = self.strAngle
        self.steer_pub.publish(steer_msg)

        self.get_logger().info(f'Speed: {self.speed}, Steering: {self.strAngle}')

    def pubDebugPoint(self):
        targetPnts = PointCloud()
        targetPnts.channels.append(ChannelFloat32(name="intensity", values=[]))
        targetPnts.header.stamp = self.get_clock().now().to_msg()
        targetPnts.header.frame_id = "laser"

        ang = radians(self.targetAng)
        offX = self.castR*cos(ang + pi/2)
        offY = self.castR*sin(ang + pi/2)

        for i in range(0, int(self.targetD/0.1)):
            x = (i*0.1)*cos(ang) + offX
            y = (i*0.1)*sin(ang) + offY
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.2)
        offX = self.castR*cos(ang - pi/2)
        offY = self.castR*sin(ang - pi/2)
        for i in range(0, int(self.targetD/0.1)):
            x = (i*0.1)*cos(ang) + offX
            y = (i*0.1)*sin(ang) + offY
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.4)

        for i in range(-int(pi*25), int(pi*25), 1):
            ang = i/25.0
            x = self.dangerDist*cos(ang)
            y = self.dangerDist*sin(ang)
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))

            ang = degrees(ang)
            if(ang > self.dangerAng[0] and ang < self.dangerAng[1]):
                targetPnts.channels[0].values.append(0.0)
            elif(ang > -self.dangerAng[1] and ang < -self.dangerAng[0]):
                targetPnts.channels[0].values.append(1.0)
            else:
                targetPnts.channels[0].values.append(0.5)
        
        for i in range(len(self.datass)):
            targetPnts.points.append(Point32(x=self.datass[i][0], y=self.datass[i][1], z=0.0))
            targetPnts.channels[0].values.append(datass[i][2])

        self.pubDebug.publish(targetPnts)

    def i2a(self, i, angMin, angInc):
        """Get angle value from the index value"""
        return angMin + angInc*i
    
    def indRng(self, Min, Max, angMin, angInc):
        return [self.a2i(Min, angMin, angInc), self.a2i(Max, angMin, angInc)]

    def a2i(self, ang, angMin, angInc):
        """Get index value from the angle value"""
        return round((ang - angMin) / angInc)

    def clamp(self, val, mini, maxi):
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
        newVal = (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min
        return self.clamp(newVal, new_min, new_max)

    def lerp(self, a, b, t):
        return self.clamp(a + (b - a) * t, a, b)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()