import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32, Float32MultiArray, String
from math import pi, radians, degrees, sin, cos
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.obj_sub = self.create_subscription(String, '/closest_obj', self.obj_callback, 1)
        
        self.objPub = self.create_publisher(Float32MultiArray, '/obj_data', 1)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 1)
        self.steer_pub = self.create_publisher(Float32, '/steer', 1)

        self.pubDebug = self.create_publisher(PointCloud, "/target_point", 10)

        self.IS_SIM = True
        
        self.datass = []

        self.maxSpeed : float = 0.45
        self.speed : float = 0.0
        self.strAngle : float = 0.0
        self.strRange = 1.0
        self.str_ang_thresh = 60.0
        self.strSpd = 0.65
        
        self.prec = 81
        self.skip1 = 2
        self.skip2 = 1

        self.dangerDist = 0.25
        self.dangerAng = [25.0, 90.0]

        self.castRange = [0.15, 0.18]
        self.castR = 0.25
        self.lookRng = radians(80.0)
        self.lookRngS = radians(90.0)
        self.targetAng = 0.0
        self.targetD = 0.0
        
        self.objs = []
        self.cont_stack = []
        
        self.closest = "N"

        self.get_logger().info('Control node has been started.')
        self.lastTime = time.time()

    def obj_callback(self, msg: String):
        self.closest = msg.data
        
    def lidar_callback(self, msg: LaserScan):
        self.dt = time.time() - self.lastTime
        self.lastTime = time.time()

        angMin = msg.angle_min
        angMax = msg.angle_max
        angInc = msg.angle_increment

        angDats = msg.ranges
        angInts = msg.intensities
        
        # self.getObjInds(angMin, angInc, angDats, angInts)
        
        self.castR = self.remap(self.speed/self.maxSpeed, 0.45, 1, self.castRange[0], self.castRange[1])

        self.pubObjData()

        maxD, tA = self.getMaxDOBJ(angMin, angInc, angDats, angInts)
        dS = self.dangerSense(angDats, angMin, angInc)

        if(dS):
            tA -= dS * self.strRange * 1.0

        delta = abs(tA-self.targetAng)
        self.targetAng = tA if(delta > 0.5) else self.lerp(self.targetAng, tA, 35*self.dt)
        self.targetAng = self.lerp(self.targetAng, tA, 0.1)
        self.targetAng = degrees(self.targetAng)
        self.targetD = maxD

        # self.get_logger().info(f"Target Angle: {self.targetAng}, Max Dst: {maxD}")
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

    
    # def getObjInds(self, angMin, _inc, _dats, _ints):
    #     self.objs = []
    #     for i in range(self.a2i(-self.lookRng, angMin, _inc), self.a2i(self.lookRng, angMin, _inc), self.skip1):
    #         if(self.IS_SIM): _ints[i] = 1.0
    #         if(i < 1 or i >= len(_dats) or _ints[i] <= 0.05 or _dats[i] > 3.0):
    #             continue
            
    #         slope = (_dats[i] - _dats[i-self.skip1]) / self.skip1
            
    #         if(slope < -0.15):
    #             self.cont_stack.append(i)
    #         elif(slope > 0.15):
    #             if(len(self.cont_stack) > 0):
    #                 pop = self.cont_stack.pop()
    #                 mid = (i + pop) // 2
    #                 if(_ints[mid] <= 0.1 or _dats[mid] > 3.0):
    #                     _dats[mid] = self.fix_missing(_dats, _ints, mid)
    #                 self.objs.append({
    #                     "index": mid,
    #                     "ang": self.i2a(mid, angMin, _inc),
    #                     "dst": _dats[mid],
    #                     "x": _dats[mid] * cos(self.i2a(mid, angMin, _inc)),
    #                     "y": _dats[mid] * sin(self.i2a(mid, angMin, _inc)),
    #                     "intensity": _ints[mid]
    #                 })
    
    def getMaxDOBJ(self, angMin, _inc, _dats, _ints):
        _max = {"dst": 0, "ang": 0}
        chkRng = self.indRng(-self.lookRng, self.lookRng, angMin, _inc)
        
        self.objs = []
        self.cont_stack = []

        remove = "None" 
        if self.closest == "G":
            remove = "Left"
        elif self.closest == "R":
            remove = "Right"
        
        for i in range(chkRng[0], chkRng[1], self.skip1):
            if(self.IS_SIM): _ints[i] = 1.0
            if(_ints[i] <= 0.1 or _dats[i] > 3.0):
                _dats[i] = self.fix_missing(_dats, _ints, i)
                _ints[i] = 1.0

            if(i < 1 or i >= len(_dats) or _ints[i] <= 0.05 or _dats[i] > 3.0):
                continue
            
            dt = self.marching(i, _dats, _ints, angMin,_inc)
            if(dt["dst"] > _max["dst"]):
                _max = dt

            objectFound = self.detectContrast(i, _dats, angMin, _inc, _ints)
            
            if objectFound:
                if remove == "Left":
                    _max = dt
                if remove == "Right":
                    return _max["dst"], _max["ang"]
                
        return _max["dst"], _max["ang"]
    
    
    def detectContrast(self, i, _dats, angMin, _inc, _ints):
        slope = (_dats[i] - _dats[i-self.skip1]) / self.skip1
        if(slope < -0.15):
            self.cont_stack.append(i)
        elif(slope > 0.15):
            if(len(self.cont_stack) > 0):
                pop = self.cont_stack.pop()
                mid = (i + pop) // 2
                sz = _dats[mid] * abs(i-pop)*_inc
                # print(sz)
                # if(_ints[mid] <= 0.1 or _dats[mid] > 3.0):
                #     _dats[mid] = self.fix_missing(_dats, _ints, mid)
                if( sz > 0.035 and sz < 0.1):
                    self.objs.append({
                        "index": mid,
                        "ang": self.i2a(mid, angMin, _inc),
                        "dst": _dats[mid],
                        "x": _dats[mid] * cos(self.i2a(mid, angMin, _inc)),
                        "y": _dats[mid] * sin(self.i2a(mid, angMin, _inc)),
                        "intensity": _ints[mid]
                    })
                if(_dats[mid] < 1.0):
                    return True
        return False
    # def getMaxD(self, angMin, _inc, _dats, _ints):
    #     _max = {"dst": 0, "ang": 0}
    #     chkRng = self.indRng(-self.lookRng, self.lookRng, angMin, _inc)

    #     mid = (chkRng[0] + chkRng[1]) // 2
    #     for i in range(0, mid - chkRng[0], self.skip1):
    #         if(self.IS_SIM): _ints[i] = 1.0
    #         if(_ints[mid-i] <= 0.1 or _dats[mid-i] > 3.0):
    #             _dats[mid-i] = self.fix_missing(_dats, _ints, mid-i)
    #             _ints[mid-i] = 1.0
    #         if(_ints[mid+i] <= 0.1 or _dats[mid+i] > 3.0):
    #             _dats[mid+i] = self.fix_missing(_dats, _ints, mid+i)
    #             _ints[mid+i] = 1.0

    #         if(mid - i < 0 or mid + i >= len(_dats) or _ints[mid - i] == 0.0 or _ints[mid + i] == 0.0):
    #             continue
    #         dt = self.marching(mid - i, _dats, _ints, angMin,_inc)
    #         if(dt["dst"] > _max["dst"]):
    #             _max = dt
            
    #         dt = self.marching(mid + i, _dats, _ints, angMin,_inc)
    #         if(dt["dst"] > _max["dst"]):
    #             _max = dt
    #         if(mid+i < chkRng[1]-10 and _max["dst"] < 15):
    #             chkRng = self.indRng(-self.lookRngS, self.lookRngS, angMin, _inc)
    #     return _max["dst"], _max["ang"]

    def marching(self, indx, _dats, _ints, angMin, _inc):
        rng = [indx-self.prec*self.skip2, indx+self.prec*self.skip2]

        self.datass.append([_dats[indx]*cos(self.i2a(indx, angMin, _inc)), _ints[indx]*sin(self.i2a(indx, angMin, _inc)), _ints[indx] ])


        targetRay = {
            "dst": _dats[indx],
            "ang": self.i2a(indx, angMin, _inc)
        }
        
        _min = {"dst": 1000, "ang": 0}
        for i in range(rng[0], rng[1], self.skip2):
            if(self.IS_SIM): _ints[i] = 1.0
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
            return {"dst": targetRay["dst"], "ang": targetRay["ang"]}
        else:
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

    def pubDrive(self, disable = False):
        if disable:
            self.speed = 0.0
            self.strAngle = 0.0
        throttle_msg = Float32()
        throttle_msg.data = self.speed
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()
        steer_msg.data = self.strAngle
        self.steer_pub.publish(steer_msg)

        # self.get_logger().info(f'Speed: {self.speed}, Steering: {self.strAngle}')

    def pubObjData(self):
        obj_data = Float32MultiArray()
        obj_data.data = []
        self.objs.sort(key=lambda x: x["dst"], reverse=True)
        for obj in self.objs:
            obj_data.data.append(obj["ang"])
        
        self.objPub.publish(obj_data)
            
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

        circle_points = 50
        for i in range(circle_points):
            ang = 2 * pi * i / circle_points
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
        
        circle_points = 50
        
        for i in range(circle_points):
            theta = 2 * pi * i / circle_points
            x = self.castR * cos(theta)
            y = self.castR * sin(theta)
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.2)
        
        circle_points = 20
        circle_radius = 0.07

        for obj in self.objs:
            cx, cy = obj["x"], obj["y"]
            for j in range(circle_points):
                theta = 2 * pi * j / circle_points
                x = cx + circle_radius * cos(theta)
                y = cy + circle_radius * sin(theta)
                targetPnts.points.append(Point32(x=x, y=y, z=0.0))
                if(self.closest == "R"):
                    targetPnts.channels[0].values.append(0.0)
                elif(self.closest == "G"):
                    targetPnts.channels[0].values.append(0.2)
                else:
                    targetPnts.channels[0].values.append(0.6)

        for i in range(len(self.datass)):
            targetPnts.points.append(Point32(x=self.datass[i][0], y=self.datass[i][1], z=0.0))
            targetPnts.channels[0].values.append(self.datass[i][2])

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