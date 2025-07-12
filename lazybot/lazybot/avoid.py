import rclpy
from rclpy.node import Node

from threading import Thread
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2, time
import numpy as np


def clamp(val, mini, maxi):
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

def mapF(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def mapFC(x, in_min, in_max, out_min, out_max):
    return clamp( mapF(x, in_min, in_max, out_min, out_max), out_min, out_max)


def lerpF(a, b, t):
    return a + t * (b - a)


class vStream:
    def __init__(self,src,width=860,height=480, Type=False, blr = 5):
        self.width=width
        self.height=height
        if(Type):
            self.capture=cv2.VideoCapture(src, Type)
        else:
            self.capture=cv2.VideoCapture(src)
        
        self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.5)  # Set auto exposure to manual
        self.capture.set(cv2.CAP_PROP_EXPOSURE, -2)  # Set exposure value
        self.startTime=time.time()
        self.dtav=0
        self.fps = 0

        self.thread=Thread(target=self.update,args=())
        self.thread.daemon=True
        self.thread.start()
        self.frame=None
        self.frame2=None

    def update(self):
        while True:
            _,self.frame=self.capture.read()
            self.frame2=cv2.resize(self.frame,(self.width,self.height))

            self.dt=time.time()-self.startTime
            self.startTime=time.time()
            self.dtav=.9*self.dtav+.1*self.dt
            self.fps=1/self.dtav
    def getFrame(self):
        return self.frame2

class Avoid(Node):
    def __init__(self):
        super().__init__('avoid')

        self.dispW=860
        self.dispH=480
        self.cap=vStream(0, blr=3)
        self.font=cv2.FONT_HERSHEY_SIMPLEX

        self.areaMin = 200

        self.timer = self.create_timer(0.1, self.feed_timer_callback)

        self.pub = self.create_publisher(CompressedImage, "/lazycam", 10)
        self.strPub = self.create_publisher(Float32, "/objPs", 10)
        self.get_logger().info('Avoid node has been started.')

    
    def makeMask(self, _hsv, rngMin, rngMax):
        lower_range = np.array(rngMin)
        upper_range = np.array(rngMax)

        

        # Create a black mask with a white horizontal stripe in the middle
        _mask = np.zeros(_hsv.shape[:2], dtype=np.uint8)
        thickness = 100  # Set your desired stripe thickness here
        center = _mask.shape[0] // 2
        top = max(center - thickness // 2, 0)
        bottom = min(center + thickness // 2, _mask.shape[0])
        _mask[top:bottom, :] = 255

        mask = None
        if(rngMin[0] < rngMax[0]):
            mask = cv2.inRange(_hsv, lower_range, upper_range)
        else:
            mask = cv2.inRange(_hsv, np.array([0, rngMin[1], rngMin[2]]), upper_range)
            mask2 = cv2.inRange(_hsv, lower_range, np.array([179, rngMax[1], rngMax[2]]))

            mask = mask + mask2
        mask = cv2.bitwise_and(mask, _mask)
        ret, thresh = cv2.threshold(mask, 150, 255, cv2.THRESH_BINARY)
        cnt, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        return cnt, mask

    def getObjs(self, _frame, _cnt, colName, areaThr):
        objList = []

        for cont in _cnt:
            cArea = cv2.contourArea(cont)
            bArea = cv2.contourArea( cv2.convexHull(cont) )
            solidity = 0
            if(cArea > 0 and bArea > 0):
                solidity = cArea/bArea
            if( cArea > areaThr and solidity > 0.75):
                x,y,w,h = cv2.boundingRect(cont)
                rct = cv2.minAreaRect(cont)
                rBnd = np.int0(cv2.boxPoints(rct))
                pX, pY, sWA, sWB = int(rct[0][0]), int(rct[0][1]), int(rct[1][0]), int(rct[1][1])

                sW = min(sWA, sWB)
                sH = max(sWA, sWB)
                if( (w*h)/(sWA*sWB) < 1.25 ):
                    sW = w
                    sH = h
                cv2.circle(_frame, (int(rct[0][0]), int(rct[0][1])), 5, (255, 0, 0), -1)
                posX = pX-sW/2 if(colName=="G") else pX+sW/2
                _obj = {
                    "color": colName,
                    "pos": int(mapFC(posX, 0, self.dispW, 0, 200)),
                    "dist": int(3462 / sW)-7,
                    "coord": (pX, pY)
                }
                if(sW/sH < 1.25 and sW/sH > 0.25):
                    objList.append(_obj)
                    cv2.rectangle(_frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    cv2.drawContours(_frame, [rBnd], -1, (255,0,0),2)
                    cv2.putText(_frame,f"{_obj['color']} | {_obj['pos']} | {_obj['dist']}({sW})",(pX, pY), self.font, .5,(0,255,255),2)

        return objList

    def feed_timer_callback(self):
        frame = self.cap.getFrame()
        if(frame is None):
            return

        _hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        cntG, maskG = self.makeMask(_hsv, [43, 89, 26], [103, 213, 233])
        cntR, maskR = self.makeMask(_hsv, [128, 82, 5],  [35, 243, 228])

        objectsG = self.getObjs(frame, cntG, "G", self.areaMin)
        objectsR = self.getObjs(frame, cntR, "R", self.areaMin)

        objList = objectsG + objectsR
        objType = "N"
        objPos = 0
        objDist = 200
        objList.sort(key=lambda obj:obj["dist"])

        bar = self.dispW // 2

        dir = 0

        if(len(objList) > 0):
            objType = objList[0]["color"]
            objPos = objList[0]["pos"]
            objDist = objList[0]["dist"]

            self.get_logger().info(f"Detected object: {objList[0]}")

        if(objType == "G"):
            bar += mapFC(objDist, 30, 200, self.dispW//4, self.dispW//8)
            if(objPos < bar):
                dir = -1
        elif(objType == "R"):
            bar -= mapFC(objDist, 30, 200, self.dispW//4, self.dispW//8)
            if(objPos > bar):
                dir = 1
        cv2.line(frame, (int(bar), 0), (int(bar), self.dispH), (0, 255, 0), 2)
        stmsg = Float32()
        stmsg.data = float(dir)
        self.strPub.publish(stmsg)

        
        _frm = cv2.resize(frame, (self.dispW//2, self.dispH//2))
        
        _, compressed = cv2.imencode('.jpg', _frm, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.format = "jpeg"
        msg.data = compressed.tobytes()

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Avoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()