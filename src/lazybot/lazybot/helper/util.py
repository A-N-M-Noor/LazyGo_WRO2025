import cv2
import numpy as np
import pathlib
import ruamel.yaml

from math import pi, radians, degrees, sin, cos, sqrt, inf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

directory = pathlib.Path("~/WRO_25_ws/src/lazybot/lazybot").expanduser()
config_dir = directory / 'configure'
color_calib_file_cam = config_dir / 'color_data.yaml'
color_calib_file_sim = config_dir / 'color_data_sim.yaml'
color_calib_file = color_calib_file_cam

yaml = ruamel.yaml.YAML(pure=True)
yaml.indent(mapping=4)
yaml.default_flow_style = None

color_calib_data = None

color_spaces = {
    "HSV": cv2.COLOR_BGR2HSV,
    "HLS": cv2.COLOR_BGR2HLS,
    "LAB": cv2.COLOR_BGR2Lab,
    "LUV": cv2.COLOR_BGR2Luv,
    "YCrCb": cv2.COLOR_BGR2YCrCb
}

def set_sim():
    global color_calib_file
    color_calib_file = color_calib_file_sim

# set_sim()

def get_color_calib_file():
    with open(color_calib_file, 'r') as file:
        return file.read()

def getConfig():
    global yaml_str

    yaml_str = get_color_calib_file()

    with open(color_calib_file) as file:
        config = yaml.load(yaml_str)
        return config
    return None

def load_color_calib():
    yaml_str = ""
    with open(color_calib_file, 'r') as file:
        yaml_str = file.read()
        return yaml.load(yaml_str)
    return None
    
def save_color_calib(values):
    with open(color_calib_file, 'w') as file:
        yaml.dump(color_calib_data, file)

color_calib_data = load_color_calib()

def get_colors():
    return list(color_calib_data['colors'].keys())

def get_main_blur_val():
    return color_calib_data['main_blur']

def get_mask_blur_val():
    return color_calib_data['mask_blur']

def get_gamma_val():
    return color_calib_data['gamma']

def get_range_data(color):
    rngObject = color_calib_data['colors'][color]
    rngList = [rngObject['min'], rngObject['max']]
    return rngList

def save_range_data(cSpace, color, rng, blr, mskBlr, gamma):
    color_calib_data['color_space'] = cSpace
    color_calib_data['colors'][color]['min'] = rng[0]
    color_calib_data['colors'][color]['max'] = rng[1]
    color_calib_data['main_blur'] = int(blr)
    color_calib_data['mask_blur'] = int(mskBlr)
    color_calib_data['gamma'] = round(gamma, 2)

    save_color_calib(color_calib_data)

def make_mask(_hsv, rngMin, rngMax):
    lower_range = np.array(rngMin)
    upper_range = np.array(rngMax)

    mask = None
    if(rngMin[0] < rngMax[0]):
        mask = cv2.inRange(_hsv, lower_range, upper_range)
    else:
        mask = cv2.inRange(_hsv, np.array([0, rngMin[1], rngMin[2]]), upper_range)
        mask2 = cv2.inRange(_hsv, lower_range, np.array([180, rngMax[1], rngMax[2]]))

        mask = mask + mask2

    return mask

def set_space(scp):
    if scp in color_spaces:
        color_calib_data['color_space'] = scp

def preprocess_image(img, clip_limit=3.0, tile_grid_size=(8,8), gamma=1.5, blr=0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(256)]).astype("uint8")
    gamma_img = cv2.LUT(img, table)
    
    ret_img = gamma_img
    blr = int(blr)
    if blr > 0:
        if blr % 2 == 0:
            blr += 1
        ret_img = cv2.GaussianBlur(ret_img, (blr, blr), cv2.BORDER_DEFAULT)

    return ret_img

def process_mask(frame, rng = None, maskBlr=0, crop = None, hsv = None):
    frm = frame.copy()
    if(crop is not None):
        frm = frm[crop[0]:crop[1], :]
    if hsv is None:
        hsv = cv2.cvtColor(frm, color_spaces[color_calib_data['color_space']])
        
    if rng is None:
        return hsv, None, None
    mask = make_mask(hsv, rng[0], rng[1])
    blrMask = blur(mask, maskBlr)
    _, thresh = cv2.threshold(blrMask, 127, 255, cv2.THRESH_BINARY)
    
    return hsv, blrMask, thresh

def blur(frm, amount):
    amount = int(amount)
    if(amount > 0):
        _frm = frm.copy()
        if(amount%2 == 0):
            amount = amount+1
        _frm = cv2.GaussianBlur(_frm, (amount,amount), cv2.BORDER_DEFAULT)
        return _frm
    return frm

def get_contours(mask):
    cnt, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    return cnt
    
# def clamp(_num, min_value = 0, max_value = 255):
#     return max(min(_num, max_value), min_value)

def dist(x1, y1, x2, y2):
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

def local2global(robot_pos: Vector3, local_x, local_y):
    x = robot_pos.x
    y = robot_pos.y
    a = robot_pos.z
    global_x = x + local_x * cos(a) - local_y * sin(a)
    global_y = y + local_x * sin(a) + local_y * cos(a)

    return global_x, global_y

def global2local(robot_pos: Vector3, global_x, global_y):
    dx = global_x - robot_pos.x
    dy = global_y - robot_pos.y

    local_x =  dx * cos(robot_pos.z) + dy * sin(robot_pos.z)
    local_y = dx * sin(robot_pos.z) + dy * cos(robot_pos.z)

    return local_x, local_y


def clamp(val, mini = 0, maxi = 255):
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
            
def remap(old_val, old_min, old_max, new_min, new_max):
    newVal = (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min
    return clamp(newVal, new_min, new_max)

def lerp(a, b, t):
    return clamp(a + (b - a) * t, a, b)

def norm_ang(a):
        return (a + pi) % (2 * pi) - pi



class LidarHandler:
    def __init__(self):
        self.angMin = -pi
        self.angMax = pi
        self.angInc = radians(0.5)
        self.ranges = []
        self.ints = []
    

    def set_laser_vals(self, scan: LaserScan):
        self.angMin = scan.angle_min
        self.angMax = scan.angle_max
        self.angInc = scan.angle_increment
        self.ranges = scan.ranges
        self.ints = scan.intensities
    
    def fix_missing(self, i):
        if(i < 0 or i >= len(self.ints)):
            return 0
        if(self.ints[i] > 0.05 and self.ranges[i] <= 3.0):
            return self.ranges[i]
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
        
        return lerp(self.ranges[first], self.ranges[last], (i-first)/(last-first))
    
    def get_dst(self, ang):
        i = self.a2i(radians(ang))
        if(self.ints[i] <= 0.05 or self.ranges[i] > 3.0):
            self.ranges[i] = self.fix_missing(i)
            self.ints[i] = 1.0
        return self.ranges[i]
    
    def i2a(self, i, deg = False):
        if deg:
            return degrees(self.angMin + self.angInc*i)
        return self.angMin + self.angInc*i
    
    def indRng(self, Min, Max):
        return [self.a2i(Min), self.a2i(Max)]

    def a2i(self, ang, degrees = False):
        if degrees:
            ang = radians(ang)
        return round((ang -self.angMin) / self.angInc)