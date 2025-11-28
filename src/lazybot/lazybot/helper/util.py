import cv2
import numpy as np
import pathlib
import ruamel.yaml

from math import pi, radians, degrees, sin, cos, sqrt, inf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# --- Configuration Paths ---
# Sets up paths to load/save color calibration data
directory = pathlib.Path("~/WRO_25_ws/src/lazybot/lazybot").expanduser()
config_dir = directory / 'configure'
color_calib_file_cam = config_dir / 'color_data.yaml'
color_calib_file_sim = config_dir / 'color_data_sim.yaml'
color_calib_file = color_calib_file_cam # Default to physical camera config

# --- YAML Setup ---
# Configures YAML parser to preserve comments and formatting
yaml = ruamel.yaml.YAML(pure=True)
yaml.indent(mapping=4)
yaml.default_flow_style = None

color_calib_data = None

# Supported Color Spaces for OpenCV conversion
color_spaces = {
    "HSV": cv2.COLOR_BGR2HSV,
    "HLS": cv2.COLOR_BGR2HLS,
    "LAB": cv2.COLOR_BGR2Lab,
    "LUV": cv2.COLOR_BGR2Luv,
    "YCrCb": cv2.COLOR_BGR2YCrCb
}

def set_sim():
    """Switches configuration to use simulation parameters."""
    global color_calib_file
    color_calib_file = color_calib_file_sim

# set_sim() # Uncomment to force simulation mode by default

def get_color_calib_file():
    """Reads the raw YAML string from the config file."""
    with open(color_calib_file, 'r') as file:
        return file.read()

def getConfig():
    """Loads the YAML configuration into a dictionary."""
    global yaml_str

    yaml_str = get_color_calib_file()

    with open(color_calib_file) as file:
        config = yaml.load(yaml_str)
        return config
    return None

def load_color_calib():
    """Helper to load calibration data safely."""
    yaml_str = ""
    with open(color_calib_file, 'r') as file:
        yaml_str = file.read()
        return yaml.load(yaml_str)
    return None
    
def save_color_calib(values):
    """Writes the current calibration data back to the YAML file."""
    with open(color_calib_file, 'w') as file:
        yaml.dump(color_calib_data, file)

# Load data on module import
color_calib_data = load_color_calib()

# --- Getters for Calibration Data ---

def get_colors():
    """Returns a list of configured color names (e.g., ['red_tower', 'green_tower'])."""
    return list(color_calib_data['colors'].keys())

def get_main_blur_val():
    return color_calib_data['main_blur']

def get_mask_blur_val():
    return color_calib_data['mask_blur']

def get_gamma_val():
    return color_calib_data['gamma']

def get_range_data(color):
    """Returns [min, max] HSV/LAB ranges for a specific color."""
    rngObject = color_calib_data['colors'][color]
    rngList = [rngObject['min'], rngObject['max']]
    return rngList

def save_range_data(cSpace, color, rng, blr, mskBlr, gamma):
    """Updates the internal data structure and saves to disk."""
    color_calib_data['color_space'] = cSpace
    color_calib_data['colors'][color]['min'] = rng[0]
    color_calib_data['colors'][color]['max'] = rng[1]
    color_calib_data['main_blur'] = int(blr)
    color_calib_data['mask_blur'] = int(mskBlr)
    color_calib_data['gamma'] = round(gamma, 2)

    save_color_calib(color_calib_data)

# --- Image Processing Utilities ---

def make_mask(_hsv, rngMin, rngMax):
    """
    Creates a binary mask for a given color range.
    Handles Hue wrapping (e.g., Red crossing 179->0).
    """
    lower_range = np.array(rngMin)
    upper_range = np.array(rngMax)

    mask = None
    if(rngMin[0] < rngMax[0]):
        # Standard case: Min < Max
        mask = cv2.inRange(_hsv, lower_range, upper_range)
    else:
        # Wrap-around case: Range crosses 0 (e.g., Red)
        # Split into two ranges: [Min -> 180] and [0 -> Max]
        mask = cv2.inRange(_hsv, np.array([0, rngMin[1], rngMin[2]]), upper_range)
        mask2 = cv2.inRange(_hsv, lower_range, np.array([180, rngMax[1], rngMax[2]]))

        mask = mask + mask2

    return mask

def set_space(scp):
    """Updates the active color space (HSV, LAB, etc.)."""
    if scp in color_spaces:
        color_calib_data['color_space'] = scp

def preprocess_image(img, clip_limit=3.0, tile_grid_size=(8,8), gamma=1.5, blr=0):
    """
    Applies Gamma Correction and Gaussian Blur to the image.
    Gamma correction helps in low-light or high-contrast environments.
    """
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(256)]).astype("uint8")
    gamma_img = cv2.LUT(img, table)
    
    ret_img = gamma_img
    blr = int(blr)
    if blr > 0:
        if blr % 2 == 0:
            blr += 1 # Kernel size must be odd
        ret_img = cv2.GaussianBlur(ret_img, (blr, blr), cv2.BORDER_DEFAULT)

    return ret_img

def process_mask(frame, rng = None, maskBlr=0, crop = None, hsv = None):
    """
    Full pipeline: Convert Color Space -> Threshold -> Blur Mask.
    Returns: Converted Image, Blurred Mask, Binary Threshold
    """
    frm = frame.copy()
    if(crop is not None):
        frm = frm[crop[0]:crop[1], :]
    
    # Convert to selected color space if not already provided
    if hsv is None:
        hsv = cv2.cvtColor(frm, color_spaces[color_calib_data['color_space']])
        
    if rng is None:
        return hsv, None, None
    
    mask = make_mask(hsv, rng[0], rng[1])
    blrMask = blur(mask, maskBlr)
    _, thresh = cv2.threshold(blrMask, 127, 255, cv2.THRESH_BINARY)
    
    return hsv, blrMask, thresh

def blur(frm, amount):
    """Helper for Gaussian Blur."""
    amount = int(amount)
    if(amount > 0):
        _frm = frm.copy()
        if(amount%2 == 0):
            amount = amount+1
        _frm = cv2.GaussianBlur(_frm, (amount,amount), cv2.BORDER_DEFAULT)
        return _frm
    return frm

def get_contours(mask):
    """Finds external contours in a binary mask."""
    cnt, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    return cnt
    
# --- Math Utilities ---

def dist(x1, y1, x2, y2):
    """Euclidean distance between two points."""
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

def local2global(robot_pos: Vector3, local_x, local_y):
    """Transforms local robot coordinates to global map coordinates."""
    x = robot_pos.x
    y = robot_pos.y
    a = robot_pos.z
    global_x = x + local_x * cos(a) - local_y * sin(a)
    global_y = y + local_x * sin(a) + local_y * cos(a)

    return global_x, global_y

def global2local(robot_pos: Vector3, global_x, global_y):
    """Transforms global map coordinates to local robot coordinates."""
    dx = global_x - robot_pos.x
    dy = global_y - robot_pos.y

    local_x =  dx * cos(robot_pos.z) + dy * sin(robot_pos.z)
    local_y = dx * sin(robot_pos.z) + dy * cos(robot_pos.z)

    return local_x, local_y


def clamp(val, mini = 0, maxi = 255):
    """Constrains a value between min and max."""
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
    """Maps a value from one range to another."""
    newVal = (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min
    return clamp(newVal, new_min, new_max)

def lerp(a, b, t):
    """Linear interpolation between a and b by factor t."""
    return clamp(a + (b - a) * t, a, b)

def norm_ang(a):
    """Normalizes angle to be within [-pi, pi]."""
    return (a + pi) % (2 * pi) - pi


# --- LiDAR Helper Class ---

class LidarHandler:
    """
    Helper class to manage LaserScan data.
    Provides methods to convert between indices and angles, 
    and handle missing/invalid data points.
    """
    def __init__(self):
        self.angMin = -pi
        self.angMax = pi
        self.angInc = radians(0.5)
        self.ranges = []
        self.ints = []
    

    def set_laser_vals(self, scan: LaserScan):
        """Updates internal state with new LaserScan message."""
        self.angMin = scan.angle_min
        self.angMax = scan.angle_max
        self.angInc = scan.angle_increment
        self.ranges = scan.ranges
        self.ints = scan.intensities
    
    def fix_missing(self, i):
        """
        Interpolates missing LiDAR data (inf or 0.0) using neighbors.
        Useful for filtering out noise or glass reflections.
        """
        if(i < 0 or i >= len(self.ints)):
            return 0
        # If valid, return as is
        if(self.ints[i] > 0.05 and self.ranges[i] <= 4.0 and self.ranges[i] != float('inf')):
            return self.ranges[i]
        
        # Search for nearest valid neighbors
        first = i
        last = i
        
        while(first > 0 and (self.ints[first] == 0.0 or self.ranges[first] > 4.0) or self.ranges[first] == float('inf')):
            first -= 1
            if(first < 0):
                first = 0
                break
        while(last < len(self.ints) and (self.ints[last] == 0.0 or self.ranges[last] > 4.0) or self.ranges[last] == float('inf')):
            last += 1
            if(last >= len(self.ints)):
                last = len(self.ints) - 1
                break
        
        # If interpolation fails, return 0
        if(self.ints[first] == 0.0 or self.ints[last] == 0.0):
            return 0
        if(first == last):
            return self.ranges[first]
        
        lerpFactor = (i-first)/(last-first)
        
        if abs(self.ranges[first] - self.ranges[last]) > 0.1:
            if lerpFactor < 0.5:
                return self.ranges[first] 
            else:
                return self.ranges[last]
            # return self.ranges[first]
        
        # Linear interpolation
        return lerp(self.ranges[first], self.ranges[last], lerpFactor)
    
    def get_dst(self, ang):
        """Returns distance at a specific angle (in degrees)."""
        i = self.a2i(radians(ang))
        if(self.ints[i] <= 0.05 or self.ranges[i] > 3.0):
            self.ranges[i] = self.fix_missing(i)
            self.ints[i] = 1.0
        return self.ranges[i]
    
    def i2a(self, i, deg = False):
        """Converts index to angle."""
        if deg:
            return degrees(self.angMin + self.angInc*i)
        return self.angMin + self.angInc*i
    
    def indRng(self, Min, Max):
        """Returns index range [start, end] for a given angle range."""
        return [self.a2i(Min), self.a2i(Max)]

    def a2i(self, ang, degrees = False):
        """Converts angle to index."""
        if degrees:
            ang = radians(ang)
        return round((ang -self.angMin) / self.angInc)