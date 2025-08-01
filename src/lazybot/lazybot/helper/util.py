import cv2
import numpy as np
import pathlib
import ruamel.yaml

directory = pathlib.Path(__file__).parent.parent.resolve()
config_dir = directory / 'configure'
color_calib_file = config_dir / 'color_data.yaml'

yaml = ruamel.yaml.YAML(pure=True)
yaml.indent(mapping=4)
yaml.default_flow_style = None

color_calib_data = None

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

def get_range_data(color):
    rngObject = color_calib_data['colors'][color]
    rngList = [rngObject['min'], rngObject['max']]
    return rngList

def save_range_data(color, rng, blr, mskBlr):
    color_calib_data['colors'][color]['min'] = rng[0]
    color_calib_data['colors'][color]['max'] = rng[1]
    color_calib_data['main_blur'] = int(blr)
    color_calib_data['mask_blur'] = int(mskBlr)
    
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

def process_mask(frame, rng, maskBlr=0, crop = None, hsv = None):
    frm = frame.copy()
    if(crop is not None):
        frm = frm[crop[0]:crop[1], :]
    if hsv is None:
        hsv = cv2.cvtColor(frm, cv2.COLOR_BGR2HSV)
        

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
    
def clamp(_num, min_value = 0, max_value = 255):
    return max(min(_num, max_value), min_value)