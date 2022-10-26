from asyncio import constants
import time
import os
import numpy as np


'''
Computer Vision Constants
'''
# The fiducial marker indexes for the sandbox perimeter and goals. 
# Ordering is not important.
CORNER_FIDUCIALS = [1,4,7,10]
PALLET_FIDUCIALS = [1,2,4,5,7,8,10]

WEBCAM_ID = 2

CV_DEBUG = True

path = os.getcwd()
CV_DEBUG_IMAGE_PATH = path + "/computerVisionFragments/simRobots.JPG"
CV_DEBUG_IMAGE_PATH = path + "/computerVisionFragments/fullTest2.JPG"

# Note this is HSV, not RGB
CV_ROBOT_VARIENCE_LOWER_BOUND = (0, 0, 253)
CV_ROBOT_VARIENCE_UPPER_BOUND = (255, 255, 255)

# visualizer constants
CV_VISUALIZE_PATH = False
CV_VISUALIZE_ACTUAL_PATH = False

CV_LEDS_PER_ROBOT = 5

CV_LED_MIN_CIRCLE = 5
CV_LED_MAX_CIRCLE = 15
CV_CIRCLE_PARAM1 = 50
CV_CIRCLE_PARAM2 = 16
CV_PIXEL_DISTANCE_BETWEEN_LEDS = 20
CV_CIRCLE_MASK_MULTIPLIER = 1.5

# The physical real world width of the fiducial markers in mm.
FIDUCIAL_WIDTH_MM = 30 # TODO: Just a guess

CV_INIT_RESOLUTION = 3000 * 2000 # rough estimate of a starting resolution
CV_SANDBOX_IMAGE_BUFFER_PERCENT = 0.1 # How much extra to scale the image by when cropping the sandbox

CAMERA_MATRIX = np.array([[2.03722817e+03, 0.00000000e+00, 1.50860597e+03], \
    [0.00000000e+00, 2.03842706e+03, 2.00969117e+03], \
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
DISTORTION_COEFFICIENTS = np.array([[ 1.13393179e-01, -2.99566559e-01, -1.80836671e-05, -7.23427956e-05, 2.41628393e-01]])

def debugPrint(debugMsg):
    if CV_DEBUG:
        print(debugMsg)

def blockingError(errorMsg):
    while(True):
        print(errorMsg)
        time.sleep(1)


'''
Controls Constants
'''
wheel_base = 100 # mm # wheelbase is actually 100mm
half_wheel_base = 50 # mm # wheelbase is actually 100mm
wheel_radius = 30 # mm
controlsLinearSpeed = 1 #mm/s
controlsDeltaTime = 0.001 #s