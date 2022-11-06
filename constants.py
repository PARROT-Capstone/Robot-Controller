from asyncio import constants
import time
import os
import numpy as np


'''
Computer Vision Constants
'''
CV_DEBUG = False
CV_USE_CAMERA = True

# visualizer constants
CV_VISUALIZE_PATH = False
CV_VISUALIZE_ACTUAL_PATH = False

CV_LOCALIZE_ROBOTS_FIDUCIALS = True

# The fiducial marker indexes for the sandbox perimeter and goals. 
# Ordering is not important.
CORNER_FIDUCIALS = [1,4,7,10]
PALLET_FIDUCIALS = [2]
ROBOT_FIDUCIALS = [0, 3]

WEBCAM_ID = 1

path = os.getcwd()
CV_DEBUG_IMAGE_PATH = path + "/computerVisionFragments/field1080p.jpg"

# The physical real world width of the fiducial markers in mm.
FIDUCIAL_WIDTH_MM = 30 # TODO: Just a guess

CV_SANDBOX_IMAGE_BUFFER_PERCENT = 0.1 # How much extra to scale the image by when cropping the sandbox
CV_SANDBOX_HEIGHT = 1000 # mm
CV_SANDBOX_WIDTH = 1500 # mm

CAMERA_MATRIX = np.array([[2.03722817e+03, 0.00000000e+00, 1.50860597e+03], \
    [0.00000000e+00, 2.03842706e+03, 2.00969117e+03], \
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
DISTORTION_COEFFICIENTS = np.array([[ 1.13393179e-01, -2.99566559e-01, -1.80836671e-05, -7.23427956e-05, 2.41628393e-01]])

if CV_LOCALIZE_ROBOTS_FIDUCIALS == False:

    CV_LEDS_PER_ROBOT = 5

    CV_LED_MIN_CIRCLE = 4
    CV_LED_MAX_CIRCLE = 10
    CV_CIRCLE_PARAM1 = 50
    CV_CIRCLE_PARAM2 = 5
    CV_PIXEL_DISTANCE_BETWEEN_LEDS = 20
    CV_CIRCLE_MASK_MULTIPLIER = 1.2

    # Note this is HSV, not RGB
    CV_ROBOT_VARIENCE_LOWER_BOUND = (0, 50, 150)
    CV_ROBOT_VARIENCE_UPPER_BOUND = (255, 255, 255)

    CV_MIN_LED_AREA = 30
    CV_MAX_LED_AREA = 90

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
tangent_curviness = 1.25 # multiplier for curviness of the spline interpolation
maxRobotSpeed = 100 #mm/s
CONTROLS_DEBUG = False
CONTROLS_ELECTROMAGNET_TIME_THRESHOLD = 0.5 #s
CONTROLS_STATE_DRIVING_TO_GOAL = 0
CONTROLS_STATE_DRIVING_TO_PALLET = 1
CONTROLS_MAX_PWM_OFFSET = 40
CONTROLS_MAX_PWM = 90 + CONTROLS_MAX_PWM_OFFSET
CONTROLS_MIN_PWM = 90 - CONTROLS_MAX_PWM_OFFSET


CONTROLS_ROBOT_PID_KPx = 1
CONTROLS_ROBOT_PID_KIx = 0.0
CONTROLS_ROBOT_PID_KDx = 1
CONTROLS_ROBOT_PID_KPy = 0.02
CONTROLS_ROBOT_PID_KIy = 0.0
CONTROLS_ROBOT_PID_KDy = 0.0
CONTROLS_ROBOT_PID_KPtheta = 1
CONTROLS_ROBOT_PID_KItheta = 0.0
CONTROLS_ROBOT_PID_KDtheta = 0.0

CONTROLS_ELECTROMAGNET_TIME_THRESHOLD = 0.5

CONTROLS_STATE_DRIVING_TO_PALLET = 0
CONTROLS_STATE_DRIVING_TO_GOAL = 1

ELECTROMAGNET_DONT_SEND = 0
ELECTROMAGNET_ENABLE = 1
ELECTROMAGNET_DISABLE = 2