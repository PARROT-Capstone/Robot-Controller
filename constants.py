import time
import os


'''
Computer Vision Constants
'''
# The fiducial marker indexes for the sandbox perimeter and goals. 
# Ordering is not important.
CORNER_FIDUCIALS = [1,4,7,10]
PALLET_FIDUCIALS = [1,2,4,5,7,8,10]

WEBCAM_ID = 0

CV_DEBUG = True

path = os.getcwd()
CV_DEBUG_IMAGE_PATH = path + "/computerVisionFragments/simRobots.JPG"

# Note this is HSV, not RGB
CV_ROBOT_VARIENCE_LOWER_BOUND = (0, 0, 230)
CV_ROBOT_VARIENCE_UPPER_BOUND = (255, 255, 255)

CV_LEDS_PER_ROBOT = 5

# The physical real world width of the fiducial markers in mm.
FIDUCIAL_WIDTH_MM = 30 # TODO: Just a guess

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