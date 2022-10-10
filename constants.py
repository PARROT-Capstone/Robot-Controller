import time


'''
Computer Vision Constants
'''
# The fiducial marker indexes for the sandbox perimeter and goals. 
# Ordering is not important.
CORNER_FIDUCIALS = [0,3,6,9]
PALLET_FIDUCIALS = [1,2,4,5,7,8,10]


ROBOT_ID_TO_COLOR = {
    0: (0, 0, 255),
    1: (0, 255, 0),
    2: (255, 0, 0),
    3: (255, 255, 0),
    4: (255, 0, 255),
    5: (0, 255, 255),
    6: (255, 255, 255),
    7: (0, 0, 0),
    8: (128, 128, 128),
    9: (128, 0, 0)
}

# The physical real world width of the fiducial markers in mm.
FIDUCIAL_WIDTH_MM = 30 # TODO: Just a guess

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