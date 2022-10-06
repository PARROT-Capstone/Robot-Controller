import time
'''
Computer Vision Constants
'''

# The fiducial markers used to locate the sandbox perimeter. 
CORNER_FIDUCIALS = [0,3,6,9]

FIDUCIAL_WIDTH_MM = 30 # TODO: Just a guess

def blockingError(errorMsg):
    while(True):
        print(errorMsg)
        time.sleep(1)

