import time
# from cv_main import CV
import os
os.environ["OPENCV_LOG_LEVEL"]="SILENT"
os.environ["OPENCV_LOG_LEVEL"]="FATAL"

import cv2 as cv
import constants
start = time.time()
cap = cv.VideoCapture(0)
ret, frame = cap.read()
cap.release()
end = time.time()
print("Time is ", end-start)

while True:
    start = time.time()
    print("Test")
    time.sleep(1)
    print("Time: ",start - time.time())