'''
Source: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
'''

import numpy as np
import cv2 as cv

captureImage = False

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

if (captureImage):
    cap = cv.VideoCapture(0)
    ret = False
    while (not ret):
        _, img = cap.read()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (9,6), None)

    cap.release()
    # save image
    cv.imwrite('calibration.JPG', img)
else:
    img = cv.imread('calibration.jpg')

test_img = cv.imread('test_fiducial.jpg')

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# Find the chess board corners
ret, corners = cv.findChessboardCorners(gray, (9,6), None)
print(ret)
# If found, add object points, image points (after refining them)
if ret == True:
    objpoints.append(objp)
    corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    imgpoints.append(corners)
    # Draw and display the corners
    cv.drawChessboardCorners(img, (9,6), corners2, ret)
    # cv.imshow('img', img)
    # cv.waitKey(0)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (np.shape(img)[1], np.shape(img)[0]), 1, (np.shape(img)[1], np.shape(img)[0]))
    # newcameramtx = mtx
    new_image = cv.undistort(img, mtx, dist, None, newcameramtx)
    cv.imshow('img original', img)
    cv.waitKey(0)
    cv.imshow('img undistored', new_image)
    cv.waitKey(0)