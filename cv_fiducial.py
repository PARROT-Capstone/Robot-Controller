'''
'''
from dis import code_info
from email.mime import image
from tkinter import E
from turtle import position
from unicodedata import name
import cv2 as cv
import numpy as np
import os
import time 
import constants
from constants import debugPrint

# cap = cv.VideoCapture(0)
debug = True

class CV_Fiducial:
    def __init__(self):
        self.cv_fiducial_markerDict = {}
        self.cv_fiducial_cornerMarkerDict = {}
        self.cv_fiducial_warpedCornerMarkerDict = {}
        self.mm_per_pixel = None
        self.sandbox_height_mm = None
        self.sandbox_width_mm = None

    def cv_fiducial_setupSandbox(self, image_frame):
        # Find the sandbox corner fiducials's pose
        while(self._cv_fiducial_detectSandboxCorners(image_frame) == False):
            print("No sandbox corners detected.")
            time.sleep(1)

        # Sort them
        top_left_id, top_right_id, bottom_left_id, bottom_right_id = self._cv_fiducial_findCornerFiducialOrdering()

        # Calculate the Sandbox's Aspect Ratio
        length_top = self._cv_fiducial_distanceBetweenSandboxFiducials(top_left_id, top_right_id)
        length_bottom = self._cv_fiducial_distanceBetweenSandboxFiducials(bottom_left_id, bottom_right_id)
        length_left = self._cv_fiducial_distanceBetweenSandboxFiducials(top_left_id, bottom_left_id)
        length_right = self._cv_fiducial_distanceBetweenSandboxFiducials(top_right_id, bottom_right_id)

        debugPrint("Top: " + str(length_top))
        debugPrint("Bottom: " + str(length_bottom))
        debugPrint("Left: " + str(length_left))
        debugPrint("Right: " + str(length_right))

        aspectRatioTop = (length_top + length_bottom) / 2
        aspectRatioSide = (length_left + length_right) / 2
        
        #scale to something reasonable
        multiplier = 5000 / (aspectRatioTop + aspectRatioSide) / 2
        aspectRatioTop = int(aspectRatioTop * multiplier)
        aspectRatioSide = int(aspectRatioSide * multiplier)

        sandboxImage = self.cv_fiducial_flattenSandboxImage(image_frame, aspectRatioTop, aspectRatioSide)

        if constants.CV_DEBUG:
            cv.imshow("Sandbox Init Image", sandboxImage)
            cv.waitKey(0)

        return sandboxImage

        
    def _cv_fiducial_distanceBetweenSandboxFiducials(self, fiducialId1, fiducialId2):
        # calculate the distance between two fiducials
        x_distance = self.cv_fiducial_cornerMarkerDict[fiducialId1][7][0] - self.cv_fiducial_cornerMarkerDict[fiducialId2][7][0]
        y_distance = self.cv_fiducial_cornerMarkerDict[fiducialId1][7][1] - self.cv_fiducial_cornerMarkerDict[fiducialId2][7][1]
        z_distance = self.cv_fiducial_cornerMarkerDict[fiducialId1][7][2] - self.cv_fiducial_cornerMarkerDict[fiducialId2][7][2]

        return np.sqrt(x_distance**2 + y_distance**2 + z_distance**2)


    def _cv_fiducial_detectSandboxCorners(self, image_frame):
        if constants.CV_DEBUG:
            image_frame_annotated = image_frame.copy()

        arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        arucoParams = cv.aruco.DetectorParameters_create()
        corner_list, fiducial_ids, _ = cv.aruco.detectMarkers(image_frame, arucoDict, parameters=arucoParams)

        if len(corner_list) > 0:
            fiducial_ids = fiducial_ids.flatten()
            for (marker_corner, fiducial_id) in zip(corner_list, fiducial_ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corner_list = marker_corner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner_list
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                centerX = int((topLeft[0] + bottomRight[0]) / 2.0)
                centerY = int((topLeft[1] + bottomRight[1]) / 2.0)

                rvec = None
                tvec = None

                # reserve the extra processing for the corner fiducials
                if fiducial_id in constants.CORNER_FIDUCIALS:
                    # estimate the pose of the marker
                    rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(\
                        marker_corner, \
                        constants.FIDUCIAL_WIDTH_MM, \
                        constants.CAMERA_MATRIX, \
                        constants.DISTORTION_COEFFICIENTS)
                    
                    rvec = rvec.flatten()
                    tvec = tvec.flatten()
                    
                    debugPrint("Fiducial ID, tvec: " + str(fiducial_id) + ", " + ", " + str(tvec))

                    if constants.CV_DEBUG:
                        # cv.aruco.drawDetectedMarkers(image_frame_annotated, corner_list)
                        cv.drawFrameAxes(image_frame_annotated, constants.CAMERA_MATRIX, constants.DISTORTION_COEFFICIENTS, rvec, tvec, 20)
                
                self.cv_fiducial_cornerMarkerDict[fiducial_id] = (centerX, centerY, topLeft, topRight, bottomRight, bottomLeft, rvec, tvec)

        else:
            return False

        if constants.CV_DEBUG:
            cv.imshow("Corner Pose", image_frame_annotated)
            cv.waitKey(0)

        return True
        

    ''' 
    This function returns the fiducial locations in the image.
    '''
    def cv_fiducial_generatePalletLocations(self, sandbox_image):

        arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        arucoParams = cv.aruco.DetectorParameters_create()
        corner_list, fiducial_ids, _ = cv.aruco.detectMarkers(sandbox_image, arucoDict, parameters=arucoParams)

        if len(corner_list) > 0:
            fiducial_ids = fiducial_ids.flatten()
            for (marker_corner, fiducial_id) in zip(corner_list, fiducial_ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corner_list = marker_corner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner_list
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                centerX = int((topLeft[0] + bottomRight[0]) / 2.0)
                centerY = int((topLeft[1] + bottomRight[1]) / 2.0)
         
                self.cv_fiducial_markerDict[fiducial_id] = (centerX, centerY, topLeft, topRight, bottomRight, bottomLeft)

    '''
    This function finds the correct fiducial locations and returns them in the correct order.

    2 of the corners maximize and minimize their x,y values respectively.
    The other 2 corners only have 1 value that is either the max or min.
    This simple logic is used to sort the fiducials into the correct order.
    '''
    def _cv_fiducial_findCornerFiducialOrdering(self):
        cornerFiducialIDs = constants.CORNER_FIDUCIALS
        unsortedCornerFiducialCenters = []
        unsortedCornerFiducialCenterIDs = []
        
        # Get information about the corner fiducials.
        for fiducialID in cornerFiducialIDs:
            if fiducialID in self.cv_fiducial_cornerMarkerDict.keys():
                unsortedCornerFiducialCenters.append(self.cv_fiducial_cornerMarkerDict[fiducialID][0:2])
                unsortedCornerFiducialCenterIDs.append(fiducialID)
            else:
                print(self.cv_fiducial_cornerMarkerDict)
                constants.blockingError("Error, Sandbox corner fiducial not found." + str(fiducialID))
        
        # Get the min and max fiducials since those are easy
        top_left = min(unsortedCornerFiducialCenters, key=lambda x: x[0] + x[1])
        top_right = max(unsortedCornerFiducialCenters, key=lambda x: x[0] - x[1])
        bottom_right = max(unsortedCornerFiducialCenters, key=lambda x: x[0] + x[1])
        bottom_left = min(unsortedCornerFiducialCenters, key=lambda x: x[0] - x[1])

        # get the appropriate fiducial ID for each corner
        top_left_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(top_left)]
        top_right_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(top_right)]
        bottom_right_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(bottom_right)]
        bottom_left_id = unsortedCornerFiducialCenterIDs[unsortedCornerFiducialCenters.index(bottom_left)]

        return top_left_id, top_right_id, bottom_left_id, bottom_right_id

    def cv_fiducial_findSandboxSize(self, sandbox_image):
        # calculate the number of pixels per fiducial

        arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        arucoParams = cv.aruco.DetectorParameters_create()
        corner_list, fiducial_ids, _ = cv.aruco.detectMarkers(sandbox_image, arucoDict, parameters=arucoParams)
        sum_of_fiducial_pixels = 0
        num_fiducials_incr = 0
         

        if len(corner_list) > 0:
            fiducial_ids = fiducial_ids.flatten()
            for (marker_corner, fiducial_id) in zip(corner_list, fiducial_ids):
                if fiducial_id in constants.CORNER_FIDUCIALS:

                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corner_list = marker_corner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corner_list
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    centerX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    centerY = int((topLeft[1] + bottomRight[1]) / 2.0)

                    # top left to top right x distance
                    sum_of_fiducial_pixels += abs(topLeft[0] - topRight[0])
                    # top left to bottom left y distance
                    sum_of_fiducial_pixels += abs(topLeft[1] - bottomLeft[1])
                    num_fiducials_incr += 2

                    self.cv_fiducial_warpedCornerMarkerDict[fiducial_id] = (centerX, centerY, topLeft, topRight, bottomRight, bottomLeft)

        
                    
        average_fiducial_pixels = sum_of_fiducial_pixels / num_fiducials_incr
        self.mm_per_pixel = constants.FIDUCIAL_WIDTH_MM/average_fiducial_pixels
        
        # calculate the sandbox size in mm
        top_left_id, top_right_id, bottom_left_id, bottom_right_id = self._cv_fiducial_findCornerFiducialOrdering()
        print("keys: ", self.cv_fiducial_warpedCornerMarkerDict.keys())
        self.sandbox_width_mm = self.cv_fiducial_warpedCornerMarkerDict[top_right_id][2][0] - self.cv_fiducial_warpedCornerMarkerDict[top_left_id][3][0] * self.mm_per_pixel
        self.sandbox_height_mm = self.cv_fiducial_warpedCornerMarkerDict[bottom_left_id][2][1] - self.cv_fiducial_warpedCornerMarkerDict[top_left_id][5][1] * self.mm_per_pixel
        debugPrint("Sandbox size (w,h): " + str(self.sandbox_width_mm) + " x " + str(self.sandbox_height_mm) + " mm")


    def cv_fiducial_flattenSandboxImage(self, image_frame, height = 0, width = 0):
        
        # override after initial sandbox size is found
        if (self.sandbox_width_mm != None and self.sandbox_height_mm != None):
            height = int(self.sandbox_height_mm)
            width = int(self.sandbox_width_mm)

        # add a bit of a buffer to the sandbox size
        buffer_pixels_height = int(height * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2)
        buffer_pixels_width = int(width * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2)

        destination_corners = np.array([
            [buffer_pixels_height, buffer_pixels_width],
            [height - 1 + buffer_pixels_height, buffer_pixels_width],
            [height - 1 + buffer_pixels_height, width - 1 + buffer_pixels_width],
            [buffer_pixels_height, width - 1 + buffer_pixels_width]], dtype = "float32")
        
        # get fiducials in the right order
        top_left_id, top_right_id, bottom_left_id, bottom_right_id = self._cv_fiducial_findCornerFiducialOrdering()

        fiducial_corners = np.array([
            self.cv_fiducial_cornerMarkerDict[top_left_id][2],
            self.cv_fiducial_cornerMarkerDict[top_right_id][3],
            self.cv_fiducial_cornerMarkerDict[bottom_right_id][4],
            self.cv_fiducial_cornerMarkerDict[bottom_left_id][5]], dtype = "float32")

        M = cv.getPerspectiveTransform(fiducial_corners, destination_corners)
        sandbox_image = cv.warpPerspective(image_frame, M, (height + (buffer_pixels_height * 2), width + (buffer_pixels_width * 2)))

        return sandbox_image


    def cv_fiducial_getPalletPositions(self):
        palletFiducialIDs = constants.PALLET_FIDUCIALS
        foundPalletFiducialIDs = {}

        # Find and pack the found pallets from the possible pallet fiducials
        for fiducialID in palletFiducialIDs:
            if fiducialID in self.cv_fiducial_markerDict.keys():
                foundPalletFiducialIDs[fiducialID] = self.cv_fiducial_markerDict[fiducialID][0:2]
        
        return foundPalletFiducialIDs