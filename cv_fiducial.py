'''
'''
from dis import code_info
from email.mime import image
from turtle import position
from unicodedata import name
import cv2 as cv
import numpy as np
import os
import time 
import constants

# cap = cv.VideoCapture(0)
debug = True

def get_webcam_image():
    '''
    This function returns an image from the webcam.
    '''
    ret, frame = cap.read()
    # cap.release()
    return frame

class CV_Fiducial:
    def __init__(self):
        self.cv_fiducial_markerDict = {}
        self.mm_per_pixel = None
        self.arena_width_mm = None
        self.arena_height_mm = None

    ''' 
    This function returns the fiducial locations in the image.
    Return: [Top left, Top right, Bottom left, Bottom right]
    '''
    def cv_fiducial_generateFiducialLocations(self, image_frame):

        arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        arucoParams = cv.aruco.DetectorParameters_create()
        corner_list, fiducial_ids, _ = cv.aruco.detectMarkers(image_frame, arucoDict, parameters=arucoParams)
        if len(corner_list) >= 4:
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
    def _cv_fiducial_findCornerFiducials(self):
        cornerFiducialIDs = constants.CORNER_FIDUCIALS
        unsortedCornerFiducialCenters = []
        unsortedCornerFiducialCenterIDs = []
        
        # Get information about the corner fiducials.
        for fiducialID in cornerFiducialIDs:
            if fiducialID in self.cv_fiducial_markerDict.keys():
                unsortedCornerFiducialCenters.append(self.cv_fiducial_markerDict[fiducialID][0:2])
                unsortedCornerFiducialCenterIDs.append(fiducialID)
            else:
                print(self.cv_fiducial_markerDict)
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

    def cv_fiducial_findPixelPitch(self):
        # calculate the number of pixels per fiducial
        
        sum_of_fiducial_pixels = 0
        for fiducial_id in self.cv_fiducial_markerDict.keys():
            # top left to top right x distance
            print(self.cv_fiducial_markerDict[fiducial_id][3][0] - self.cv_fiducial_markerDict[fiducial_id][2][0])
            sum_of_fiducial_pixels += self.cv_fiducial_markerDict[fiducial_id][3][0] - self.cv_fiducial_markerDict[fiducial_id][2][0]
            # top left to bottom left y distance
            print(self.cv_fiducial_markerDict[fiducial_id][5][1] - self.cv_fiducial_markerDict[fiducial_id][2][1])
            sum_of_fiducial_pixels += self.cv_fiducial_markerDict[fiducial_id][4][1] - self.cv_fiducial_markerDict[fiducial_id][2][1]

        average_fiducial_pixels = sum_of_fiducial_pixels / (len(self.cv_fiducial_markerDict.keys()) * 2)
        self.mm_per_pixel = constants.FIDUCIAL_WIDTH_MM/average_fiducial_pixels
        return self.mm_per_pixel
    
    def cv_fiducial_findSandboxSize(self):
        top_left_id, top_right_id, bottom_left_id, bottom_right_id = self._cv_fiducial_findCornerFiducials()
        # calculate the arena size in mm
        self.arena_width_mm = self.cv_fiducial_markerDict[top_left_id][3][0] - self.cv_fiducial_markerDict[top_right_id][2][0] * self.mm_per_pixel
        self.arena_height_mm = self.cv_fiducial_markerDict[top_left_id][4][1] - self.cv_fiducial_markerDict[bottom_left_id][2][1] * self.mm_per_pixel
        

    def cv_fiducial_flattenSandboxImage(self, image_frame):
            maxWidth = np.shape(image_frame)[1]
            maxHeight = np.shape(image_frame)[0]

            destination_corners = np.array([
                [0, 0],
                [maxWidth - 1, 0],
                [maxWidth - 1, maxHeight - 1],
                [0, maxHeight - 1]], dtype = "float32")
            
            # get fiducials in the right order
            top_left_id, top_right_id, bottom_left_id, bottom_right_id = self._cv_fiducial_findCornerFiducials()

            fiducial_corners = np.array([
                self.cv_fiducial_markerDict[top_left_id][2],
                self.cv_fiducial_markerDict[top_right_id][3],
                self.cv_fiducial_markerDict[bottom_right_id][4],
                self.cv_fiducial_markerDict[bottom_left_id][5]], dtype = "float32")

            M = cv.getPerspectiveTransform(fiducial_corners, destination_corners)
            warped_image = cv.warpPerspective(image_frame, M, (maxWidth, maxHeight))

            return warped_image


    def cv_fiducial_getPalletPositions():
        pass