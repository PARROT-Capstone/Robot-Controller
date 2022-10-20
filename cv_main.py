'''
This file contains high level helper functions for 
the Computer Vision part of the project.

Be very careful with the cordinate system since computer vision corrdinate system 
is different from the robot coordinate system.
'''

from dis import code_info
from turtle import position
import cv2 as cv
import numpy as np
import time 
from cv_fiducial import CV_Fiducial
import constants
from constants import debugPrint
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import pandas as pd
    
cap = cv.VideoCapture(constants.WEBCAM_ID)

class CV:
    def __init__(self):
        self.cv_fiducial = CV_Fiducial()
        self.robotCount = None
        self.robotRGBMasks = None

##############################################
#            Public Functions
##############################################

    def cv_GetRobotCounts(self):
        return self.robotCount

    def cv_GetRobotPositions(self):

        latestImageRaw = self._cv_CaptureImage()
        latestImageWarped = self.cv_fiducial.cv_fiducial_flattenSandboxImage(latestImageRaw)

        # for each led, figure out which robot it belongs to
        maskedImg, pointCenters, numOfCenters, robotLedValues  = self._cv_extractLEDpositions(latestImageWarped)

        robotIdPixelDict = {}
        for i in range(0, numOfCenters):
            robotId = self.ledNeigbors.fit(robotLedValues[i])
            if robotId in robotIdPixelDict.keys():
                robotIdPixelDict[robotId].append(pointCenters[i])
            else:
                robotIdPixelDict[robotId] = [pointCenters[i]]
            
        # now that we have an unsorted list of which leds are which, figure out the best fit 
        # robot transform for each robot
        robot_positions = {}
        for robotId in robotIdPixelDict.keys():
            transform, most_inliers, did_it_work = self._cv_getRobotAffineTransform(robotIdPixelDict[robotId])
            robot_rotation_deg, robot_rotation_rad, robot_pos_x, robot_pos_y = get_robot_coordinates_from_transformation_matrix(transform)
            robot_positions[robotId] = (robot_pos_x, robot_pos_y, robot_rotation_rad)
        
        return robot_positions

    def cv_GetPalletPositions(self):
        # Loop thourgh pallet fiducials and get their positions
        pass

    def cv_GetSandboxSize(self):
        # get this from the cv_fiducial class
        return self.cv_fiducial.arena_width_mm, self.cv_fiducial.arena_height_mm

    def cv_GetSandboxGoals(self):
        # get this from the cv_fiducial class
        pass

    def cv_InitComputerVision(self):
        # Initialize the computer vision
        # wait till at least 4 fiducials are found (corner fiducials)
        while(len(self.cv_fiducial.cv_fiducial_markerDict.keys()) < 4):
            self.cv_fiducial.cv_fiducial_generateFiducialLocations(self._cv_CaptureImage())
            print("Computer Vision: Waiting for sandbox fiducial Markers")
            time.sleep(1)

        self.cv_fiducial.cv_fiducial_findPixelPitch()
        self.cv_fiducial.cv_fiducial_findSandboxSize()
        print("Computer Vision Field Ready")
        print("Sandbox Size: ", self.cv_fiducial.arena_width_mm, self.cv_fiducial.arena_height_mm)

        # generate the robot masks dynamically
        self._cv_GenerateRobotMasks()

##############################################
#            Private Functions
##############################################
    def _cv_GenerateRobotMasks(self):
        '''
        Use K nearest neighbors to generate the robot masks
        '''
        maskedLedImage, robotLedCenters, numLeds, ledRGBvals = self._cv_extractLEDpositions(self._cv_CaptureImage())
        self.robotCount = round(numLeds/constants.CV_LEDS_PER_ROBOT)

        debugPrint(str("Robot's detected: " + str(self.robotCount)))
        debugPrint(ledRGBvals)

        # Use K nearest neighbors to generate the robot masks
        self.ledNeigbors = KMeans(n_clusters=self.robotCount).fit(ledRGBvals)
        self.ledNeigbors = KMeans(n_clusters=4).fit(ledRGBvals)

        if constants.CV_DEBUG:
            # make a 3d scatter plot of the robot leds with the led colors classified and led centroids
            y_kmeans = self.ledNeigbors.predict(ledRGBvals)
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(ledRGBvals[:,0], ledRGBvals[:,1], ledRGBvals[:,2], c=y_kmeans, cmap='rainbow')
            ax.scatter(self.ledNeigbors.cluster_centers_[:,0], self.ledNeigbors.cluster_centers_[:,1], self.ledNeigbors.cluster_centers_[:,2], c='black', s=200, alpha=0.5)
            plt.show()


    def _cv_CaptureImage(self):
        if constants.CV_DEBUG:
            image = cv.imread(constants.CV_DEBUG_IMAGE_PATH)
            cv.imshow("Source Image", image)
            cv.waitKey(0)
            return image
        ret, frame = cap.read()
        return frame

    def _cv_extractLEDpositions(self, img):
        '''
        This function returns the LED positions in the image.
        '''
        color_filter_lower_bound = constants.CV_ROBOT_VARIENCE_LOWER_BOUND
        color_filter_upper_bound = constants.CV_ROBOT_VARIENCE_UPPER_BOUND

        # convert to HSV
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, color_filter_lower_bound, color_filter_upper_bound)
        img_filtered = cv.bitwise_and(img, img, mask=mask)

        if constants.CV_DEBUG:
            cv.imshow("LED varience masked image", img_filtered)
            cv.waitKey(0)

        # erode and dilate to remove noise
        kernel = np.ones((5,5),np.uint8)
        img_filtered = cv.erode(img_filtered,kernel,iterations = 1)
        img_filtered = cv.dilate(img_filtered,kernel,iterations = 3)

        if constants.CV_DEBUG:
            cv.imshow("LED varience mask refined image", img_filtered)
            cv.waitKey(0)

        # Convert to grayscale and find moments on the thresholded img
        gray_img_filtered = cv.cvtColor(img_filtered, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(gray_img_filtered, 127, 255, 0)

        if constants.CV_DEBUG:
            cv.imshow("LED varience mask refined threshold image", thresh)
            cv.waitKey(0)

        contours, hierarchy = cv.findContours(thresh, 1, 2)
        
        #TODO: one can filter out the countours that are not the right size here


        ledRGB = np.zeros((len(contours), 3))
        centers = np.empty([len(contours), 2])
        for i in range(0, len(contours)):
            Mi = cv.moments(contours[i])
            if Mi['m00'] != 0:
                centers[i,0]= Mi['m10']/Mi['m00'] # x coordinate
                centers[i, 1]= Mi['m01']/Mi['m00'] # y coordinate

                # calculate the radius and rgb value of the led
                # currently brute forced to center pixel for speed and ease
                ledRGB[i] = img_filtered[int(centers[i, 1]), int(centers[i, 0])]

                # Note: possible way to get the rgb value of the led: 
                #   Use average of the contourMaskedImg
                #   Use the center of the contourMaskedImg
                #   Use kmeans lol
                # contourMask = np.zeros(img.shape, np.uint8)
                # cv.drawContours(contourMask, contours, i, (255,255,255), -1)
                # contourMaskedImg = cv.bitwise_and(img, contourMask)


        if(constants.CV_DEBUG):
            for i in range(0, len(contours)):
                cv.circle(img_filtered, (int(centers[i, 0]), int(centers[i, 1])), 2, (0, 0, 255), -1)
                cv.putText(img_filtered, str(ledRGB[i]), (int(centers[i, 0]), int(centers[i, 1])), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)
            cv.imshow('countours', img_filtered)
            cv.waitKey(0)

        return img_filtered, centers, len(contours), ledRGB

    def _cv_getRobotPoints(self):
        # These are hardcoded robot points as follows
        # -----> +x                    #
        # |   0                 4      #
        # |                            #
        # |                            #
        # v                            #
        # +y        1     3            #
        #              2               #
        #                              #
        #                              #

        # point 0 is at 0,0
        # Points are (x,y) in Computer Vision Coordinates
        # Points are in mm
        LED_0 = (0.0,0.0)
        LED_1 = (23.0, 67.16)
        LED_2 = (34.0, 84.85)
        LED_3 = (45.5, 67.16)
        LED_4 = (67.5,0)
        points = [LED_0, LED_1, LED_2, LED_3, LED_4]
        return points
    
    '''
    Calculate robot position from LED positions based on estimateAffinePartial2D
    '''
    def _cv_getRobotAffineTransform(self, robot_template_points, robot_cv_points):
        robot_template_points = np.array(self._cv_getRobotPoints())

        # Try all transform combinations since there's an ordering invariant and the order of the points is not known
        from itertools import permutations
        permus = list(permutations(robot_cv_points)) # TODO: setify this for speed

        best_transform = None
        most_inliers = -1

        try:
            for p in permus:
                transform, inliers = cv.estimateAffinePartial2D(np.array(p), robot_template_points) #TODO: there are other params like condience and refine iters that can make this better
                if np.count_nonzero(inliers) > most_inliers:
                    best_transform = transform
                    most_inliers = np.count_nonzero(inliers)

            position_x_offset = 34.0 # this is the offset from the center of the robot to LED0
            position_y_offset = 20.0 # this is the offset from the center of the robot to LED0

            best_transform[0,2] = best_transform[0,2] - position_x_offset
            best_transform[1,2] = best_transform[1,2] - position_y_offset

            return best_transform, most_inliers, True
        except:
            return None, None, False

        # print("Best transform", best_transform)
        # print("Most inliers", most_inliers)







def apply_transformation_matrix(img, transformation_matrix):
    '''
    This function applies the transformation matrix to the points with cv2.
    '''
    # get img size
    h, w = img.shape[:2]
    return cv.warpAffine(img, transformation_matrix, (w, h))



def get_robot_coordinates_from_transformation_matrix(transformation_matrix):
    '''
    This function returns the robot coordinates from the transformation matrix.
    '''
    # reference material
    # https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html#scipy.spatial.transform.Rotation
    # https://gamedev.stackexchange.com/questions/50963/how-to-extract-euler-angles-from-transformation-matrix

    rotation_degrees = np.arctan2(transformation_matrix[1,0], transformation_matrix[0,0]) * 180 / np.pi
    rotation_radians = np.arctan2(transformation_matrix[1,0], transformation_matrix[0,0])
    position_x = transformation_matrix[0,2]
    position_y = transformation_matrix[1,2]

    return rotation_degrees, rotation_radians, position_x, position_y

# ''' 
# Main function
# '''
# # if __name__ == "__main__":
# def controller_demo_cv():
#     debug = False # use static image

#     pre_image_time = time.time()
#     if debug:
#         image_frame = get_test_frame()

#     else:
#         image_frame = get_webcam_image()
#     post_image_time = time.time()

#     # debugging HSV values for the test_frame
#     green_rgb = np.uint8([[[0, 255, 0]]])
#     green_hsv = cv.cvtColor(green_rgb, cv.COLOR_RGB2HSV)[0][0]
#     blue_rgb = np.uint8([[[0, 0, 255]]])
#     blue_hsv = cv.cvtColor(green_rgb, cv.COLOR_RGB2HSV)[0][0]
#     # print(green_hsv)
#     h_buffer = 50
#     s_buffer = 10
#     lower = (int(green_hsv[0]) - h_buffer, int(green_hsv[1]) - s_buffer , 0)
#     upper = (int(green_hsv[0]) + h_buffer, int(green_hsv[1]) + s_buffer , 255)
#     lower = (int(blue_hsv[0]) - h_buffer, int(blue_hsv[1]) - s_buffer , 0)
#     upper = (int(blue_hsv[0]) + h_buffer, int(blue_hsv[1]) + s_buffer , 255)

#     # HSV is being fucky due to blown out highlights, so hardcode values for testing
#     # lower = (0, 0, 253)
#     lower = (0, 0, 250)
#     upper = (255, 255, 255)

#     algo_1 = time.time()
#     masked_img, centers, num_of_centers = extract_LED_positions(image_frame, lower, upper)
#     # cv.imshow("LED masked image", masked_img)
#     # cv.waitKey(0)
#     # cv.imshow("og image", image_frame)
#     # cv.waitKey(0)
#     # print("Number of LEDs found", num_of_centers)
#     # print("LED positions", centers)
#     algo_1_end = time.time()
#     transform, most_inliers, did_it_work = get_robot_affine_transform(np.array(get_robot_points()), centers)
#     if(did_it_work == False):
#         # print("FUCKED CV")
#         return None, None, None
#     algo_2_end = time.time()
#     robot_rotation_deg, robot_rotation_rad, robot_pos_x, robot_pos_y = get_robot_coordinates_from_transformation_matrix(transform)
#     algo_3_end = time.time()

#     # print("Number of LEDs found", num_of_centers)
#     # print("Robot rotation (deg)", robot_rotation_deg)
#     # print("Robot position x", robot_pos_x)
#     # print("Robot position y", robot_pos_y)

#     test_transformed_image = apply_transformation_matrix(image_frame, transform)
#     algo_4_end = time.time()

#     # print("Time to get image", post_image_time - pre_image_time)
#     # print("Time to get LED positions", algo_1_end - algo_1)
#     # print("Time to get robot transform", algo_2_end - algo_1_end)
#     # print("Time to get robot coordinates", algo_3_end - algo_2_end)
#     # print("Time to apply transform", algo_4_end - algo_3_end)
#     # print("total time", algo_4_end - pre_image_time)

#     # print("Frame rate: " + str(int(1/(algo_4_end - pre_image_time))) + " Pose: " + str(robot_pos_x) + ", " + str(robot_pos_y) + ", " + str(robot_rotation_deg))
#     # cv.imshow("original image", image_frame)
#     # cv.waitKey(0)
#     # cv.imshow("LED masked image", masked_img)
#     # cv.waitKey(0)
#     # cv.imshow("transformed image", test_transformed_image)
#     # cv.waitKey(0)

#     # break

#     return robot_pos_x, robot_pos_y, robot_rotation_rad

    