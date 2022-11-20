'''
This file contains high level helper functions for 
the Computer Vision part of the project.

Be very careful with the cordinate system since computer vision corrdinate system 
is different from the robot coordinate system.
'''

from dis import code_info
from distutils.log import debug
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
        self.visualizerField = None

        self.latestImageRaw = None
        self.latestSandboxImage = None
        self.latestRobotPositions = None
        self.actualRobotPaths = None # eventually replace with a dict as follows {robotId: [(time, pose), (time, pose), ...]}

##############################################
#            Public Functions
##############################################

    def cv_GetRobotCounts(self):
        return self.robotCount

    def cv_runLocalizer(self):
        self.latestImageRaw = self._cv_CaptureImage()
        self.latestSandboxImage = self.cv_fiducial.cv_fiducial_flattenSandboxImage(self.latestImageRaw)

        if self.latestImageRaw is None:
            debugPrint("Computer Vision: No Image Captured")
            return

        if constants.CV_LOCALIZE_ROBOTS_FIDUCIALS == False:
            self._cv_GenerateRobotPositions(self.latestSandboxImage)
        
        # also generates robot positions
        self.cv_fiducial.cv_fiducial_generatePalletLocations(self.latestSandboxImage)


    def cv_GetRobotPositions(self):
        if constants.CV_LOCALIZE_ROBOTS_FIDUCIALS == False:
            return self.latestRobotPositions
        else:
            return self.cv_fiducial.cv_fiducial_getRobotPositions()

    def cv_GetPalletPositions(self):
        return self.cv_fiducial.cv_fiducial_getPalletPositions()

    def cv_GetSandboxSize(self):
        # get this from the cv_fiducial class
        return self.cv_fiducial.sandbox_width_mm, self.cv_fiducial.sandbox_height_mm

    def cv_GetGoalFiducials(self):
        return self.cv_fiducial.cv_fiducial_getGoalPositions()

    def cv_InitComputerVision(self):
        # Initialize the computer vision
        self.latestSandboxImage = self.cv_fiducial.cv_fiducial_setupSandbox(self._cv_CaptureImage())

        print("Computer Vision Field Ready")

        if constants.CV_LOCALIZE_ROBOTS_FIDUCIALS == False:
            # generate the robot masks dynamically
            self._cv_GenerateRobotMasks()
    
    def cv_visualize(self, robotPaths, targetPoses, robotRightSpeeds, robotLeftSpeeds, feedforward, feedback):
        # Goals:
        # 1. Place a vector at each robot's position
        # 2. Place a centroid at each pallet's position
        # 3. Visualize the current path (assume it's a dict with keys being robot ID)
        # 4. Visualize the actual path that the robot has taken 

        # if self.visualizerField is None:
        #     if self.latestSandboxImage is None:
        #         return # wait for a valid image to be captured
        self.visualizerField = self.latestSandboxImage.copy()

        robotPositions = None
        if constants.CV_LOCALIZE_ROBOTS_FIDUCIALS == False:
            robotPositions = self.latestRobotPositions
        else:
            robotPositions, _ = self.cv_fiducial.cv_fiducial_getRobotPositions()

        # 1. Place a vector at each robot's position
        for i in range(len(robotPositions)):
            robotPose = robotPositions[i]
            (robot_pos_x, robot_pos_y, robot_rotation_rad) = robotPose
            start_point = (int(robot_pos_x), int(robot_pos_y))
            end_point = (int(robot_pos_x + 100*np.cos(robot_rotation_rad)), int(robot_pos_y - 100*np.sin(robot_rotation_rad)))
            cv.arrowedLine(self.visualizerField, start_point, end_point, (255, 0, 0), 2)
        
            # # 5. Visualize the robot's indivigual speeds
            # # Left wheel position 5 cm to the left of the robot pose
            # # Right wheel position 5 cm to the right of the robot pose
            # mult = 10
            # leftMag = robotLeftSpeed * mult
            # rightMag = robotRightSpeed * mult
            # leftArrowStart = (int(robot_pos_x - 50*np.sin(robot_rotation_rad)), int(robot_pos_y - 50*np.cos(robot_rotation_rad)))
            # rightArrowStart = (int(robot_pos_x + 50*np.sin(robot_rotation_rad)), int(robot_pos_y + 50*np.cos(robot_rotation_rad)))
            # leftArrowEnd = (int(robot_pos_x - 50*np.sin(robot_rotation_rad) + leftMag * np.cos(robot_rotation_rad)), int(robot_pos_y - 50*np.cos(robot_rotation_rad) - leftMag*np.sin(robot_rotation_rad)))
            # rightArrowEnd = (int(robot_pos_x + 50*np.sin(robot_rotation_rad) + rightMag * np.cos(robot_rotation_rad)), int(robot_pos_y + 50*np.cos(robot_rotation_rad) - rightMag*np.sin(robot_rotation_rad)))

            # cv.arrowedLine(self.visualizerField, leftArrowStart, leftArrowEnd, (100, 0, 0), 2)
            # cv.arrowedLine(self.visualizerField, rightArrowStart, rightArrowEnd, (0, 0, 100), 2)

            # 6. Visualize the robot direction vector
            robotRightSpeed = robotRightSpeeds[i]
            robotLeftSpeed = robotLeftSpeeds[i]
            magnitude = (abs(robotRightSpeed) + abs(robotLeftSpeed)) * 10
            angle = ((robotRightSpeed - robotLeftSpeed) / constants.maxRobotSpeed) * np.pi # scale to pi radians
            angle += robot_rotation_rad
            start_point = (int(robot_pos_x), int(robot_pos_y))
            end_point = (int(robot_pos_x + magnitude * np.cos(angle)), int(robot_pos_y - magnitude * np.sin(angle)))
            cv.arrowedLine(self.visualizerField, start_point, end_point, (255, 255, 255), 2)




        
        # # 2. Place an arrow at each pallet's position 
        # palletPositions = self.cv_fiducial.cv_fiducial_getPalletPositions()
        # for palletPose in palletPositions:
        #     (pallet_pos_x, pallet_pos_y, pallet_rotation_rad) = palletPose
        #     start_point = (int(pallet_pos_x), int(pallet_pos_y))
        #     end_point = (int(pallet_pos_x + 100*np.cos(pallet_rotation_rad)), int(pallet_pos_y - 100*np.sin(pallet_rotation_rad)))
        #     cv.arrowedLine(self.visualizerField, start_point, end_point, (0, 0, 255), 2)
        
        # 3. Visualize the current path (assume it's a dict with keys being robot ID)
        for robotPath in robotPaths:
            for pose in robotPath:
                (robot_pos_x, robot_pos_y, robot_rotation_rad, time, tag) = pose
                start_point = (int(robot_pos_x), int(robot_pos_y))
                end_point = (int(robot_pos_x + 20*np.cos(robot_rotation_rad)), int(robot_pos_y - 20*np.sin(robot_rotation_rad)))
                cv.arrowedLine(self.visualizerField, start_point, end_point, (0, 255, 0), 2)

        # 4. Visualize the target pose
        for targetPose in targetPoses:
            (target_pos_x, target_pos_y, target_rotation_rad) = targetPose
            start_point = (int(target_pos_x), int(target_pos_y))
            end_point = (int(target_pos_x + 15*np.cos(target_rotation_rad)), int(target_pos_y - 15*np.sin(target_rotation_rad)))
            cv.arrowedLine(self.visualizerField, start_point, end_point, (28, 121, 225), 3)




                    
        if(constants.CV_VISUALIZE_PATH):
            pass

        if(constants.CV_VISUALIZE_ACTUAL_PATH):
            pass
        
        cv.imshow("Visualizer", self.visualizerField)
        cv.waitKey(1)
    def cv_getPalletPositionsOffset(self):
        # Generates a list of pallet positions with the offset applied
        palletPositions = self.cv_fiducial.cv_fiducial_getPalletPositions()
        palletPositionsModified = []
        for palletPose in palletPositions:
            (pallet_pos_x, pallet_pos_y, pallet_rotation_rad) = palletPose
            pallet_pos_x += np.cos(pallet_rotation_rad) * constants.CV_PALLET_OFFSET
            pallet_pos_y -= np.sin(pallet_rotation_rad) * constants.CV_PALLET_OFFSET
            palletPose = (pallet_pos_x, pallet_pos_y, pallet_rotation_rad)
            palletPositionsModified.append(palletPose)
        
        return palletPositionsModified
        

##############################################
#            Private Functions
##############################################

    def _cv_GenerateRobotPositions(self, latestSandboxImage):

        # for each led, figure out which robot it belongs to
        maskedImg, pointCenters, numOfCenters, robotLedValues  = self._cv_extractLEDpositions(latestSandboxImage)

        robotIdPixelDict = {}
        
        # TODO: could easily be vectorized
        for i in range(0, numOfCenters):
            robotId = self.ledNeigbors.predict([robotLedValues[i]])[0]
            if robotId in robotIdPixelDict.keys():
                robotIdPixelDict[robotId].append(pointCenters[i])
            else:
                robotIdPixelDict[robotId] = [pointCenters[i]]
            
        # now that we have an unsorted list of which leds are which, figure out the best fit 
        # robot transform for each robot
        robotPositions = {}
        for robotId in robotIdPixelDict.keys():
            transform, most_inliers, did_it_work = self._cv_getRobotAffineTransform(self._cv_getRobotPoints(), robotIdPixelDict[robotId])
            if did_it_work == False:
                return
            robot_rotation_deg, robot_rotation_rad, robot_pos_x, robot_pos_y = get_robot_coordinates_from_transformation_matrix(transform)
            robotPositions[robotId] = (robot_pos_x, robot_pos_y, robot_rotation_rad)
        
        self.latestRobotPositions = robotPositions

        return robotPositions

    def _cv_GenerateRobotMasks(self):
        '''
        Use K nearest neighbors to generate the robot masks
        '''
        latestImageWarped = self.cv_fiducial.cv_fiducial_flattenSandboxImage(self._cv_CaptureImage())

        if constants.CV_DEBUG:
            cv.imshow("Warped Image", latestImageWarped)
            cv.waitKey(0)

        maskedLedImage, robotLedCenters, numLeds, ledRGBvals = self._cv_extractLEDpositions(latestImageWarped)
        self.robotCount = round(numLeds/constants.CV_LEDS_PER_ROBOT)

        debugPrint(str("Robot's detected: " + str(self.robotCount)))
        debugPrint(ledRGBvals)

        # Use K nearest neighbors to generate the robot masks
        self.ledNeigbors = KMeans(n_clusters=self.robotCount).fit(ledRGBvals)

        if constants.CV_DEBUG:
            # make a 3d scatter plot of the robot leds with the led colors classified and led centroids
            y_kmeans = self.ledNeigbors.predict(ledRGBvals)
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(ledRGBvals[:,0], ledRGBvals[:,1], ledRGBvals[:,2], c=y_kmeans, cmap='rainbow')
            ax.scatter(self.ledNeigbors.cluster_centers_[:,0], self.ledNeigbors.cluster_centers_[:,1], self.ledNeigbors.cluster_centers_[:,2], c='black', s=200, alpha=0.5)
            plt.show()


    def _cv_CaptureImage(self):
        if constants.CV_USE_CAMERA == False:
            image = cv.imread(constants.CV_DEBUG_IMAGE_PATH)
            cv.imshow("Source Image", image)
            cv.waitKey(0)
            return image

        ret, frame = cap.read()
        while np.shape(frame) == ():
            ret, frame = cap.read()
            print("Waiting for camera to initialize...")
        if constants.CV_DEBUG:
            cv.imshow("Source Image", frame)
            cv.waitKey(0)
        return frame

    def _cv_extractLEDpositions(self, img):
        '''
        This function returns the LED positions in the image.
        '''
        
        #  #apply a varience filter to the image
        color_filter_lower_bound = constants.CV_ROBOT_VARIENCE_LOWER_BOUND
        color_filter_upper_bound = constants.CV_ROBOT_VARIENCE_UPPER_BOUND
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, color_filter_lower_bound, color_filter_upper_bound)
        img_filtered = cv.bitwise_and(img, img, mask=mask)

        if constants.CV_DEBUG:
            cv.imshow("LED varience masked image", img_filtered)
            cv.waitKey(0)

        # erode and dilate to remove noise
        kernel = np.ones((3,3),np.uint8)
        img_filtered = cv.erode(img_filtered,kernel,iterations = 1)
        img_filtered = cv.dilate(img_filtered,kernel,iterations = 3)

        if constants.CV_DEBUG:
            cv.imshow("LED varience mask refined image", img_filtered)
            cv.waitKey(0)

        
        # Find all circles that meet a certain size, and create a mask
        mask = np.zeros(img_filtered.shape[:2], np.uint8)
        img_filtered_gray = cv.cvtColor(img_filtered, cv.COLOR_BGR2GRAY)
        img_filtered_gray_text = img_filtered_gray.copy()
        circles = cv.HoughCircles(\
            img_filtered_gray, \
            cv.HOUGH_GRADIENT, \
            1, \
            minDist = constants.CV_PIXEL_DISTANCE_BETWEEN_LEDS, \
            param1 = constants.CV_CIRCLE_PARAM1, \
            param2 = constants.CV_CIRCLE_PARAM2, \
            minRadius = constants.CV_LED_MIN_CIRCLE, \
            maxRadius = constants.CV_LED_MAX_CIRCLE)
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                mask = cv.circle(mask, (x, y), int(r * constants.CV_CIRCLE_MASK_MULTIPLIER), (255, 255, 255), -1)
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                if constants.CV_DEBUG:
                    cv.circle(img_filtered_gray_text, (x, y), r, (0, 255, 0), 4)
                    cv.rectangle(img_filtered_gray_text, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                    cv.putText(img_filtered_gray_text, str(r), (x,y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv.LINE_AA)
            if constants.CV_DEBUG:
                # show the output image
                cv.imshow("circles found", img_filtered_gray_text)
                cv.waitKey(0)
        else: 
            debugPrint("No circles found")
        
        # Apply the mask to the image
        img_filtered_gray = cv.bitwise_and(img_filtered_gray, img_filtered_gray, mask=mask) 
        if constants.CV_DEBUG:
            cv.imshow("Masked Image with circles", img_filtered_gray)
            cv.waitKey(0)

        # Convert to grayscale and find moments on the thresholded img
        # gray_img_filtered = cv.cvtColor(img_filtered, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(img_filtered_gray, 50, 255, 0)

        if constants.CV_DEBUG:
            cv.imshow("LED varience mask refined threshold image", thresh)
            cv.waitKey(0)

        contours, hierarchy = cv.findContours(thresh, 1, 2)

        ledRGB = np.zeros((len(contours), 3))
        centers = np.empty([len(contours), 2])
        areas = np.empty([len(contours), 1])
        for i in range(0, len(contours)):
            Mi = cv.moments(contours[i])
            area = cv.contourArea(contours[i])
            # jank in range function
            if Mi['m00'] != 0 and (min(constants.CV_MIN_LED_AREA, constants.CV_MAX_LED_AREA) < area < max(constants.CV_MIN_LED_AREA, constants.CV_MAX_LED_AREA)):
                centers[i,0]= Mi['m10']/Mi['m00'] # x coordinate
                centers[i, 1]= Mi['m01']/Mi['m00'] # y coordinate

                # calculate the radius and rgb value of the led
                # currently brute forced to center pixel for speed and ease
                ledRGB[i] = img_filtered[int(centers[i, 1]), int(centers[i, 0])]
                areas[i] = area

                # Note: possible way to get the rgb value of the led: 
                #   Use average of the contourMaskedImg
                #   Use the center of the contourMaskedImg
                #   Use kmeans lol
                # contourMask = np.zeros(img.shape, np.uint8)
                # cv.drawContours(contourMask, contours, i, (255,255,255), -1)
                # contourMaskedImg = cv.bitwise_and(img, contourMask)


        if(constants.CV_DEBUG):
            for i in range(0, len(contours)):
                cv.circle(img_filtered_gray, (int(centers[i, 0]), int(centers[i, 1])), 2, (0, 0, 255), -1)
                cv.putText(img_filtered_gray, str(ledRGB[i]), (int(centers[i, 0]), int(centers[i, 1])), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)
                # cv.putText(img_filtered_gray, str(areas[i]), (int(centers[i, 0]), int(centers[i, 1])), cv.FONT_HERSHEY_SIMPLEX,q 1, (255, 255, 255), 1, cv.LINE_AA) 
            cv.imshow('countours', img_filtered_gray)
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
        from itertools import permutations, combinations

        # TODO: setify this for speed
        permusToTry = list(permutations(robot_cv_points, min(constants.CV_LEDS_PER_ROBOT, len(robot_cv_points)))) 
        permusToMatch = list(combinations(robot_template_points, min(constants.CV_LEDS_PER_ROBOT, len(robot_cv_points))))
        # debugPrint("Permutations: " + str(permus))
        # debugPrint("cv points: "  + str(robot_cv_points))
        # debugPrint("template points: " + str(robot_template_points))

        best_transform = None
        most_inliers = -1
        workedAtLeastOnce = False


        for permToTry in permusToTry:
            for permToMatch in permusToMatch:
                try:
                    transform, inliers = cv.estimateAffinePartial2D(np.array(permToTry), np.array(permToMatch)) #todo: there are other params like condience and refine iters that can make this better
                    if np.count_nonzero(inliers) > most_inliers:
                        best_transform = transform
                        most_inliers = np.count_nonzero(inliers)
                        workedAtLeastOnce = True
                    
                        position_x_offset = 34.0 # this is the offset from the center of the robot to led0
                        position_y_offset = 20.0 # this is the offset from the center of the robot to led0

                        best_transform[0,2] = best_transform[0,2] - position_x_offset
                        best_transform[1,2] = best_transform[1,2] - position_y_offset
                except Exception as e:
                    print("Error in _cv_getrobotaffinetransform")
                    print("permToTry: " + str(permToTry))
                    print("permToMatch: " + str(permToMatch))
                    print("Exception: " + str(e))
                    print("\n")

        return best_transform, most_inliers, workedAtLeastOnce
                
                


        # try:
        #     for p in permus:
        #         transform, inliers = cv.estimateaffinepartial2d(np.array(p), robot_template_points) #todo: there are other params like condience and refine iters that can make this better
        #         if np.count_nonzero(inliers) > most_inliers:
        #             best_transform = transform
        #             most_inliers = np.count_nonzero(inliers)

        #     position_x_offset = 34.0 # this is the offset from the center of the robot to led0
        #     position_y_offset = 20.0 # this is the offset from the center of the robot to led0

        #     best_transform[0,2] = best_transform[0,2] - position_x_offset
        #     best_transform[1,2] = best_transform[1,2] - position_y_offset

        #     return best_transform, most_inliers, true
        # except:
        #     debugprint("error in _cv_getrobotaffinetransform")
        #     return none, none, false



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
    debugPrint("rotation_degrees: " + str(rotation_degrees))
    debugPrint("rotation_radians: " + str(rotation_radians))
    debugPrint("position_x: " + str(position_x))
    debugPrint("position_y: " + str(position_y))

    return rotation_degrees, rotation_radians, position_x, position_y
