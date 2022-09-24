'''
This file contains high level helper functions for 
the kumpooterBision part of the project.
'''

import cv2 as cv
import numpy as np
import os

print("Test")


def get_image():
    '''
    This function returns an image from the webcam.
    '''
    cap = cv.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()
    return frame

def get_test_frame():
    '''
    This function returns the test.img file.
    '''
    path = os.getcwd() + "/test.JPG"
    print(path)
    return cv.imread(path, cv.COLOR_BGR2HSV)

def get_robot_points():
    # These are hardcoded robot points as follows
    #             2               #
    #          1     3            #
    #                             #
    #                             #
    #                             #
    #    0                 4      #
    #                             #
    #                             #

    # point 0 is at 0,0
    # Points are (x,y) in *CARTESIAN* coordinates, not stupid KumpooterBision coordinates
    # Points are in mm
    LED_0 = (0.0,0.0)
    LED_1 = (23.0, 67.16)
    LED_2 = (34.0, 84.85)
    LED_3 = (45.5, 67.16)
    LED_4 = (67.5,0)
    points = [LED_0, LED_1, LED_2, LED_3, LED_4]
    return points

def get_robot_centering_translation_matrix(): 
    '''
    This function returns the translation matrix to center the robot.
    This basically translate the robot from the bottom left corner to the center.
    The robot center as defined as the point between the 3 extremety LEDs.
    '''

def extract_LED_positions(img, color_filter_lower_bound, color_filter_upper_bound):
    '''
    This function returns the LED positions in the image.
    '''
    mask = cv.inRange(img, color_filter_lower_bound, color_filter_upper_bound)
    img_filtered = cv.bitwise_and(img, img, mask=mask)

    # erode and dilate to remove noise
    kernel = np.ones((5,5),np.uint8)
    img_filtered = cv.erode(img_filtered,kernel,iterations = 1)
    img_filtered = cv.dilate(img_filtered,kernel,iterations = 1)

    # Convert to grayscale and find moments on the thresholded img
    gray_img_filtered = cv.cvtColor(img_filtered, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray_img_filtered, 127, 255, 0)
    contours,hierarchy = cv.findContours(thresh, 1, 2)

    print(len(contours))
    a = np.empty([len(contours), 1])
    center = np.empty([len(contours), 2])
    for i in range(0, len(contours)):
        Mi = cv.moments(contours[i])
        if Mi['m00'] != 0:
            center[i,0]= Mi['m10']/Mi['m00'] # x coordinate
            center[i, 1]= Mi['m01']/Mi['m00'] # y coordinate
            a[i] = cv.contourArea(contours[i])

    # draw circles on the image to show the LEDs
    for i in range(0, len(contours)):
        cv.circle(img_filtered, (int(center[i, 0]), int(center[i, 1])), 2, (0, 0, 255), -1)

    return img_filtered, center, len(contours)

def apply_transformation_matrix(img, transformation_matrix):
    '''
    This function applies the transformation matrix to the points with cv2.
    '''

    # get img size
    h, w = img.shape[:2]
    # transformation_matrix = np.float32([[0.2, 0, 0], [0, 0.2, 0]]) #debugging only
    # print("Transformation matrix", transformation_matrix)
    # transform
    return cv.warpAffine(img, transformation_matrix, (w, h))

'''
Calculate robot position from LED positions based on estimateAffinePartial2D
'''

def get_robot_affine_transform(robot_template_points, robot_cv_points):
    # print("robot_template_points", robot_template_points, np.shape(robot_template_points))
    # print("robot_cv_points", robot_cv_points, np.shape(robot_cv_points))
    return cv.estimateAffinePartial2D(robot_cv_points, robot_template_points)


''' 
Main function
'''
if __name__ == "__main__":
    # cv.imshow("Test", get_test_frame())
    # cv.waitKey(0)

    # debugging HSV values
    green_rgb = np.uint8([[[0, 255, 0]]])
    green_hsv = cv.cvtColor(green_rgb, cv.COLOR_RGB2HSV)[0][0]
    # print(green_hsv)
    h_buffer = 100
    s_buffer = 100
    lower = (int(green_hsv[0]) - h_buffer, int(green_hsv[1]) - s_buffer , 0)
    upper = (int(green_hsv[0]) + h_buffer, int(green_hsv[1]) + s_buffer , 255)

    lower = (0, 0, 250)
    upper = (255, 255, 255)

    masked_img, center, num_of_centers = extract_LED_positions(get_test_frame(), lower, upper)

    # cv.imshow("Extracted", masked_img)
    # cv.waitKey(0)

    transform, _ = get_robot_affine_transform(np.array(get_robot_points()), np.array(center))
    print(transform)

    test_transformed_image = apply_transformation_matrix(get_test_frame(), transform)

    cv.imshow("Test", test_transformed_image)
    cv.waitKey(0)
    