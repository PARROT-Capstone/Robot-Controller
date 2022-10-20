'''
This module is copy-pasted from the following link https://www.folkstalk.com/2022/09/getting-pixels-with-mouse-click-with-code-examples.html
The only change is reading an image from a camera instead of a file and printing the RGB value of the pixels
'''
# importing the module
import cv2
import numpy as np
import time

# function to display the coordinates of
# of the points clicked on the image
def click_event(event, x, y, flags, params):
 
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(x) + ',' +
                    str(y), (x,y), font,
                    1, (255, 0, 0), 2)
        cv2.imshow('image', img)
 
    # checking for right mouse clicks    
    if event==cv2.EVENT_RBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)

        # print the RGB value of the pixel
        print(img[y, x]) #(B, G, R)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        cv2.putText(img, str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x,y), font, 1,
                    (255, 255, 0), 2)
        cv2.imshow('image', img)
 
# driver function
if __name__=="__main__":
 
    while(True):
        # reading the image
        cap = cv2.VideoCapture(1)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)

        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)

        # time.sleep(2)

        cap.set(cv2.CAP_PROP_EXPOSURE, -8.0)
        gain = cap.get(cv2.CAP_PROP_GAIN)
        print("Gain: {0}".format(gain))
        cap.set(cv2.CAP_PROP_GAIN,120)
        cap.set(cv2.CAP_PROP_EXPOSURE,-3)
        expo = cap.get(cv2.CAP_PROP_EXPOSURE)
        print ("Exposure: {0}".format(expo))
        print(cap.get(cv2.CAP_PROP_EXPOSURE))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240) # before works fine
        cap.set(cv2.CAP_PROP_EXPOSURE, 4) 
        _, img = cap.read()
        print(np.shape(img))
        while(np.shape(img)[0] < 100):
            _, img = cap.read()
        print(np.shape(img))

        # displaying the image
        cv2.imshow('image', img)

        # setting mouse handler for the image
        # and calling the click_event() function
        cv2.setMouseCallback('image', click_event)

        # wait for a key to be pressed to exit
        cv2.waitKey(0)

        # close the window
        cv2.destroyAllWindows()