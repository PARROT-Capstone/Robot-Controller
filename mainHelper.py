import math
import constants
import requests

def Main_getRobotCounts():
    return 1

def Main_getRobotPaths(robotId):
    return [(0,0,0,0), (3,0,0,3), (5,2,math.pi/4,5),
        (6,3,math.pi/2,7), (6,5,math.pi/2,8), (4,7,3*math.pi/4,10),
        (3,9,math.pi,12), (0,8,5*math.pi/4,14), (-1,6,3*math.pi/2,16),
        (-1,4,3*math.pi/2,18), (-1,0,0,20)]
    return [(-1, -1, math.pi, 0, 0), (0, 0, 0, 1, 0), (1, 0, 0, 2, 0), (2, 4, math.pi/2, 6, 0),
    (0, 4, -math.pi/2, 8, 0), (0, -6, -math.pi/2, 11, 0), (10, 0, math.pi/2, 20, 0),
    (10, 0, math.pi/2, 25, 0), (-2, -4, -math.pi/4, 35, 0)]

def Main_SendRobotControls(robotId, velLeftLinear, velRightLinear):
    velLeftAng = velLeftLinear / constants.wheel_radius
    velRightAng = velRightLinear / constants.wheel_radius

    # convert angular velocity to pwm
    scaleFactor = 7.5 # 1 PWM Duty Cycle = 7.5 mm/s
    leftPWM = 90 + (velLeftAng * scaleFactor)
    rightPWM = 90 - (velRightAng * scaleFactor)

    # clamp pwm between 0 and 180
    leftPWM = min(max(leftPWM, 0), 180)
    rightPWM = min(max(rightPWM, 0), 180)

    # TODO: change robot url
    robot_url = "http://parrot-robot1.wifi.local.cmu.edu"
    json = {"dtype": "speed", 
                "servo1": int(leftPWM),
                "servo2": int(rightPWM)
            }

    x = requests.post(robot_url, data=json)  