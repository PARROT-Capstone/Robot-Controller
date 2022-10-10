import math
import constants

def Main_getRobotCounts():
    return 1

def Main_getRobotPaths(robotId):
    return [(-1, -1, math.pi, 0), (0, 0, 0, 1, 0), (1, 0, 0, 2, 0), (2, 4, math.pi/2, 6, 0),
    (0, 4, -math.pi/2, 8, 0), (0, -6, -math.pi/2, 11, 0), (10, 0, math.pi/2, 20, 0),
    (10, 0, math.pi/2, 25, 0), (-2, -4, -math.pi/4, 35, 0)]

def Main_SendRobotControls(robotId, velLeftLinear, velRightLinear):
    velLeftAng = velLeftLinear / constants.wheel_radius
    velRightAng = velRightLinear / constants.wheel_radius

    # convert angular velocity to pwm
    scaleFactor = 5
    leftPWM = velLeftAng * scaleFactor + 90
    rightPWM = 90 - velRightAng * scaleFactor

    # clamp pwm between 0 and 180
    leftPWM = min(max(leftPWM, 0), 180)
    rightPWM = min(max(rightPWM, 0), 180)

    # TODO: change robot url
    robot_url = "http://172.26.171.154"
    json = {"dtype": "speed", 
                "servo1": int(leftPWM),
                "servo2": int(rightPWM)
            }

    print(json)
    x = requests.post(robot_url, data=json)  