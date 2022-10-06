import constants

def Main_getRobotCounts():
    return 1

def Main_getRobotPaths(robotId):
    return [(0, 0, 0, 0)]

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