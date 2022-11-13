from posixpath import pathsep
from constants import CV_SANDBOX_HEIGHT, CV_SANDBOX_WIDTH
import constants
import requests
import mainHelper
from cv_main import CV
from controller_main import Controller
import threading
import time as time
import math
import os

# cd build and make the c++ code
os.system("cd build && make")

from build import planner

computerVision = CV()

# call this when the cameras are setup
computerVision.cv_InitComputerVision()
computerVision.cv_GetRobotPositions()

    
# NOTE Test planning a path from point A to point B
computerVision.cv_runLocalizer()
robotPoses, robotfiducialIds = computerVision.cv_GetRobotPositions()

# print robotPoses
for pose in robotPoses:
    print(pose)

palletPoses, palletfiducialIds = computerVision.cv_getPalletPositionsOffset() # NOTE right now we are using fiducial ID 2
map_size = (CV_SANDBOX_WIDTH, CV_SANDBOX_HEIGHT)

# set the goal poses to be palletPoses - 100mm in the x direction
goalPoses = []
for pose in palletPoses:
    goalPoses.append([pose[0] - 300, pose[1], pose[2]])

paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, [[1300, 800, 0]])

for path in paths:
    mainHelper.preconditionPath(path)

threads = []
robotNumber = len(paths)
robotCommands = [(0,0,constants.ELECTROMAGNET_DONT_SEND) for _ in range(robotNumber)]
import numpy as np
sendCommands = np.zeros(robotNumber, dtype=bool)
def Main_RequestsThreading(robotId):
    fiducialId = robotfiducialIds[robotId]
    robotHwNumber = constants.ROBOT_HARDWARE_NUMBERS[constants.ROBOT_FIDUCIALS.index(fiducialId)]
    url = f"http://parrot-robot{robotHwNumber}.wifi.local.cmu.edu"
    session = requests.Session()
    session.headers.update({'Connection': 'Keep-Alive', 'Keep-Alive': "timeout=5, max=1000000"})
    while True:
        # if sendCommands[robotId]:
        if True:
            sendCommands[robotId] = False
            velLeftLinear = robotCommands[robotId][0]
            velRightLinear = robotCommands[robotId][1]
            velLeftAng = velLeftLinear / constants.wheel_radius
            velRightAng = velRightLinear / constants.wheel_radius
            electromagnet_command = robotCommands[robotId][2]

            offset = 2

            # convert angular velocity to pwm
            scaleFactor = 7.5 # 1 PWM Duty Cycle = 7.5 mm/s


            leftPWM = 90 + (velLeftAng * scaleFactor)
            if velLeftAng > 0:
                leftPWM += offset
            elif velLeftAng < 0:
                leftPWM -= offset

            
            rightPWM = 90 - (velRightAng * scaleFactor)
            if velRightAng > 0:
                rightPWM -= offset
            elif velRightAng < 0:
                rightPWM += offset

            
            if (electromagnet_command != constants.ELECTROMAGNET_DONT_SEND):
                emJson = {"dtype": "pallet",
                        "power": (1 if electromagnet_command == constants.ELECTROMAGNET_ENABLE else 0)}
                session.post(url, data=emJson)

            servoJson = {"dtype": "speed", 
                        "servo1": int(leftPWM),
                        "servo2": int(rightPWM)
                    }
            session.post(url, data=servoJson)

for i in range(robotNumber):
    thread = threading.Thread(target=Main_RequestsThreading, args=(i,))
    threads.append(thread)
    thread.start()

# initialize controllers when path planner is done
controllers = [Controller(i, paths[i]) for i in range(robotNumber)]

# control loop
while True:
    start = time.time()
    computerVision.cv_runLocalizer()
    robotPoses, _ = computerVision.cv_GetRobotPositions()
    print("CV Framerate: ", 1/(time.time() - start))
    # print(robotPoses)
    palletPoses = computerVision.cv_GetPalletPositions() # NOTE right now we are using fiducial ID 2
    for i in range(robotNumber):#TODO: change later
        robotCommand, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
        robotCommands[i] = robotCommand
        velLeftLinear, velRightLinear, electromagnet_command = robotCommand
        print("Controller Framerate: ", 1/(time.time() - start))
        sendCommands[i] = True
    # asyncio.run(mainHelper.Main_SendRobotControls(robotCommands))
    print("PostReq Framerate: ", 1/(time.time() - start))
    computerVision.cv_visualize(paths, targetPose, velRightLinear, velLeftLinear, ffterm, fbkterm)
    end = time.time()
    print("Frame Rate: ", 1 / (end - start))
