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
import aiohttp
import asyncio
import pickle
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
robotPoses = computerVision.cv_GetRobotPositions()

# convert robot poses to 0 -> 2 pi
# for i in range(len(robotPoses)):
#     robotPoses[i][2] = -robotPoses[i][2]

# print robotPoses
for pose in robotPoses:
    print(pose)

palletPoses = computerVision.cv_GetPalletPositions() # NOTE right now we are using fiducial ID 2
map_size = (CV_SANDBOX_WIDTH, CV_SANDBOX_HEIGHT)

# set the goal poses to be palletPoses - 100mm in the x direction
goalPoses = []
for pose in palletPoses:
    goalPoses.append([pose[0] - 300, pose[1], pose[2]])

<<<<<<< HEAD
paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, [[100, 100, 0]])
=======
paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, [[200, 200, math.pi]])
# paths = [paths[0][0:3]]

# open a file, where you ant to store the data
file = open('Pickle2', 'wb')

data = [paths, robotPoses, palletPoses]
>>>>>>> 9a5d9ea03d87ddbed72e1ad0ce47033bd2a7d454



# print the paths for each robot
for i in range(len(paths)):
    print("Robot", i, "path:")
    for j in range(len(paths[i])):
        print(paths[i][j])

threads = []
robotNumber = len(paths)
robotCommands = [(0,0,constants.ELECTROMAGNET_DONT_SEND) for _ in range(robotNumber)]
def Main_RequestsThreading(robotId):
    session = requests.Session()
    session.headers.update({'Connection': 'Keep-Alive', 'Keep-Alive': "timeout=5, max=1000000"})
    # TODO: change robot url
    url = "http://parrot-robot1.wifi.local.cmu.edu"
    while True:
        velLeftLinear = robotCommands[i][0]
        velRightLinear = robotCommands[i][1]
        velLeftAng = velLeftLinear / constants.wheel_radius
        velRightAng = velRightLinear / constants.wheel_radius

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
    robotPoses = computerVision.cv_GetRobotPositions()
    print("CV Framerate: ", 1/(time.time() - start))
    # print(robotPoses)
    palletPoses = computerVision.cv_GetPalletPositions() # NOTE right now we are using fiducial ID 2
    for i in range(robotNumber):#TODO: change later
        robotCommand, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
        robotCommands[i] = robotCommand
        velLeftLinear, velRightLinear, electromagnet_command = robotCommand
        print("Controller Framerate: ", 1/(time.time() - start))
    # asyncio.run(mainHelper.Main_SendRobotControls(robotCommands))
    print("PostReq Framerate: ", 1/(time.time() - start))
    computerVision.cv_visualize(paths, targetPose, velRightLinear, velLeftLinear, ffterm, fbkterm)
    end = time.time()
    print("Frame Rate: ", 1 / (end - start))
