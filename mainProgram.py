from posixpath import pathsep
from constants import CV_SANDBOX_HEIGHT, CV_SANDBOX_WIDTH
import mainHelper
from cv_main import CV
from controller_main import Controller
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

paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, [[200, 200, math.pi]])
# paths = [paths[0][0:3]]

# open a file, where you ant to store the data
file = open('Pickle2', 'wb')

data = [paths, robotPoses, palletPoses]

# dump information to that file
pickle.dump(data, file)

# close the file
file.close()


# print the paths for each robot
for i in range(len(paths)):
    print("Robot", i, "path:")
    for j in range(len(paths[i])):
        print(paths[i][j])

# initialize controllers when path planner is done
controllers = [Controller(i, paths[i]) for i in range(mainHelper.Main_getRobotCounts())]

# control loop
while True:
    start = time.time()
    computerVision.cv_runLocalizer()
    robotPoses = computerVision.cv_GetRobotPositions()
    print("CV Framerate: ", 1/(time.time() - start))
    # print(robotPoses)
    palletPoses = computerVision.cv_GetPalletPositions() # NOTE right now we are using fiducial ID 2
    robotCommands = []
    for i in range(mainHelper.Main_getRobotCounts()):#TODO: change later
        linearWheelVelocities, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
        velLeftLinear, velRightLinear, electromagnet_command = linearWheelVelocities
        print("Controller Framerate: ", 1/(time.time() - start))
        asyncio.run(mainHelper.Main_SendRobotControls(i, velLeftLinear, velRightLinear, electromagnet_command))
    print("PostReq Framerate: ", 1/(time.time() - start))
    computerVision.cv_visualize(paths, targetPose, velRightLinear, velLeftLinear, ffterm, fbkterm)
    end = time.time()
    print("Frame Rate: ", 1 / (end - start))
