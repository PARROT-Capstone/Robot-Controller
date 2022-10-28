from posixpath import pathsep
from constants import CV_SANDBOX_HEIGHT, CV_SANDBOX_WIDTH
import mainHelper
from cv_main import CV
from controller_main import Controller
import time as time
from build import planner
import math

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

paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, [[800,800,0]])

# print the paths for each robot
for i in range(len(paths)):
    print("Robot", i, "path:")
    for j in range(len(paths[i])):
        print(paths[i][j])

# initialize controllers when path planner is done
controllers = [Controller(i, paths[i]) for i in range(mainHelper.Main_getRobotCounts())]

# control loop
while True:
    # start = time.time()
    computerVision.cv_runLocalizer()
    robotPoses = computerVision.cv_GetRobotPositions()
    # print(robotPoses)
    # end = time.time()
    palletPoses = computerVision.cv_GetPalletPositions() # NOTE right now we are using fiducial ID 2
    # for i in range(mainHelper.Main_getRobotCounts()):#TODO: change later
    #     linearWheelVelocities, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
    #     velLeftLinear, velRightLinear = linearWheelVelocities
    #     mainHelper.Main_SendRobotControls(i, velLeftLinear, velRightLinear)
    computerVision.cv_visualize(paths)
    # print("Time: ", end - start)
