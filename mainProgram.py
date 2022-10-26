import mainHelper
from cv_main import CV
from controller_main import Controller
import time as time

computerVision = CV()

# call this when the cameras are setup
computerVision.cv_InitComputerVision()
computerVision.cv_GetRobotPositions()

# initialize controllers when path planner is done
# controllers = [Controller(i) for i in range(mainHelper.Main_getRobotCounts())]

# control loop
while True:
    start = time.time()
    computerVision.cv_runLocalizer()
    robotPoses = computerVision.cv_GetRobotPositions()
    end = time.time()
    # fiducialPoses = computerVision.cv_GetPalletPositions()
    # # for i in range(mainHelper.Main_getRobotCounts()):
    # #     linearWheelVelocities, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
    # #     velLeftLinear, velRightLinear = linearWheelVelocities
    # #     mainHelper.Main_SendRobotControls(i, velLeftLinear, velRightLinear)
    # computerVision.cv_visualize()
    print("Time: ", end - start)