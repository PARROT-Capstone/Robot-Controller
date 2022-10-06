import mainHelper
from cv_main import CV
from controller_main import Controller

computerVision = CV()

# call this when the cameras are setup
computerVision.cv_InitComputerVision()
computerVision.cv_GetRobotPositions()

# initialize controllers when path planner is done
controllers = [Controller(i) for i in range(mainHelper.Main_getRobotCounts())]

# control loop
while True:
    robotPoses = computerVision.cv_GetRobotPositions()
    for i in range(mainHelper.Main_getRobotCounts()):
        velLeftLinear, velRightLinear = controllers[i].controller_getRobotVelocities(robotPoses[i])
        mainHelper.Main_SendRobotControls(i, velLeftLinear, velRightLinear)