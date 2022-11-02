import pickle
from controller_main import Controller
import time
import math
import matplotlib.pyplot as plt

file = open('workingPathPickle', 'rb')
# data = [paths, robotPoses, palletPoses]
data = pickle.load(file)
paths = data[0]
robotPath = paths[0]
controllerTime = 0
endTime = robotPath[-1][3]
controller = Controller(0, robotPath)
targetPoses = []
targetTimes = []
frameRate = 10
while controllerTime < endTime:
    linearWheelVelocities, targetPose, ffterm, fbkterm = controller.controller_getRobotVelocities([0, 0, 0])
    controllerTime = time.time() - controller.startTime
    targetTimes.append(controllerTime)
    targetPoses.append(targetPose)
    time.sleep(1/frameRate)

plt.figure()
targetX = [targetPoses[i][0] for i in range(len(targetPoses))]
targetY = [-targetPoses[i][1] for i in range(len(targetPoses))]
for targetPose in targetPoses:
    (robot_pos_x, robot_pos_y, robot_rotation_rad) = targetPose
    plt.arrow(robot_pos_x, -robot_pos_y, 1*math.cos(robot_rotation_rad), 1*math.sin(robot_rotation_rad), width=1, color='b', length_includes_head=True)

# print(robotPath)
# for robotPose in robotPath:
#     (robot_pos_x, robot_pos_y, robot_rotation_rad, time_sec, tag) = robotPose
#     plt.arrow(robot_pos_x, -robot_pos_y, 20*math.cos(robot_rotation_rad), 25*math.sin(robot_rotation_rad), width=5, color='g')
plt.show()