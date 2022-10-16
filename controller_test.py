import matplotlib.pyplot as plt
import numpy as np
import math
import mainHelper
import time
import constants
from controller_main import Controller

dt = 0.03 #sec


# Visualize trajectory
figure, axis = plt.subplots(1, 2)
traj = mainHelper.Main_getRobotPaths(0)
for point in traj:
    dist = 0.3
    axis[0].arrow(point[0], point[1], dist*math.cos(point[2]), dist*math.sin(point[2]), color="red", width=0.1)
axis[0].set_title("Expected")

robotX = [traj[0][0]]
robotY = [traj[0][1]]
robotTheta = traj[0][2]

expectedX = [traj[0][0]]
expectedY = [traj[0][1]]

controller = Controller(0)
endTime = time.time() + 40
while time.time() <= endTime:
    wheelV, pos = controller.controller_getRobotVelocities((robotX[-1], robotY[-1], robotTheta))
    expectedX.append(pos[0])
    expectedY.append(pos[1])

    velLeft, velRight = wheelV
    v = (velLeft + velRight) / 2
    w = (velRight - velLeft) / constants.wheel_base

    dtheta = dt*w
    robotTheta += dtheta/2
    robotX.append(robotX[-1]+dt*v*math.cos(robotTheta))
    robotY.append(robotY[-1]+dt*v*math.sin(robotTheta))
    robotTheta += dtheta/2
    time.sleep(dt)

axis[0].plot(expectedX, expectedY)

axis[1].plot(robotX, robotY)
axis[1].set_title("Robot")

print("Expected: {}, {}\nActual: {}, {}".format(expectedX[-1], expectedY[-1], robotX[-1], robotY[-1]))

plt.show()