import matplotlib.pyplot as plt
import numpy as np
import math
import mainHelper
import time
import constants
from controller_main import Controller

dt = 0.04 #sec

# Visualize trajectory
plt.subplot(3, 2, 1)
traj = mainHelper.Main_getRobotPaths(0)
for point in traj:
    dist = 0.3
    plt.arrow(point[0], point[1], dist*math.cos(point[2]), dist*math.sin(point[2]), color="red", width=0.1)
plt.title("Expected")

robotX = [traj[0][0]]
robotY = [traj[0][1]]
robotTheta = traj[0][2]

expectedX = [traj[0][0]]
expectedY = [traj[0][1]]
ffv = []
ffw = []
fbkv = []
fbkw = []
times = []

controller = Controller(0)
endTime = time.time() + 40
while time.time() <= endTime:
    startCompute = time.time()
    wheelV, pos, ff, fbk = controller.controller_getRobotVelocities((robotX[-1], robotY[-1], robotTheta))
    expectedX.append(pos[0])
    expectedY.append(pos[1])
    ffv.append(ff[0])
    ffw.append(ff[1])
    fbkv.append(fbk[0])
    fbkw.append(fbk[1])

    velLeft, velRight = wheelV
    v = (velLeft + velRight) / 2
    w = (velRight - velLeft) / constants.wheel_base

    dtheta = dt*w
    robotTheta += dtheta/2
    robotX.append(robotX[-1]+dt*v*math.cos(robotTheta))
    robotY.append(robotY[-1]+dt*v*math.sin(robotTheta))
    robotTheta += dtheta/2

    stopCompute = time.time()
    times.append(stopCompute-startCompute)
    while time.time() < startCompute + dt:
        continue

plt.plot(expectedX, expectedY)

plt.subplot(3,2,2)
plt.plot(robotX, robotY)
plt.title("Robot")

plt.subplot(3,2,3)
plt.plot(ffv)
plt.title("FF v")

plt.subplot(3,2,4)
plt.plot(ffw)
plt.title("FF w")

plt.subplot(3,2,5)
plt.plot(fbkv)
plt.title("FBK v")

plt.subplot(3,2,6)
plt.plot(fbkw)
plt.title("FBK w")

print("Frame Rate: {} fps".format(1/dt))
print("Expected: ({}, {})\nActual: ({}, {})".format(expectedX[-1], expectedY[-1], robotX[-1], robotY[-1]))
print("Distance: {}".format(math.sqrt((expectedX[-1] - robotX[-1])**2 + (expectedY[-1] - robotY[-1])**2)))
print("Average Compute Time: {} sec".format(sum(times)/len(times)))

plt.show()