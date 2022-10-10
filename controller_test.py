import matplotlib.pyplot as plt
import numpy as np
import math
import mainHelper
from controller_main import Controller

controller = Controller(0)

# Visualize trajectory
figure, axis = plt.subplots(1, 2)
timeEnd = 35
timeStep = 500
timesViz = np.linspace(0, timeEnd, num=timeStep, endpoint=True)
X = controller.splineX
Y = controller.splineY
axis[0].plot(X(timesViz), Y(timesViz))
traj = mainHelper.Main_getRobotPaths(0)
for point in traj:
    dist = 0.3
    axis[0].arrow(point[0], point[1], dist*math.cos(point[2]), dist*math.sin(point[2]), color="red", width=0.1)
axis[0].set_title("Expected")

x = [X(0)]
y = [Y(0)]
theta = controller.theta[0]
for time in timesViz:
    v, w = controller.controller_getFeedforwardTerm(time)
    dt = timeEnd/timeStep
    x.append(x[-1]+dt*v*math.cos(theta))
    y.append(y[-1]+dt*v*math.sin(theta))
    theta += dt*w
axis[1].plot(x, y)
axis[1].set_title("Robot")

plt.show()