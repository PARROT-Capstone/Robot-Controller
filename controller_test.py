import matplotlib.pyplot as plt
import numpy as np
import math
import mainHelper
from controller_main import Controller

timeEnd = 35
timeStep = 5000
timesViz = np.linspace(0, timeEnd, num=timeStep, endpoint=True)

controller = Controller(0)
X = controller.splineX
Y = controller.splineY
# figure, axis = plt.subplots(1, 2)
# axis[0].plot(timesViz, X(timesViz))
# axis[0].set_title("X")
# axis[1].plot(timesViz, Y(timesViz))
# axis[1].set_title("Y")
# plt.show()

# Visualize trajectory
figure, axis = plt.subplots(1, 2)
axis[0].plot(X(timesViz), Y(timesViz))
traj = mainHelper.Main_getRobotPaths(0)
for point in traj:
    dist = 0.3
    axis[0].arrow(point[0], point[1], dist*math.cos(point[2]), dist*math.sin(point[2]), color="red", width=0.1)
axis[0].set_title("Expected")

x = [X(0)]
y = [Y(0)]
theta = controller.theta[0]
for i in range(1, len(timesViz)):
    dt = timesViz[i] - timesViz[i-1]
    v, w = controller.controller_getFeedforwardTerm(timesViz[i], dt)
    dtheta = dt*w
    theta += dtheta/2
    x.append(x[-1]+dt*v*math.cos(theta))
    y.append(y[-1]+dt*v*math.sin(theta))
    theta += dtheta/2
axis[1].plot(x, y)
axis[1].set_title("Robot")

plt.show()