import matplotlib.pyplot as plt
import numpy as np
import math
import mainHelper
from scipy.interpolate import interp1d
from scipy.misc import derivative
from controller_main import Controller

controller = Controller(0)

# Visualize trajectory
timesViz = np.linspace(0, 35, num=500, endpoint=True)
# X = controller.splineX
# Y = controller.splineY
# plt.plot(X(timesViz), Y(timesViz))
# for time in timesViz:
#     dist = 0.3
#     theta = controller.splineTheta(time)
#     plt.arrow(X(time), Y(time), dist*math.cos(theta), dist*math.sin(theta))
# plt.show()

# Better trajectory attempt
linearSpeed = 1 #mm/s
deltaTime = 0.001 #s
times = controller.times
x = controller.x
y = controller.y
theta = controller.theta
deltaXp = [x[i]+linearSpeed*deltaTime*math.cos(theta[i]) for i in range(len(x))]
deltaXn = [x[i]-linearSpeed*deltaTime*math.cos(theta[i]) for i in range(len(x))]
deltaYp = [y[i]+linearSpeed*deltaTime*math.sin(theta[i]) for i in range(len(y))]
deltaYn = [y[i]-linearSpeed*deltaTime*math.sin(theta[i]) for i in range(len(y))]
x.extend(deltaXp)
x.extend(deltaXn)
y.extend(deltaYp)
y.extend(deltaYn)
deltaTp = [times[i]+deltaTime for i in range(len(times))]
deltaTn = [times[i]-deltaTime for i in range(len(times))]
times.extend(deltaTp)
times.extend(deltaTn)

splineX = interp1d(times, x, kind='cubic')
splineY = interp1d(times, y, kind='cubic')
plt.plot(splineX(timesViz), splineY(timesViz))
traj = mainHelper.Main_getRobotPaths(0)
for point in traj:
    dist = 0.3
    plt.arrow(point[0], point[1], dist*math.cos(point[2]), dist*math.sin(point[2]), color="red", width=0.1)
plt.show()