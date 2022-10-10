import numpy as np
import time
import math
from scipy.interpolate import interp1d
from scipy.misc import derivative
import mainHelper
import constants

Kpx = 0.3
Kpy = 0.002
Kpth = 0.1
Ki = 0
Kd = 0

# Based on Mobile Robot Programming Lab and https://www.researchgate.net/profile/Muhammad-Asif-62/publication/266798448_20140930101805712/links/543cc55d0cf24ef33b7639cb/20140930101805712.pdf

# TODO: Simulation and Electromagnet Commands

class Controller:
    def __init__(self, robotId):
        self.startTime = time.time()
        self.robotId = robotId
        self.robotPath = mainHelper.Main_getRobotPaths(self.robotId)
        # path is (x, y, theta, relative_time)
        self.times = [self.robotPath[i][3] for i in range(len(self.robotPath))]
        self.x = [self.robotPath[i][0] for i in range(len(self.robotPath))]
        self.y = [self.robotPath[i][1] for i in range(len(self.robotPath))]
        self.theta = [self.robotPath[i][2] for i in range(len(self.robotPath))]
        deltaXp = [self.x[i]+constants.controlsLinearSpeed*constants.controlsDeltaTime*math.cos(self.theta[i]) for i in range(len(self.x))]
        deltaXn = [self.x[i]-constants.controlsLinearSpeed*constants.controlsDeltaTime*math.cos(self.theta[i]) for i in range(len(self.x))]
        deltaYp = [self.y[i]+constants.controlsLinearSpeed*constants.controlsDeltaTime*math.sin(self.theta[i]) for i in range(len(self.y))]
        deltaYn = [self.y[i]-constants.controlsLinearSpeed*constants.controlsDeltaTime*math.sin(self.theta[i]) for i in range(len(self.y))]
        self.x.extend(deltaXp)
        self.x.extend(deltaXn)
        self.y.extend(deltaYp)
        self.y.extend(deltaYn)
        deltaTp = [self.times[i]+constants.controlsDeltaTime for i in range(len(self.times))]
        deltaTn = [self.times[i]-constants.controlsDeltaTime for i in range(len(self.times))]
        self.times.extend(deltaTp)
        self.times.extend(deltaTn)
        self.splineX = interp1d(self.times, self.x, kind='cubic', fill_value="extrapolate")
        self.splineY = interp1d(self.times, self.y, kind='cubic', fill_value="extrapolate")
        self.robotError = (0, 0, 0)
        self.errorDiff_robot = (0, 0, 0)
        self.robotErrorLast = (0, 0, 0)
        self.robotErrorSum = (0, 0, 0)
        self.robotCommand = (0, 0)
        self.last_position_time = self.startTime

    def controller_getRobotVelocities(self, robotPose_global):
        # Get robot time
        currentTime = time.time()
        relativeTime = currentTime - self.startTime

        # Find next point in sequence in global coordinates
        targetPose_global = self.controller_getNextTargetPoint(relativeTime)

        # Find error in robot coordinates
        self.robotError = self.controller_getErrorRobotCoords(targetPose_global, robotPose_global)

        # Find error sum and error difference
        self.robotErrorSum = tuple(np.add(self.robotError, self.robotErrorSum))
        self.errorDiff_robot = tuple(np.subtract(self.robotError, self.robotErrorLast))

        # Find Feedforward term
        feedforward = self.controller_getFeedforwardTerm(targetPose_global, relativeTime)

        # Find Feedback term
        feedback = self.controller_getFeedbackTerm()

        # Update error last
        self.last_position_time = currentTime
        self.robotErrorLast = self.robotError

        # Return wheel speeds based on FFW + FBK terms
        self.robotCommand = tuple(np.add(feedforward, feedback))
        return self.controller_getWheelVelocities()
    
    # Gets next target point - returns None if no path exists
    def controller_getNextTargetPoint(self, relativeTime):
        if self.robotPath is None or len(self.robotPath) == 0:
            return None
        # path is (x, y, theta, relative_time, tag)
        for point in self.robotPath:
            if point[3] > relativeTime:
                return point
        return self.robotPath[-1]
    
    # Poses are tuples of (x, y, theta)
    def controller_getErrorRobotCoords(self, targetPose_global, robotPose_global):
        errorRobot_global = np.array([targetPose_global[0] - robotPose_global[0], targetPose_global[1] - robotPose_global[1]])
        robotTheta = robotPose_global[2]
        rotationMatrix = np.array([[math.cos(robotTheta), -math.sin(robotTheta)], [math.sin(robotTheta), math.cos(robotTheta)]])
        invRotationMatrix = (rotationMatrix).T
        errorRobot_robot = invRotationMatrix @ errorRobot_global
        errorTheta = targetPose_global[0] - robotPose_global[0]
        errorTheta = math.atan2(math.sin(errorTheta), math.cos(errorTheta))
        return (errorRobot_robot[0], errorRobot_robot[1], errorTheta)
    
    # Returns (linear velocity, angular velocity)
    def controller_getFeedforwardTerm(self, relativeTime):
        dxdt = derivative(self.splineX, relativeTime, n=1, dx=1e-3)
        dydt = derivative(self.splineY, relativeTime, n=1, dx=1e-3)
        d2xdt2 = derivative(self.splineX, relativeTime, n=2,dx=1e-3)
        d2ydt2 = derivative(self.splineY, relativeTime, n=2, dx=1e-3)
        linearVelocity = math.sqrt(dxdt**2 + dydt**2)
        curvature = (dxdt * d2ydt2 - dydt * d2xdt2) / (linearVelocity ** 3)
        angularVelocity = linearVelocity * curvature
        return (linearVelocity, angularVelocity)

    # Returns (linear velocity, angular velocity)
    def controller_getFeedbackTerm(self):
        errorV = self.robotError[0] * Kpx
        errorOmega = self.robotError[1] * Kpy + self.robotError[2] * Kpth
        return (errorV, errorOmega)

    def controller_getWheelVelocities(self):
        velLeft = self.robotCommand[0] - constants.half_wheel_base * self.robotCommand[1]
        velRight = self.robotCommand[0] + constants.half_wheel_base * self.robotCommand[1]
        return (velLeft, velRight)