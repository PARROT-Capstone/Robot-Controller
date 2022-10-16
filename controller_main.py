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

xIndex = 0
yIndex = 1
thetaIndex = 2
timeIndex = 3

dt = 1e-6

# Based on Mobile Robot Programming Lab and https://www.researchgate.net/profile/Muhammad-Asif-62/publication/266798448_20140930101805712/links/543cc55d0cf24ef33b7639cb/20140930101805712.pdf

# TODO: Electromagnet Commands

class Controller:
    def __init__(self, robotId):
        self.startTime = time.time()
        self.robotId = robotId
        # path is (x, y, theta, relative_time)
        self.robotPath = mainHelper.Main_getRobotPaths(self.robotId)
        if (self.robotPath is None or len(self.robotPath) < 2):
            raise Exception('Error with Robot path')
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

        # Get past and next point
        pastPoint, nextPoint = self.controller_getPastNextPoints(relativeTime)

        # Interpolate between past and next points
        xFunc, yFunc = self.controller_getInterpolation(pastPoint, nextPoint)

        # Find next point in sequence in global coordinates
        targetPose_global, dxdt, dydt = self.controller_getNextTargetPoint(relativeTime, pastPoint, nextPoint, xFunc, yFunc)

        # Find error in robot coordinates
        self.robotError = self.controller_getErrorRobotCoords(targetPose_global, robotPose_global)

        # Find error sum and error difference
        self.robotErrorSum = tuple(np.add(self.robotError, self.robotErrorSum))
        self.errorDiff_robot = tuple(np.subtract(self.robotError, self.robotErrorLast)) # TODO: account for wrap around with theta error

        # Find Feedforward term
        feedforward = self.controller_getFeedforwardTerm(relativeTime, xFunc, yFunc, dxdt, dydt)

        # Find Feedback term
        feedback = self.controller_getFeedbackTerm()

        # Update error last
        self.last_position_time = currentTime
        self.robotErrorLast = self.robotError

        # Return wheel speeds based on FFW + FBK terms
        self.robotCommand = tuple(np.add(feedforward, feedback))
        return (self.controller_getWheelVelocities(), targetPose_global)
    
    def controller_getPastNextPoints(self, relativeTime):
        # point is (x, y, theta, relative_time, tag)
        pastPoint = None
        nextPoint = None
        for point in self.robotPath:
            pointTime = point[timeIndex]
            if pointTime <= relativeTime:
                pastPoint = point
            if pointTime > relativeTime:
                nextPoint = point
                break
        
        if (pastPoint is None):
            pastPoint = nextPoint
        elif (nextPoint is None):
            nextPoint = pastPoint
        if (pastPoint is None or nextPoint is None):
            raise Exception("Error finding next point on trajectory")
        return (pastPoint, nextPoint)

    def controller_getInterpolation(self, pastPoint, nextPoint):
        if pastPoint[timeIndex] == nextPoint[timeIndex]:
            xFunc = lambda t: pastPoint[xIndex]
            yFunc = lambda t: nextPoint[xIndex]
        # if Thetas are same, drive straight
        elif pastPoint[thetaIndex] == nextPoint[thetaIndex]:
            deltaX = nextPoint[xIndex] - pastPoint[xIndex]
            deltaY = nextPoint[yIndex] - pastPoint[yIndex]
            deltaT = nextPoint[timeIndex] - pastPoint[timeIndex]
            xFunc = lambda t: pastPoint[xIndex] + (t-pastPoint[timeIndex]) * deltaX / deltaT
            yFunc = lambda t: pastPoint[yIndex] + (t-pastPoint[timeIndex]) * deltaY / deltaT
        # spline interpolation
        else:
            times = [pastPoint[timeIndex]-constants.controlsDeltaTime, pastPoint[timeIndex], pastPoint[timeIndex]+constants.controlsDeltaTime,
                    nextPoint[timeIndex]-constants.controlsDeltaTime, nextPoint[timeIndex], nextPoint[timeIndex]+constants.controlsDeltaTime]
            x = [pastPoint[xIndex]-constants.controlsLinearSpeed*constants.controlsDeltaTime*math.cos(pastPoint[thetaIndex]),
                pastPoint[xIndex],
                pastPoint[xIndex]+constants.controlsLinearSpeed*constants.controlsDeltaTime*math.cos(pastPoint[thetaIndex]),
                nextPoint[xIndex]-constants.controlsLinearSpeed*constants.controlsDeltaTime*math.cos(nextPoint[thetaIndex]),
                nextPoint[xIndex],
                nextPoint[xIndex]+constants.controlsLinearSpeed*constants.controlsDeltaTime*math.cos(nextPoint[thetaIndex])]
            y = [pastPoint[yIndex]-constants.controlsLinearSpeed*constants.controlsDeltaTime*math.sin(pastPoint[thetaIndex]),
                pastPoint[yIndex],
                pastPoint[yIndex]+constants.controlsLinearSpeed*constants.controlsDeltaTime*math.sin(pastPoint[thetaIndex]),
                nextPoint[yIndex]-constants.controlsLinearSpeed*constants.controlsDeltaTime*math.sin(nextPoint[thetaIndex]),
                nextPoint[yIndex],
                nextPoint[yIndex]+constants.controlsLinearSpeed*constants.controlsDeltaTime*math.sin(nextPoint[thetaIndex])]
            xFunc = interp1d(times, x, kind='cubic', fill_value="extrapolate")
            yFunc = interp1d(times, y, kind='cubic', fill_value="extrapolate")
        return (xFunc, yFunc)

    # Gets next target point - ((x, y, theta), dxdt, dydt)
    def controller_getNextTargetPoint(self, relativeTime, pastPoint, nextPoint, xFunc, yFunc):
        if relativeTime >= nextPoint[timeIndex]:
            return ((nextPoint[xIndex], nextPoint[yIndex], nextPoint[thetaIndex]), 0, 0)
        if relativeTime < pastPoint[timeIndex]:
            return ((pastPoint[xIndex], pastPoint[yIndex], pastPoint[thetaIndex]), 0, 0)

        x = xFunc(relativeTime)
        y = yFunc(relativeTime)
        dxdt = derivative(xFunc, relativeTime, n=1, dx=dt)
        dydt = derivative(yFunc, relativeTime, n=1, dx=dt)
        if (pastPoint[thetaIndex] == nextPoint[thetaIndex]):
            theta = pastPoint[thetaIndex]
        else:
            # TODO: dxdt and dydt are both 0
            theta = math.atan2(dydt, dxdt)

        return ((float(x), float(y), theta), dxdt, dydt)
    
    # Poses are tuples of (x, y, theta)
    def controller_getErrorRobotCoords(self, targetPose_global, robotPose_global):
        errorRobot_global = np.array([targetPose_global[xIndex] - robotPose_global[xIndex], targetPose_global[yIndex] - robotPose_global[yIndex]])
        robotTheta = robotPose_global[thetaIndex]
        rotationMatrix = np.array([[math.cos(robotTheta), -math.sin(robotTheta)], [math.sin(robotTheta), math.cos(robotTheta)]])
        invRotationMatrix = (rotationMatrix).T
        errorRobot_robot = invRotationMatrix @ errorRobot_global
        errorTheta = targetPose_global[thetaIndex] - robotPose_global[thetaIndex]
        errorTheta = math.atan2(math.sin(errorTheta), math.cos(errorTheta))
        return (float(errorRobot_robot[xIndex]), float(errorRobot_robot[yIndex]), errorTheta)

    # Returns (linear velocity, angular velocity)
    def controller_getFeedforwardTerm(self, relativeTime, xFunc, yFunc, dxdt, dydt):
        # Considering bounds of path
        if (relativeTime < self.robotPath[0][timeIndex]) or (relativeTime >= self.robotPath[-1][timeIndex]):
            return (0, 0)

        d2xdt2 = derivative(xFunc, relativeTime, n=2,dx=dt)
        d2ydt2 = derivative(yFunc, relativeTime, n=2, dx=dt)
        linearVelocity = math.sqrt(dxdt**2 + dydt**2)
        if (linearVelocity == 0):
            curvature = 0
        else:
            curvature = (dxdt * d2ydt2 - dydt * d2xdt2) / (linearVelocity ** 3)
        angularVelocity = linearVelocity * curvature
        return (linearVelocity, angularVelocity)

    # Returns (linear velocity, angular velocity)
    def controller_getFeedbackTerm(self):
        errorV = self.robotError[xIndex] * Kpx
        errorOmega = self.robotError[yIndex] * Kpy + self.robotError[thetaIndex] * Kpth
        return (errorV, errorOmega)

    def controller_getWheelVelocities(self):
        velLeft = self.robotCommand[xIndex] - constants.half_wheel_base * self.robotCommand[yIndex]
        velRight = self.robotCommand[xIndex] + constants.half_wheel_base * self.robotCommand[yIndex]
        return (velLeft, velRight)