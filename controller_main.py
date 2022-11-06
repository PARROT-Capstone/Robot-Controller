from copy import copy
import numpy as np
import time
import math
from scipy.interpolate import interp1d
from scipy.misc import derivative
import mainHelper
import constants
import matplotlib.pyplot as plt
from scipy import interpolate


Kpx = constants.CONTROLS_ROBOT_PID_KPx
Kdx = constants.CONTROLS_ROBOT_PID_KDx

Kpy = constants.CONTROLS_ROBOT_PID_KPy
Kdy = constants.CONTROLS_ROBOT_PID_KDy

Kpth = constants.CONTROLS_ROBOT_PID_KPtheta
Ki = constants.CONTROLS_ROBOT_PID_KItheta
Kd = constants.CONTROLS_ROBOT_PID_KDtheta

# emulate enums
xIndex = 0
yIndex = 1
thetaIndex = 2
timeIndex = 3
tagIndex = 4

dt = 1e-6

# Based on Mobile Robot Programming Lab and https://www.researchgate.net/profile/Muhammad-Asif-62/publication/266798448_20140930101805712/links/543cc55d0cf24ef33b7639cb/20140930101805712.pdf

# TODO: Electromagnet Commands

class Controller:
    def __init__(self, robotId, robotPath):
        self.startTime = time.time()
        self.robotId = robotId
        # path is (x, y, theta, relative_time)
        self.robotPath = robotPath
        if (self.robotPath is None or len(self.robotPath) < 2):
            print("Robot Path: ", self.robotPath)
            raise Exception('Error with Robot path')
        self.robotError = (0, 0, 0)
        self.errorDiff_robot = (0, 0, 0)
        self.robotErrorLast = (0, 0, 0)
        self.robotErrorSum = (0, 0, 0)
        self.robotCommand = (0, 0)
        self.last_position_time = self.startTime
        self.state = constants.CONTROLS_STATE_DRIVING_TO_PALLET
        if constants.CONTROLS_DEBUG:
            self.xErrorList = []
            self.yErrorList = []
            self.timeList = []
            self.fig = plt.figure()

    def controller_getRobotVelocities(self, robotPose_global):
        # Get robot time
        currentTime = time.time()
        relativeTime = currentTime - self.startTime
        robotPose_global[yIndex] *= -1
        
        # print("Robot Pose: ", robotPose_global)

        # Get past and next point
        pastPoint, nextPoint = self.controller_getPastNextPoints(relativeTime)

        # Interpolate between past and next points
        xFunc, yFunc, thetaFunc = self.controller_getInterpolation(pastPoint, nextPoint)

        # Find next point in sequence in global coordinates
        targetPose_global, dxdt, dydt = self.controller_getNextTargetPoint(relativeTime, pastPoint, nextPoint, xFunc, yFunc, thetaFunc)
        # print("Target Pose: ", targetPose_global)

        # Find error in robot coordinates
        self.robotError = self.controller_getErrorRobotCoords(targetPose_global, robotPose_global)
        print("Robot Error: ", self.robotError)
        # Matplotlib live plot of error
        if constants.CONTROLS_DEBUG:
            self.xErrorList.append(self.robotError[xIndex])
            self.yErrorList.append(self.robotError[yIndex])
            self.timeList.append(relativeTime)

            bufferLen = 10

            while len(self.xErrorList) > bufferLen:
                self.xErrorList.pop(0)
            while len(self.yErrorList) > bufferLen:
                self.yErrorList.pop(0)
            while len(self.timeList) > bufferLen:
                self.timeList.pop(0)

            self.fig.clear()
            plt.plot(self.timeList, self.xErrorList, 'r', linestyle='-', label='xError')
            # plt.plot(self.timeList, self.yErrorList, 'b', linestyle='-', label='yError')
            plt.show(block=False)
            plt.pause(0.0001)


        targetPose_global = (targetPose_global[xIndex], -1*targetPose_global[yIndex], targetPose_global[thetaIndex])


        # Find error sum and error difference
        self.robotErrorSum = tuple(np.add(self.robotError, self.robotErrorSum))
        self.errorDiff_robot = tuple(np.subtract(self.robotError, self.robotErrorLast)) # TODO: account for wrap around with theta error

        # Find Feedforward term
        feedforward = self.controller_getFeedforwardTerm(relativeTime, xFunc, yFunc, thetaFunc, dxdt, dydt)

        # Find Feedback term
        feedback = self.controller_getFeedbackTerm()

        # Update error last
        self.last_position_time = currentTime
        self.robotErrorLast = self.robotError

        # Return wheel speeds based on FFW + FBK terms
        self.robotCommand = tuple(np.add(feedforward, feedback))
        print("V, W: ", self.robotCommand)
        velLeft, velRight = self.controller_getWheelVelocities()
        print("Left, Right: ", velLeft, velRight)

        # Determine when to send electromagnet command
        electromagnet_command = constants.ELECTROMAGNET_DONT_SEND
        if ((nextPoint[timeIndex] <= relativeTime + constants.CONTROLS_ELECTROMAGNET_TIME_THRESHOLD) and (nextPoint[tagIndex] != 0)):
            electromagnet_command = nextPoint[tagIndex]
            self.state = constants.CONTROLS_STATE_DRIVING_TO_GOAL if electromagnet_command == constants.ELECTROMAGNET_ENABLE else constants.CONTROLS_STATE_DRIVING_TO_PALLET
        elif ((pastPoint[tagIndex] == constants.ELECTROMAGNET_ENABLE) and (self.state == constants.CONTROLS_STATE_DRIVING_TO_PALLET)):
            electromagnet_command = constants.ELECTROMAGNET_ENABLE
            self.state = constants.CONTROLS_STATE_DRIVING_TO_GOAL
        elif ((pastPoint[tagIndex] == constants.ELECTROMAGNET_DISABLE) and (self.state == constants.CONTROLS_STATE_DRIVING_TO_GOAL)):
            electromagnet_command = constants.ELECTROMAGNET_DISABLE
            self.state = constants.CONTROLS_STATE_DRIVING_TO_PALLET


        if (constants.CONTROLS_DEBUG and electromagnet_command != constants.ELECTROMAGNET_DONT_SEND):
            print("Electromagnet command: ", electromagnet_command)

        return ((velLeft, velRight, electromagnet_command), targetPose_global, feedforward, feedback)
    
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
        pastPointCopy = copy(pastPoint)
        pastPointCopy[yIndex] *= -1
        nextPointCopy = copy(nextPoint)
        nextPointCopy[yIndex] *= -1
        return (pastPointCopy, nextPointCopy)

    def controller_getInterpolation(self, pastPoint, nextPoint):
        if pastPoint[timeIndex] == nextPoint[timeIndex]:
            xFunc = lambda t: pastPoint[xIndex]
            yFunc = lambda t: pastPoint[yIndex]
            thetaFunc = None
        # if Thetas are same, drive straight
        elif pastPoint[thetaIndex] == nextPoint[thetaIndex]:
            deltaX = nextPoint[xIndex] - pastPoint[xIndex]
            deltaY = nextPoint[yIndex] - pastPoint[yIndex]
            deltaT = nextPoint[timeIndex] - pastPoint[timeIndex]
            xFunc = lambda t: pastPoint[xIndex] + (t-pastPoint[timeIndex]) * deltaX / deltaT
            yFunc = lambda t: pastPoint[yIndex] + (t-pastPoint[timeIndex]) * deltaY / deltaT
            thetaFunc = None
        elif pastPoint[xIndex] == nextPoint[xIndex] and pastPoint[yIndex] == nextPoint[yIndex]:
            xFunc = lambda t: pastPoint[xIndex]
            yFunc = lambda t: pastPoint[yIndex]

            #TODO: Pick shortest theta delta
            thetaFunc = lambda t: ((nextPoint[thetaIndex] - pastPoint[thetaIndex]) * ((t-pastPoint[timeIndex]) / (nextPoint[timeIndex] - pastPoint[timeIndex]))) + pastPoint[thetaIndex]
        # spline interpolation
        else:
            # https://stackoverflow.com/questions/36644259/cubic-hermit-spline-interpolation-python
            # resolution = float(resolution)
            points = np.asarray([[pastPoint[xIndex], pastPoint[yIndex]], [nextPoint[xIndex], nextPoint[yIndex]]])
            tangents = np.asarray([[np.cos(pastPoint[thetaIndex]), np.sin(pastPoint[thetaIndex])], [np.cos(nextPoint[thetaIndex]), np.sin(nextPoint[thetaIndex])]])
            tangents = tangents * constants.tangent_curviness
            nPoints, dim = points.shape

            # Parametrization parameter s.
            dp = np.diff(points, axis=0)                 # difference between points
            dp = np.linalg.norm(dp, axis=1)              # distance between points
            d = np.cumsum(dp)                            # cumsum along the segments
            d = np.hstack([[0],d])                       # add distance from first point
            l = d[-1]                                    # length of point sequence
            # nSamples = int(l/resolution)                 # number of samples
            # s,r = np.linspace(0,l,nSamples,retstep=True) # sample parameter and step

            # Bring points and (optional) tangent information into correct format.
            assert(len(points) == len(tangents))
            data = np.empty([nPoints, dim], dtype=object)
            for i,p in enumerate(points):
                t = tangents[i]
                # Either tangent is None or has the same
                # number of dimensions as the point p.
                assert(t is None or len(t)==dim)
                fuse = list(zip(p,t) if t is not None else zip(p,))
                data[i,:] = fuse

            # Compute splines per dimension separately.
            # samples = np.zeros([nSamples, dim])
            xPoints = interpolate.BPoly.from_derivatives(d, data[:,0])
            yPoints = interpolate.BPoly.from_derivatives(d, data[:,1])
            # for i in range(dim):
            #     poly = interpolate.BPoly.from_derivatives(d, data[:,i])
            #     samples[:,i] = poly(s)
            deltaT = nextPoint[timeIndex] - pastPoint[timeIndex]
            xFunc = lambda t: xPoints(l*(t-pastPoint[timeIndex]) / deltaT)
            yFunc = lambda t: yPoints(l*(t-pastPoint[timeIndex]) / deltaT)
            thetaFunc = None

            
        return (xFunc, yFunc, thetaFunc)

    # Gets next target point - ((x, y, theta), dxdt, dydt)
    def controller_getNextTargetPoint(self, relativeTime, pastPoint, nextPoint, xFunc, yFunc, thetaFunc):
        if relativeTime >= nextPoint[timeIndex]:
            return ((nextPoint[xIndex], nextPoint[yIndex], nextPoint[thetaIndex]), 0, 0)
        if relativeTime < pastPoint[timeIndex]:
            return ((pastPoint[xIndex], pastPoint[yIndex], pastPoint[thetaIndex]), 0, 0)
        
        if thetaFunc is not None:
            x = xFunc(relativeTime)
            y = yFunc(relativeTime)
            theta = thetaFunc(relativeTime)
            dxdt = 0
            dydt = 0
        else:
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
    def controller_getFeedforwardTerm(self, relativeTime, xFunc, yFunc, thetaFunc, dxdt, dydt):
        # Considering bounds of path
        if (relativeTime < self.robotPath[0][timeIndex]) or (relativeTime >= self.robotPath[-1][timeIndex]):
            return (0, 0)
        
        if thetaFunc is not None:
            linearVelocity = 0
            angularVelocity = derivative(thetaFunc, relativeTime, n=1, dx=dt)
        else:
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
        errorV = self.robotError[xIndex] * Kpx + self.errorDiff_robot[xIndex] * Kdx
        errorOmega = self.robotError[yIndex] * Kpy + self.errorDiff_robot[yIndex] * Kdy + self.robotError[thetaIndex] * Kpth
        # print("ErrorV: " + str(errorV) + " ErrorOmega: " + str(errorOmega))
        return (errorV, errorOmega)

    def controller_getWheelVelocities(self):
        velLeft = self.robotCommand[xIndex] - constants.half_wheel_base * self.robotCommand[yIndex]
        velRight = self.robotCommand[xIndex] + constants.half_wheel_base * self.robotCommand[yIndex]
        return (velLeft, velRight)