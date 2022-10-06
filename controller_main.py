import numpy as np
import time
import math
import mainHelper
import constants

Kpx = 0.3
Kpy = 0.002
Kpth = 0.1
Ki = 0
Kd = 0

# Based on Mobile Robot Programming Lab and https://www.researchgate.net/profile/Muhammad-Asif-62/publication/266798448_20140930101805712/links/543cc55d0cf24ef33b7639cb/20140930101805712.pdf

class Controller:
    def __init__(self, robotId):
        self.startTime = time.time()
        self.robotId = robotId
        self.robotPath = mainHelper.Main_getRobotPaths(self.robotId)
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
        # path is (x, y, theta, relative_time)
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
    def controller_getFeedforwardTerm(self, targetPose_global, relativeTime):
        previousPose_global = self.controller_getNextTargetPoint(self.last_position_time - self.startTime)
        linearVelocity = 0
        angularVelocity = 0
        deltaTime = targetPose_global[3] - previousPose_global[3]
        if (deltaTime == 0):
            return (linearVelocity, angularVelocity)
        angularVelocity = (targetPose_global[2] - previousPose_global[2]) / deltaTime
        deltaX = targetPose_global[1] - previousPose_global[1]
        deltaY = targetPose_global[0] - previousPose_global[0]
        linearVelocity = math.sqrt( (deltaX/deltaTime)**2 + (deltaY/deltaTime)**2 )
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