from posixpath import pathsep
from constants import CV_SANDBOX_HEIGHT, CV_SANDBOX_WIDTH
import constants
import requests
import mainHelper
from cv_main import CV
from controller_main import Controller
import threading
import time as time
import math
import os

# cd build and make the c++ code
os.system("cd build && make")

from build import planner

computerVision = CV()

# call this when the cameras are setup
computerVision.cv_InitComputerVision()

computerVision.cv_runLocalizer()
robotPoses, robotfiducialIds = computerVision.cv_GetRobotPositions()

# init the planner
planner.Planner_Init(len(robotPoses))

map_size = (CV_SANDBOX_WIDTH, CV_SANDBOX_HEIGHT)

while(True):
        
    computerVision.cv_runLocalizer()
    robotPoses, robotfiducialIds = computerVision.cv_GetRobotPositions()

    # print robotPoses
    for pose in robotPoses:
        print("Robot Pose: ", pose)

    palletPoses = computerVision.cv_GetPalletPositions()

    # set the goal poses to be palletPoses - 100mm in the x direction
    goalPoses = computerVision.cv_GetGoalFiducials()

    # print out the goalPoses
    for pose in goalPoses:
        print("Goal Pose: ", pose)

    paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, goalPoses)

    for path in paths:
        mainHelper.preconditionPath(path)

    threads = []
    robotNumber = len(paths)
    robotCommands = [(0,0,constants.ELECTROMAGNET_DONT_SEND) for _ in range(robotNumber)]
    import numpy as np
    sendCommands = np.zeros(robotNumber, dtype=bool)
    def Main_RequestsThreading(robotId):
        fiducialId = robotfiducialIds[robotId]
        robotHwNumber = constants.ROBOT_HARDWARE_NUMBERS[constants.ROBOT_FIDUCIALS.index(fiducialId)]
        url = f"http://parrot-robot{robotHwNumber}.wifi.local.cmu.edu"
        session = requests.Session()
        session.headers.update({'Connection': 'Keep-Alive', 'Keep-Alive': "timeout=5, max=1000000"})
        while True:
            sendCommands[robotId] = False
            velLeftLinear = robotCommands[robotId][0]
            velRightLinear = robotCommands[robotId][1]
            velLeftAng = velLeftLinear / constants.wheel_radius
            velRightAng = velRightLinear / constants.wheel_radius
            electromagnet_command = robotCommands[robotId][2]

            offset = 2

            # convert angular velocity to pwm
            scaleFactor = 7.5 # 1 PWM Duty Cycle = 7.5 mm/s


            leftPWM = 90 + (velLeftAng * scaleFactor)
            if velLeftAng > 0:
                leftPWM += offset
            elif velLeftAng < 0:
                leftPWM -= offset

            
            rightPWM = 90 - (velRightAng * scaleFactor)
            if velRightAng > 0:
                rightPWM -= offset
            elif velRightAng < 0:
                rightPWM += offset

            
            if (electromagnet_command != constants.ELECTROMAGNET_DONT_SEND):
                emJson = {"dtype": "pallet",
                        "power": (1 if electromagnet_command == constants.ELECTROMAGNET_ENABLE else 0)}
                session.post(url, data=emJson)

            servoJson = {"dtype": "speed", 
                        "servo1": int(leftPWM),
                        "servo2": int(rightPWM)
                    }
            session.post(url, data=servoJson)

    for i in range(robotNumber):
        thread = threading.Thread(target=Main_RequestsThreading, args=(i,))
        threads.append(thread)
        thread.start()

    # initialize controllers when path planner is done
    controllers = [Controller(i, paths[i]) for i in range(robotNumber)]
    allControllersDone = True

    # control loop
    while True:
        start = time.time()
        computerVision.cv_runLocalizer()
        robotPoses, _ = computerVision.cv_GetRobotPositions()
        # print("CV Framerate: ", 1/(time.time() - start))
        palletPoses = computerVision.cv_GetPalletPositions() # NOTE right now we are using fiducial ID 2
        targetPoses = []
        for i in range(robotNumber):#TODO: change later
            robotCommand, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
            targetPoses.append(targetPose)
            robotCommands[i] = robotCommand
            velLeftLinear, velRightLinear, electromagnet_command = robotCommand
            allControllersDone = allControllersDone and controllers[i].finishedController

        computerVision.cv_visualize(paths, targetPoses, velRightLinear, velLeftLinear, ffterm, fbkterm)
        end = time.time()
        # print("Frame Rate: ", 1 / (end - start))

        if allControllersDone:
            print("Finished main loop")
            time.sleep(0.5)
            # implement call to backup and rotate
            
            computerVision.cv_runLocalizer()
            robotPoses, _ = computerVision.cv_GetRobotPositions()
                        
            backupPaths = []
            for robotId in range(robotNumber):
                startPoint = robotPoses[robotId]
                startPoint.append(0)
                startPoint.append(0) # initial timestep and tag
                backupPoint = startPoint.copy()
                backupPoint[0] = backupPoint[0] - np.cos(backupPoint[2]) * 75
                backupPoint[1] = backupPoint[1] + np.sin(backupPoint[2]) * 75
                backupPoint[3] += 2
                
                rotatePoint = backupPoint.copy()
                if rotatePoint[2] >= 0:
                    rotatePoint[2] = rotatePoint[2] - math.pi
                else:
                    rotatePoint[2] = rotatePoint[2] + math.pi

                    
                rotatePoint[3] += 3
                
                endPoint = rotatePoint.copy()
                distance = 50
                endPoint[0] = endPoint[0] + distance * math.cos(endPoint[2])
                endPoint[1] = endPoint[1] - distance * math.sin(endPoint[2])
                endPoint[3] = endPoint[3] + 2
                
                controllers[robotId] = Controller(robotId, [startPoint, backupPoint, rotatePoint, endPoint])
                
            # run controller for 7 seconds to rotate
            startRotate = time.time()
            while(time.time() - startRotate < 7):
                computerVision.cv_runLocalizer()
                robotPoses, _ = computerVision.cv_GetRobotPositions()
                targetBackupPoses = []
                for i in range(robotNumber):
                    robotCommand, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
                    targetBackupPoses.append(targetPose)
                    robotCommands[i] = robotCommand
                    velLeftLinear, velRightLinear, electromagnet_command = robotCommand

                computerVision.cv_visualize(paths, targetBackupPoses, velRightLinear, velLeftLinear, ffterm, fbkterm)
                
            break
        #exit and replan all the paths
        allControllersDone = True