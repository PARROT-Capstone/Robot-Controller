from posixpath import pathsep
from constants import CV_SANDBOX_HEIGHT, CV_SANDBOX_WIDTH
import constants
import numpy as np
import requests
import mainHelper
from cv_main import CV
from controller_main import Controller
import threading
import time as time
import math
import os
import cv2 as cv
from multiprocessing import Process


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

# gloabls for viz
# init viz
paths = []
targetPoses = []
velRightLinears = []
velLeftLinears = []
robotPoses = []
run_thread = [False]
latestSandboxImage = [None]
vizImage = [None]

if __name__ == '__main__':

    while(True):
            
        computerVision.cv_runLocalizer()
        robotPoses, robotfiducialIds = computerVision.cv_GetRobotPositions()

        # print robotPoses
        for pose in robotPoses:
            print("Robot Pose: ", pose)

        palletPoses = computerVision.cv_GetPalletPositions()
        
        # print out pallet poses
        for pose in palletPoses:
            print("Pallet Pose: ", pose)

        # set the goal poses to be palletPoses - 100mm in the x direction
        goalPoses = computerVision.cv_GetGoalFiducials()

        # print out the goalPoses
        for pose in goalPoses:
            print("Goal Pose: ", pose)

        paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, goalPoses)
        # firstPoint = robotPoses[0].copy()
        # firstPoint.append(0)
        # firstPoint.append(0)
        # secondPoint = firstPoint.copy()
        # secondPoint[0] += 200
        # secondPoint[1] += 200
        # secondPoint[2] -= math.pi/2
        # secondPoint[3] += 15
        # thirdPoint = secondPoint.copy()
        # thirdPoint[2] += math.pi/2
        # thirdPoint[3] += 3 # 30 degrees a second
        # fourthPoint = thirdPoint.copy()
        # fourthPoint[0] += 200
        # fourthPoint[3] += 10 # correct speed

        # paths = [np.array([firstPoint, secondPoint, thirdPoint, fourthPoint])]

        for path in paths:
            mainHelper.preconditionPath(path)

        # Exit if there are no paths for the robots
        allPathsEmpty = True
        for path in paths:
            if len(path) > 2:
                allPathsEmpty = False
        if allPathsEmpty:
            print("All robots are done")
            exit()

        threads = []
        robotNumber = len(paths)
        robotCommands = [(0,0,constants.ELECTROMAGNET_DONT_SEND) for _ in range(robotNumber)]
        sendCommands = np.zeros(robotNumber, dtype=bool)
        def Main_RequestsThreading(robotId):
            fiducialId = robotfiducialIds[robotId]
            robotHwNumber = constants.ROBOT_HARDWARE_NUMBERS[constants.ROBOT_FIDUCIALS.index(fiducialId)]
            url = f"http://parrot-robot{robotHwNumber}.wifi.local.cmu.edu"
            session = requests.Session()
            session.headers.update({'Connection': 'Keep-Alive', 'Keep-Alive': "timeout=5, max=10000000000"})
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
                    
                leftPWM = min(max(leftPWM, 90-25), 90+25)
                rightPWM = min(max(rightPWM, 90-25), 90+25)

                
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

        # p = Process(target=cv_visualize_threaded, args=())
        # mainPID = os.getpid()
        # os.fork()
        # print(os.getpid())
        # if (os.getpid() != mainPID):
        #     p.run()
        # p.join()
        # thread = threading.Thread(target=cv_visualize_threaded, args=())
        # threads.append(thread)
        # thread.start()
    
            
        # control loop
        while True:  
            
            start = time.time()
            computerVision.cv_runLocalizer()
            robotPoses, _ = computerVision.cv_GetRobotPositions()
            palletPoses = computerVision.cv_GetPalletPositions() # NOTE right now we are using fiducial ID 2
            latestSandboxImage[0] = computerVision.cv_getLatestSandboxImage()
            targetPoses = []
            velRightLinears = []
            velLeftLinears = []
            run_thread[0] = False
            for i in range(robotNumber):#TODO: change later
                robotCommand, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
                targetPoses.append(targetPose)
                robotCommands[i] = robotCommand
                velLeftLinear, velRightLinear, electromagnet_command = robotCommand
                velRightLinears.append(velRightLinear)
                velLeftLinears.append(velLeftLinear)
                allControllersDone = allControllersDone and controllers[i].finishedController

            run_thread[0] = True
            computerVision.cv_visualize(paths, targetPoses, velRightLinears, velLeftLinears, ffterm, fbkterm)
            if (np.shape(vizImage[0]) > (0,0)):
                print(np.shape(vizImage[0]))
                cv.imshow("Visualizer", vizImage[0])
                cv.waitKey(1)
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
                    startPoint[2] += math.pi
                    startPoint[2] = math.atan2(math.sin(startPoint[2]), math.cos(startPoint[2]))
                    startPoint.append(0)
                    startPoint.append(0) # initial timestep and tag
                    backupPoint = startPoint.copy()
                    backupPoint[0] = backupPoint[0] + np.cos(backupPoint[2]) * constants.BACKUP_TIME * 10
                    backupPoint[1] = backupPoint[1] - np.sin(backupPoint[2]) * constants.BACKUP_TIME * 10
                    backupPoint[3] += constants.BACKUP_TIME

                    path = [startPoint, backupPoint]
                    pathDuration = backupPoint[3]
                    controllers[robotId] = Controller(robotId, path, False)
                    
                # run controller for 7 seconds to rotate
                startRotate = time.time()
                while(time.time() - startRotate < pathDuration + 1): # one extra second to close the error
                    computerVision.cv_runLocalizer()
                    robotPoses, _ = computerVision.cv_GetRobotPositions()
                    # TODO: change later
                    targetBackupPoses = []
                    velRightLinears = []
                    velLeftLinears = []
                    for i in range(robotNumber):
                        robotCommand, targetPose, ffterm, fbkterm = controllers[i].controller_getRobotVelocities(robotPoses[i])
                        targetBackupPoses.append(targetPose)
                        robotCommands[i] = robotCommand
                        velLeftLinear, velRightLinear, electromagnet_command = robotCommand
                        velRightLinears.append(velRightLinear)
                        velLeftLinears.append(velLeftLinear)

                    computerVision.cv_visualize(paths, targetBackupPoses, velRightLinears, velLeftLinears, ffterm, fbkterm)
                    
                break
            #exit and replan all the paths
            allControllersDone = True
