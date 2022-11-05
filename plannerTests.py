# print the data from a pickle file
import pickle
from constants import CV_SANDBOX_HEIGHT, CV_SANDBOX_WIDTH
import os
import matplotlib.pyplot as plt
import math
import random

# cd build and make the c++ code
os.system("cd build && make")

from build import planner

# # open a file, where you stored the pickled data
# file = open('workingPathPickle', 'rb')

# # read the data from the file
# [paths, robotPoses, palletPoses] = pickle.load(file)

# goalPoses = []
# for pose in palletPoses:
#     goalPoses.append([pose[0] - 300, pose[1], pose[2]])
    
# # close the file
# file.close()

def generateRandomPose(mapWidth, mapHeight):
    x = mapWidth * (random.random())
    y = mapHeight * (random.random())
    theta = 2 * math.pi * random.random()
    return [x, y, theta]

map_size = (CV_SANDBOX_WIDTH, CV_SANDBOX_HEIGHT)
# print the map size
print("Map size: " + str(map_size))

# generate a random start, pallet, and goal pose within the map
robotPoses = []
robotPoses.append(generateRandomPose(map_size[0], map_size[1]))
palletPoses = []
palletPoses.append(generateRandomPose(map_size[0], map_size[1]))
goalPoses = []
goalPoses.append(generateRandomPose(map_size[0], map_size[1]))

# print the robot poses
print("Robot poses:")
for pose in robotPoses:
    print(pose)
    
# print the pallet poses
print("Pallet poses:")
for pose in palletPoses:
    print(pose)
    
# print the goal poses
print("Goal poses:")
for pose in goalPoses:
    print(pose)

# plan a path from the robot poses to the goal poses
paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, goalPoses)

# # print the planned paths
# print("Paths:")
# for i in range(len(paths)):
#     print("Robot", i, "path:")
#     for j in range(len(paths[i])):
#         print(paths[i][j])
        
# visualize the paths using matplotlib
# create a figure of map size
plt.xlim(0, map_size[0])
plt.ylim(0, map_size[1])

# plot arrows for each point in the robot paths
                
# plot the pallet poses
plt.scatter([pose[0] for pose in palletPoses], [pose[1] for pose in palletPoses], c='orange')

#plot the goal poses
plt.scatter([pose[0] for pose in goalPoses], [pose[1] for pose in goalPoses], c='red')

#plot the robot poses
plt.scatter([pose[0] for pose in robotPoses], [pose[1] for pose in robotPoses], c='green')

color = 'r'
for path in paths:
    for pose in path:
        if pose[4] == 1:
            color = 'b'
        plt.arrow(pose[0], pose[1], 5*math.cos(pose[2]), -5*math.sin(pose[2]), head_width=2, head_length=2, fc='r', ec=color)
        
# draw the figure
plt.show()

# # test the planner on a bunch of random start, pallet, and goal poses
# for i in range(25):
#     # generate a random start, pallet, and goal pose within the map
#     robotPoses = []
#     robotPoses.append(generateRandomPose(map_size[0], map_size[1]))
#     palletPoses = []
#     palletPoses.append(generateRandomPose(map_size[0], map_size[1]))
#     goalPoses = []
#     goalPoses.append(generateRandomPose(map_size[0], map_size[1]))

#     # print the robot poses
#     print("Robot poses:")
#     for pose in robotPoses:
#         print(pose)
        
#     # print the pallet poses
#     print("Pallet poses:")
#     for pose in palletPoses:
#         print(pose)
        
#     # print the goal poses
#     print("Goal poses:")
#     for pose in goalPoses:
#         print(pose)

#     # plan a path from the robot poses to the goal poses
#     paths = planner.Planner_GeneratePaths(map_size, robotPoses, palletPoses, goalPoses)
    
    
    