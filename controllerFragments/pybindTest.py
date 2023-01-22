import os

# cd build and make the c++ code
os.system("cd build && make")

from build import planner

# init the planner
planner.Planner_Init(1)

# set the map size
map_size = (800, 400)

# set the robot poses
robot_poses = [(200, 200, 0)]

# set the pallet poses
pallet_poses = [(400, 200, 0)]

# set the goal poses
goal_poses = [(600, 200, 0)]

# plan the path
paths = planner.Planner_GeneratePaths(map_size, robot_poses, pallet_poses, goal_poses)

print("Done with test!")