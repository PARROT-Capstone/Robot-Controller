from build import planner

planner.run_planner_test()
print("Done with test!")

# Map size list (x,y) in mm
map_size = [500, 500]

# list of pallet poses (x,y,theta) in mm and radians
pallet_poses = [[100, 100, 0], [200, 200, 0]]

# list of robot poses (x,y,theta) in mm and radians
robot_poses = [[50, 50, 0], [50, 150, 0]]

# list of goal poses (x,y,theta) in mm and radians
goal_poses = [[400, 400, 0], [200, 400, 0]]

paths = planner.Planner_GeneratePaths(map_size, robot_poses, pallet_poses, goal_poses)

# print the paths for each robot
for i in range(len(paths)):
    print("Robot", i, "path:")
    for j in range(len(paths[i])):
        print(paths[i][j])