/**
 * @file task_planner.cc
 * @author Prithu Pareek (ppareek@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "task_planner.hh"

TaskPlanner::TaskPlanner(int num_robots)
{
    this->num_robots = num_robots;
}

TaskPlanner::~TaskPlanner()
{
}

// TODO Implement this function with a proper algorithm
std::vector<int> TaskPlanner::assign_pallets_to_robots(std::vector<std::vector<double>> robot_poses,
                                                       std::vector<std::vector<double>> pallet_and_goal_poses)
{
    // Task assignment vector
    std::vector<int> task_assignments;

    // Loop through each robot and assign a task
    for (int i = 0; i < this->num_robots; i++)
    {
        task_assignments.push_back(i);
    }

    return task_assignments;
}
