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

    std::cout << "Dropped off pallets: ";
    for (int i = 0; i < this->dropped_off_pallets.size(); i++)
    {
        std::cout << this->dropped_off_pallets[i] << " ";
    }

    // Loop through each robot and assign a task
    for (int i = 0; i < this->num_robots; i++)
    {
        bool task_assigned = false;
        // loop through each pallet and goal
        for (int j = 0; j < pallet_and_goal_poses.size(); j++)
        {
            // make sure the pallet is not already dropped off
            if (std::find(this->dropped_off_pallets.begin(), this->dropped_off_pallets.end(), j) == this->dropped_off_pallets.end())
            {
                // assign the task to the robot
                task_assignments.push_back(j);
                // add the pallet to the dropped off pallets vector
                this->dropped_off_pallets.push_back(j);
                task_assigned = true;
                // break out of the loop
                break;
            }
        }
        // if no task was assigned, push -1
        if (!task_assigned)
        {
            task_assignments.push_back(-1);
        }
    }

    return task_assignments;
}
