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
#include "constants.hh"

TaskPlanner::TaskPlanner(int num_robots)
{
    this->num_robots = num_robots;
}

TaskPlanner::~TaskPlanner()
{
}

std::vector<int> TaskPlanner::assign_pallets_to_robots_simple(std::vector<std::vector<double>> robot_poses,
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

std::vector<int> TaskPlanner::assign_pallets_to_robots_heuristic(std::vector<std::vector<double>> robot_poses,
                                                                 std::vector<std::vector<double>> pallet_and_goal_poses)
{
    // Task assignment vector
    std::vector<int> task_assignments;

    // assign each robot to its closest pallet
    for (int i = 0; i < this->num_robots; i++)
    {
        // get the robot pose
        std::vector<double> robot_pose = robot_poses[i];
        // initialize the closest pallet index to -1
        int closest_pallet_index = -1;
        // initialize the closest pallet distance to -1
        double closest_pallet_distance = -1;
        // loop through each pallet and goal
        for (int j = 0; j < pallet_and_goal_poses.size(); j++)
        {
            // make sure the pallet is not already dropped off
            if (std::find(this->dropped_off_pallets.begin(), this->dropped_off_pallets.end(), j) == this->dropped_off_pallets.end())
            {
                // get the pallet pose
                std::vector<double> pallet_pose = pallet_and_goal_poses[j];
                // calculate the distance between the robot and the pallet
                double distance = sqrt(pow(robot_pose[0] - pallet_pose[0], 2) + pow(robot_pose[1] - pallet_pose[1], 2));
                // if the distance is less than the closest pallet distance, update the closest pallet distance and index
                if (distance < closest_pallet_distance || closest_pallet_distance == -1)
                {
                    closest_pallet_distance = distance;
                    closest_pallet_index = j;
                }
            }
        }

        // if the closest pallet index is not -1, add it to the task assignments vector
        if (closest_pallet_index != -1)
        {
            task_assignments.push_back(closest_pallet_index);
            // add the pallet to the dropped off pallets vector
            this->dropped_off_pallets.push_back(closest_pallet_index);
        }
        else
        {
            task_assignments.push_back(-1);
        }
    }

    return task_assignments;
}

std::vector<int> TaskPlanner::assign_pallets_to_robots(std::vector<std::vector<double>> robot_poses,
                                                       std::vector<std::vector<double>> pallet_and_goal_poses)
{
    if (USE_HEURISTIC)
    {
        return this->assign_pallets_to_robots_heuristic(robot_poses, pallet_and_goal_poses);
    }
    else
    {
        return this->assign_pallets_to_robots_simple(robot_poses, pallet_and_goal_poses);
    }
}
