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

    // Create task assigments vector full of -1s
    std::vector<int> task_assignments(this->num_robots, -1);

    std::vector<bool> robot_assigned(robot_poses.size(), false);

    // create a vector of distances from each pallet to all robots
    std::vector<std::vector<double>> pallet_to_robot_distances;
    for (int i = 0; i < pallet_and_goal_poses.size(); i++)
    {
        std::vector<double> pallet_to_robot_distance;
        for (int j = 0; j < robot_poses.size(); j++)
        {
            pallet_to_robot_distance.push_back(sqrt(pow(pallet_and_goal_poses[i][0] - robot_poses[j][0], 2) + pow(pallet_and_goal_poses[i][1] - robot_poses[j][1], 2)));
        }
        pallet_to_robot_distances.push_back(pallet_to_robot_distance);
    }

    // loop through each pallet and assign it to the closest robot
    for (int i = 0; i < pallet_and_goal_poses.size(); i++)
    {
        // make sure the pallet is not already dropped off
        if (std::find(this->dropped_off_pallets.begin(), this->dropped_off_pallets.end(), i) != this->dropped_off_pallets.end())
        {
            continue;
        }
        // find the closest robot to the pallet
        int closest_robot = -1;
        double closest_robot_distance = 1000000;
        for (int j = 0; j < robot_poses.size(); j++)
        {
            if (pallet_to_robot_distances[i][j] < closest_robot_distance && !robot_assigned[j])
            {
                closest_robot = j;
                closest_robot_distance = pallet_to_robot_distances[i][j];
            }
        }
        if (closest_robot != -1)
        {
            // assign the pallet to the closest robot
            task_assignments[closest_robot] = i;
            robot_assigned[closest_robot] = true;
            this->dropped_off_pallets.push_back(i);
        }
    }

    return task_assignments;
}

void TaskPlanner::assign_pallets_to_robots(std::vector<std::vector<double>> robot_poses,
                                           std::vector<std::vector<double>> pallet_and_goal_poses)
{
    if (USE_HEURISTIC)
    {
        this->task_assignments = this->assign_pallets_to_robots_heuristic(robot_poses, pallet_and_goal_poses);
    }
    else
    {
        this->task_assignments = this->assign_pallets_to_robots_simple(robot_poses, pallet_and_goal_poses);
    }
}

void TaskPlanner::unassign_pallet_from_robot(int robot_id)
{
    // unassign the pallet from the robot
    int pallet_id = this->task_assignments[robot_id];
    this->task_assignments[robot_id] = -1;

    // find the index of the pallet in the dropped off pallets vector
    int index = -1;
    for (int i = 0; i < this->dropped_off_pallets.size(); i++)
    {
        if (this->dropped_off_pallets[i] == pallet_id)
        {
            index = i;
            break;
        }
    }

    // remove the pallet from the dropped off pallets vector
    if (index != -1)
    {
        this->dropped_off_pallets.erase(this->dropped_off_pallets.begin() + index);
    }
}

std::vector<int> TaskPlanner::get_task_assignments()
{
    return this->task_assignments;
}