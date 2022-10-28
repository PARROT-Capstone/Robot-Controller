/**
 * @file planner.cc
 * @author Prithu Pareek (ppareek@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "planner.hh"
#include <cmath>

Planner::Planner(int num_robots)
{
    this->num_robots = num_robots;

    // Create planners for each robot
    for (int i = 0; i < num_robots; i++)
    {
        MotionPlanner *p = new MotionPlanner(i);
        this->motion_planners.push_back(p);
    }

    // Create a task planner
    this->task_planner = new TaskPlanner(num_robots);
}

Planner::~Planner()
{
}

void Planner::set_map(std::vector<std::vector<int>> map)
{
    this->map = map;

    // Set the map for each robot
    for (int i = 0; i < this->num_robots; i++)
    {
        this->motion_planners[i]->set_map(map);
    }
}

void Planner::set_robot_poses(std::vector<std::vector<double>> robot_poses)
{
    // set this->robot_poses to robot_poses divided by 10 to convert from mm to cm (round to nearest int), skip the last element
    for (int i = 0; i < robot_poses.size(); i++)
    {
        std::vector<double> robot_pose;
        for (int j = 0; j < robot_poses[i].size() - 1; j++)
        {
            robot_pose.push_back(round(robot_poses[i][j] / 10));
        }
        // push the last element of robot_poses[i] to robot_pose (theta)
        robot_pose.push_back(robot_poses[i][robot_poses[i].size() - 1]);
        this->robot_poses.push_back(robot_pose);
    }
}

void Planner::set_pallet_and_goal_poses(std::vector<std::vector<double>> pallet_and_goal_poses)
{
    // Set this->pallet_and_goal_poses to pallet_and_goal_poses divided by 10 to convert from mm to cm (round to nearest int), don't convert the thetas
    for (int i = 0; i < pallet_and_goal_poses.size(); i++)
    {
        std::vector<double> pallet_and_goal_pose;
        for (int j = 0; j < pallet_and_goal_poses[i].size(); j++)
        {
            if (j == 2 || j == 5)
            {
                pallet_and_goal_pose.push_back(pallet_and_goal_poses[i][j]);
            }
            else
            {
                pallet_and_goal_pose.push_back(round(pallet_and_goal_poses[i][j] / 10));
            }
        }
        this->pallet_and_goal_poses.push_back(pallet_and_goal_pose);
    }
}

int Planner::plan()
{
    // Assign tasks to robots
    std::vector<int> task_assignments = this->task_planner->assign_pallets_to_robots(this->robot_poses, this->pallet_and_goal_poses);

    // init the paths vector
    this->paths = std::vector<std::vector<std::vector<double>>>(this->num_robots);

    // Plan paths for each robot
    for (int i = 0; i < this->num_robots; i++)
    {
        std::cout << "Planning for robot " << i << std::endl;

        if (task_assignments[i] == -1)
        {
            // No task assigned to this robot
            std::cout << "No task assigned to this robot" << std::endl;
            continue;
        }

        this->motion_planners[i]->set_start(this->robot_poses[i]);
        this->motion_planners[i]->set_goals(this->pallet_and_goal_poses[task_assignments[i]]);
        this->motion_planners[i]->set_robot_paths(this->paths);

        if (this->motion_planners[i]->plan() == -1)
        {
            std::cout << "Planning failed for robot " << i << std::endl;
            return -1;
        }
        std::cout << "Planning successful for robot " << i << std::endl;
        this->paths[i] = this->motion_planners[i]->get_path();
    }

    return 0;
}

std::vector<std::vector<std::vector<double>>> Planner::get_paths()
{
    // return the paths for all the robots, convert back to cm from mm
    std::vector<std::vector<std::vector<double>>> paths;
    for (int i = 0; i < this->paths.size(); i++)
    {
        std::vector<std::vector<double>> path;
        for (int j = 0; j < this->paths[i].size(); j++)
        {
            std::vector<double> pose;
            for (int k = 0; k < this->paths[i][j].size(); k++)
            {
                if (k == 0 || k == 1)
                {
                    pose.push_back(this->paths[i][j][k] * 10);
                }
                else
                {
                    pose.push_back(this->paths[i][j][k]);
                }
            }
            path.push_back(pose);
        }
        paths.push_back(path);
    }
    return paths;
}

void Planner::print_paths()
{
    for (int i = 0; i < this->num_robots; i++)
    {
        std::cout << "Printing path for robot " << i << std::endl;
        this->motion_planners[i]->print_path();
    }
}