/**
 * @file task_planner.hh
 * @author Prithu Pareek (ppareek@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef TASK_PLANNER_HH
#define TASK_PLANNER_HH

#include <iostream>
#include <vector>

class TaskPlanner
{
public:
    TaskPlanner(int num_robots);
    ~TaskPlanner();

    /**
     * @brief Returns the task assignment for the robots. Calls simple vs. heuristic task assignment based on the USE_HEURISTIC flag
     *
     * @param robot_poses
     * @param pallet_and_goal_poses
     * @return std::vector<int> A list of pallet indices for each robot. -1 if no task assigned.
     */
    std::vector<int> assign_pallets_to_robots(std::vector<std::vector<double>> robot_poses,
                                              std::vector<std::vector<double>> pallet_and_goal_poses);

    /**
     * @brief Returns the task assignment for the robots using a simple algorithm
     *
     * @param robot_poses
     * @param pallet_and_goal_poses
     * @return std::vector<int> A list of pallet indices for each robot. -1 if no task assigned.
     */
    std::vector<int> assign_pallets_to_robots_simple(std::vector<std::vector<double>> robot_poses,
                                                     std::vector<std::vector<double>> pallet_and_goal_poses);

    /**
     * @brief Returns the task assignment for the robots using a euclidean distance algorithm
     *
     * @param robot_poses
     * @param pallet_and_goal_poses
     * @return std::vector<int> A list of pallet indices for each robot. -1 if no task assigned.
     */
    std::vector<int> assign_pallets_to_robots_heuristic(std::vector<std::vector<double>> robot_poses,
                                                        std::vector<std::vector<double>> pallet_and_goal_poses);

private:
    int num_robots;

    // vector of dropped off pallets
    std::vector<int> dropped_off_pallets;
};

#endif // TASK_PLANNER_HH