/**
 * @file planner.hh
 * @author Prithu Pareek (ppareek@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PLANNER_HH
#define PLANNER_HH

#include <iostream>
#include <vector>

#include "motion_planner.hh"
#include "task_planner.hh"

class Planner
{
public:
    Planner(int num_robots);
    ~Planner();

    /**
     * @brief Plans for all the robots
     *
     * @return int 0 if successful, -1 otherwise
     */
    int plan();

    /**
     * @brief Set the planner map.
     *
     * @param map A 2D vector of ints representing the map. 0 - free, 1 - obstacle. Each cell is 1 cm x 1 cm.
     * Map is x-axis first, y-axis second.
     */
    void set_map(std::vector<std::vector<int>> map);

    /**
     * @brief Set the robot start locations.
     *
     * @param robot_poses (x, y, theta) for each robot.
     */
    void set_robot_poses(std::vector<std::vector<double>> robot_poses);

    /**
     * @brief Set the location of each pallet and their corresponding goal locations.
     *
     * @param pallet_and_goal_poses (pallet_x, pallet_y, pallet_theta, goal_x, goal_y, goal_theta) for each pallet.
     */
    void set_pallet_and_goal_poses(std::vector<std::vector<double>> pallet_and_goal_poses);

    /**
     * @brief Get the planned paths for all the robots.
     *
     * @return std::vector<std::vector<std::vector<double>>> A list of paths for each robot. Path: (x, y, theta, time, tag).
     * Tag: 0 - driving, 1 - at pallet, 2 -  * at goal.
     */
    std::vector<std::vector<std::vector<double>>> get_paths();

    /**
     * @brief Print the paths for all the robots.
     *
     */
    void print_paths();

private:
    int num_robots;
    std::vector<std::vector<int>> map;
    std::vector<std::vector<double>> robot_poses;
    std::vector<std::vector<double>> pallet_and_goal_poses;

    /**
     * @brief A list of paths for each robot. Path: (x, y, theta, time, tag).
     * Tag: 0 - driving, 1 - at pallet, 2 -  * at goal.
     *
     */
    std::vector<std::vector<std::vector<double>>> paths;

    std::vector<MotionPlanner *> motion_planners;

    TaskPlanner *task_planner;
};

#endif // PLANNER_HH