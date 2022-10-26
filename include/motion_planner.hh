/**
 * @file planner.hh
 * @author Prithu Pareek (ppareek@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-09-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MOTION_PLANNER_HH
#define MOTION_PLANNER_HH

#include <iostream>
#include <vector>
#include "node.hh"
#include <map>
#include <cmath>

class MotionPlanner
{
public:
    MotionPlanner(int robot_index);
    ~MotionPlanner();

    /**
     * @brief Plans the path for the robot
     *
     * @return int 0 if successful, -1 otherwise
     */
    int plan();

    /**
     * @brief Set the start position for the robot
     *
     * @param start (x, y, theta)
     */
    void set_start(std::vector<double> start);

    /**
     * @brief Set the two goals for the robot (pallet and drop-off)
     *
     * @param goal (pallet_x, pallet_y, pallet_theta, goal_x, goal_y, goal_theta)
     */
    void set_goals(std::vector<double> goals);

    /**
     * @brief Set the planner map.
     *
     * @param map A 2D vector of ints representing the map. 0 - free, 1 - obstacle.
     * Each cell is 1 cm x 1 cm.
     * Map is x-axis first, y-axis second.
     */
    void set_map(std::vector<std::vector<int>> map);

    /**
     * @brief Set the robot robot paths vector used for time-based collision checking.
     *
     * @param robot_paths A list of paths for each robot. Path: (x, y, theta, time, tag).
     * Tag: 0 - driving, 1 - at pallet, 2 - at goal.
     */
    void set_robot_paths(std::vector<std::vector<std::vector<double>>> robot_paths);

    /**
     * @brief Get the planned path for the robot
     *
     * @return std::vector<std::vector<double>> Path: (x, y, theta, time, tag).
     * Tag: 0 - driving, 1 - at pallet, 2 - at goal.
     */
    std::vector<std::vector<double>> get_path();

    /**
     * @brief Print the planned path for the robot
     *
     */
    void print_path();

    /**
     * @brief The index of the robot
     */
    int robot_index;

private:
    std::vector<std::vector<int>> map;
    std::vector<double> start;
    std::vector<double> pallet_goal;
    std::vector<double> dropoff_goal;

    /**
     * @brief Path for the robot. Path: (x, y, theta, time, tag).
     * Tag: 0 - driving, 1 - at pallet, 2 - at goal.
     */
    std::vector<std::vector<double>> path;

    /**
     * @brief A list of paths for each robot. Path: (x, y, theta, time, tag).
     * Tag: 0 - driving, 1 - at pallet, 2 - at goal.
     */
    std::vector<std::vector<std::vector<double>>> paths;

    /**
     * @brief Runs the A* algorithm to find a path from start to goal
     *
     * @param is_pallet_goal true if the goal is the pallet, false if the goal is the drop-off
     * @return int 0 if successful, -1 otherwise
     */
    int a_star(bool is_pallet_goal);

    /**
     * @brief Get the neighbors of a node given the holonomic constraints of the robot.
     *
     * @param curr_vd Current position of the robot in the map. Row major indexing.
     * @return std::vector<int> A list of map indexes of the neighbors.
     */
    std::vector<Node *> get_neighbors(Node *curr_node, bool is_pallet_goal);

    /**
     * @brief Checks if the robot is in static or dynamic collision.
     *
     * @param vd Position of the robot in the map. Row major indexing.
     * @param time Time the robot is at the position.
     * @return true Is in static or dynamic collision.
     * @return false Is not in static or dynamic collision.
     */
    bool is_in_collision(Node *node, bool is_pallet_goal);

    // map between discretized angle (between 0 and 7) and list of dx, dy, theta. 0 points to the right.
    std::map<int, std::vector<std::vector<double>>> angle_to_dxdythetas = {
        {0, {{1, 0, 0.0}, {1, -1, M_PI / 4}}},
        {1, {{1, -1, M_PI / 4}, {0, -1, M_PI / 2}}},
        {2, {{0, -1, M_PI / 2}, {-1, -1, 3 * M_PI / 4}}},
        {3, {{-1, -1, 3 * M_PI / 4}, {-1, 0, M_PI}}},
        {4, {{-1, 0, M_PI}, {-1, 1, 5 * M_PI / 4}}},
        {5, {{-1, 1, 5 * M_PI / 4}, {0, 1, 3 * M_PI / 2}}},
        {6, {{0, 1, 3 * M_PI / 2}, {1, 1, 7 * M_PI / 4}}},
        {7, {{1, 1, 7 * M_PI / 4}, {1, 0, 0.0}}}};
};

#endif // MOTION_PLANNER_HH