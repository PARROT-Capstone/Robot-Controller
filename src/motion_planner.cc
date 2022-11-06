/**
 * @file planner.cc
 * @author Prithu Pareek (ppareek@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-09-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "motion_planner.hh"
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <chrono>

MotionPlanner::MotionPlanner(int robot_index)
{
    this->robot_index = robot_index;
}

MotionPlanner::~MotionPlanner()
{
}

void MotionPlanner::set_start(std::vector<double> start)
{
    this->start = start;
    // clamp the theta to be between 0 and 2pi
    if (this->start[2] < 0)
    {
        this->start[2] += 2 * M_PI;
    }
    else if (this->start[2] > 2 * M_PI)
    {
        this->start[2] -= 2 * M_PI;
    }
}

void MotionPlanner::set_goals(std::vector<double> goals)
{
    // parse the goals vector into pallet and dropoff goals
    this->pallet_goal = {goals[0], goals[1], goals[2]};
    this->dropoff_goal = {goals[3], goals[4], goals[5]};

    // clamp the theta to be between 0 and 2pi
    if (this->pallet_goal[2] < 0)
    {
        this->pallet_goal[2] += 2 * M_PI;
    }
    else if (this->pallet_goal[2] > 2 * M_PI)
    {
        this->pallet_goal[2] -= 2 * M_PI;
    }

    if (this->dropoff_goal[2] < 0)
    {
        this->dropoff_goal[2] += 2 * M_PI;
    }
    else if (this->dropoff_goal[2] > 2 * M_PI)
    {
        this->dropoff_goal[2] -= 2 * M_PI;
    }
}

void MotionPlanner::set_robot_paths(std::vector<std::vector<std::vector<double>>> robot_paths)
{
    this->paths = robot_paths;
}

void MotionPlanner::set_map(std::vector<std::vector<int>> map)
{
    this->map = map;
}

std::vector<std::vector<double>> MotionPlanner::get_path()
{
    return this->path;
}

void MotionPlanner::print_path()
{
    for (auto &p : this->path)
    {
        std::cout << "x: " << p[0] << " y: " << p[1] << " theta: " << p[2] << " time: " << p[3] << " tag: " << p[4] << std::endl;
    }

    std::cout << std::endl;
}

std::vector<Node *> MotionPlanner::get_neighbors(Node *curr_node, bool is_pallet_goal)
{
    // Create a list of neighbors
    std::vector<Node *> neighbors;

    double theta = curr_node->theta;

    // Normalize theta to be between 0 and 2pi
    if (theta < 0)
    {
        theta += 2 * M_PI;
    }
    else if (theta > 2 * M_PI)
    {
        theta -= 2 * M_PI;
    }

    // Clamp the theta to one of 8 directions
    int dir = (int)floor(theta / (M_PI / 4));

    // Get the list of dx, dy, and theta for the neighbors
    std::vector<std::vector<double>> neighbor_dxdythetadtimecosts = this->angle_to_dxdythetadtimecosts[dir];

    // add rotate in place neighbors
    if (ALLOW_IN_PLACE_TURN)
    {
        for (int i = 0; i < 16; i++)
        {
            // if the robot is not at the pallet goal and not current rotation, then it can rotate in place
            if (i != dir)
            {
                neighbor_dxdythetadtimecosts.push_back({0, 0, i * M_PI / 8, TURN_TIME_STEP, IN_PLACE_TURN_COST});
            }
        }
    }

    // set the goal based on the pallet_goal or dropoff_goal
    std::vector<double> goal = is_pallet_goal ? this->pallet_goal : this->dropoff_goal;

    // For each neighbor, create a new node and add it to the list of neighbors
    for (auto &neighbor_dxdythetadtimecost : neighbor_dxdythetadtimecosts)
    {
        double dx = neighbor_dxdythetadtimecost[0];
        double dy = neighbor_dxdythetadtimecost[1];
        double theta = neighbor_dxdythetadtimecost[2];
        double time = neighbor_dxdythetadtimecost[3];
        double cost = neighbor_dxdythetadtimecost[4];

        // calculate the g and h values for the neighbor
        double g = curr_node->g_value + cost;
        // calculate h value using euclidean distance
        double h = sqrt(pow(goal[0] - (curr_node->x + dx), 2) + pow(goal[1] - (curr_node->y + dy), 2) + pow(goal[2] - (curr_node->theta + theta), 2));

        // Do not allow turns if the robot is within threshold of the goal
        if (h <= NO_TURN_THRESH && (cost == IN_PLACE_TURN_COST || cost == TURN_COST))
        {
            continue;
        }

        // Create a new node
        Node *neighbor = new Node(curr_node->x + (int)dx, curr_node->y + (int)dy, theta, curr_node->time + time, g, h, curr_node);

        // Add the neighbor to the list of neighbors
        neighbors.push_back(neighbor);
    }

    return neighbors;
}

bool MotionPlanner::is_in_collision(Node *node, bool is_pallet_goal)
{
    // set the inflation radius based on the goal type
    int robot_inflation_radius = is_pallet_goal ? ROBOT_FOOTPRINT_RADIUS : ROBOT_FOOTPRINT_RADIUS_WITH_PALLET;

    // calculate the x and y bounds of the robot given the inflation radius
    int x_min = node->x - robot_inflation_radius;
    int x_max = node->x + robot_inflation_radius;
    int y_min = node->y - robot_inflation_radius;
    int y_max = node->y + robot_inflation_radius;

    // calculate the x and y bounds of the pallet goal given the inflation radius
    int pallet_x_min = this->pallet_goal[0] - PALLET_FOOTPRINT_RADIUS;
    int pallet_x_max = this->pallet_goal[0] + PALLET_FOOTPRINT_RADIUS;
    int pallet_y_min = this->pallet_goal[1] - PALLET_FOOTPRINT_RADIUS;
    int pallet_y_max = this->pallet_goal[1] + PALLET_FOOTPRINT_RADIUS;

    // offset the robot center towards the front of the robot by a specified amount
    int robot_offset_x = (int)(ROBOT_OFFSET * cos(node->theta)) + node->x;
    int robot_offset_y = (int)(ROBOT_OFFSET * sin(node->theta)) + node->y;

    // calculate the x and y bounds of the front of the robot given the inflation radius
    int robot_front_x_min = robot_offset_x - PALLET_FOOTPRINT_RADIUS;
    int robot_front_x_max = robot_offset_x + PALLET_FOOTPRINT_RADIUS;
    int robot_front_y_min = robot_offset_y - PALLET_FOOTPRINT_RADIUS;
    int robot_front_y_max = robot_offset_y + PALLET_FOOTPRINT_RADIUS;

    // ensure the robot is within the map bounds
    if (x_min < 0 || x_max >= this->map.size() || y_min < 0 || y_max >= this->map[0].size())
    {
        return true;
    }

    // check if the robot is in collision with any obstacles
    // check if any static obstacles are in the robot's footprint
    for (int x = x_min; x <= x_max; x++)
    {
        for (int y = y_min; y <= y_max; y++)
        {
            if (this->map[x][y] == 1)
            {
                // if the pallet is not the robots pallet, then the robot is in collision
                if (x < pallet_x_min || x > pallet_x_max || y < pallet_y_min || y > pallet_y_max)
                {
                    return true;
                }

                // if we have reached this point, we know we are checking for a collision between the robot and it's assigned pallet as it is trying to pick it up
                // only check for self pallet collisions before the robot has picked up the pallet
                // if (is_pallet_goal)
                // {
                //     // if the x and y are not within the front of the robot, then the robot is in collision
                //     if (x < robot_front_x_min || x > robot_front_x_max || y < robot_front_y_min || y > robot_front_y_max)
                //     {
                //         return true;
                //     }
                // }
            }
        }
    }

    // check if the robot is in

    // // Check if the robot is in dynamic collision with other robots.
    // for (auto &robot_path : this->paths)
    // {
    //     // For each point in the robot path
    //     for (auto &path_point : robot_path)
    //     {
    //         // If the robot is in a 1 second window of the path point, check for collision
    //         if (abs(node->time - path_point[3]) <= 1)
    //         {
    //             // Create a footprint for the other robot at the path point, inflated by 2
    //             std::vector<std::vector<int>> other_robot_footprint;
    //             for (int x = path_point[0] - 2; x <= path_point[0] + 2; x++)
    //             {
    //                 for (int y = path_point[1] - 2; y <= path_point[1] + 2; y++)
    //                 {
    //                     other_robot_footprint.push_back({x, y});
    //                 }
    //             }

    //             // Check if the two footprints overlap
    //             for (auto &p : robot_footprint)
    //             {
    //                 if (std::find(other_robot_footprint.begin(), other_robot_footprint.end(), p) != other_robot_footprint.end())
    //                 {
    //                     return true;
    //                 }
    //             }
    //         }
    //     }
    // }

    // If no collision, return false
    return false;
}

int MotionPlanner::a_star(bool is_pallet_goal)
{

    // set the goal based on the pallet_goal or dropoff_goal
    std::vector<double> goal = is_pallet_goal ? this->pallet_goal : this->dropoff_goal;
    // set the start based on the pallet_goal or dropoff_goal
    std::vector<double> start = is_pallet_goal ? this->start : this->path.back();
    // set the relative start time based on the pallet_goal or dropoff_goal
    double start_time = is_pallet_goal ? 0 : (this->path.back()[3] + 5);

    // Create the open list of Nodes
    std::priority_queue<Node *, std::vector<Node *>, OpenListNodeComparator> open_list = std::priority_queue<Node *, std::vector<Node *>, OpenListNodeComparator>();

    // Create the start and goal nodes
    double start_h_value = sqrt(pow(goal[0] - start[0], 2) + pow(goal[1] - start[1], 2) + pow(goal[2] - start[2], 2));
    Node *start_node = new Node((int)start[0], (int)start[1], start[2], start_time, 0, start_h_value, nullptr);
    double goal_h_value = 0;
    Node *goal_node = new Node((int)goal[0], (int)goal[1], goal[2], 0.0, 0, goal_h_value, nullptr);

    // Print the start and goal nodes for debugging
    std::cout << "Start Node: " << start_node->x << ", " << start_node->y << ", " << start_node->theta << ", " << start_node->time << ", " << start_node->g_value << ", " << start_node->h_value << std::endl;
    std::cout << "Goal Node: " << goal_node->x << ", " << goal_node->y << ", " << goal_node->theta << ", " << goal_node->time << ", " << goal_node->g_value << ", " << goal_node->h_value << std::endl;

    // if goal is in collision, return
    if (this->is_in_collision(goal_node, is_pallet_goal) && !is_pallet_goal)
    {
        std::cout << "Goal is in collision" << std::endl;
        return -1;
    }

    // Add the start node to the open list
    open_list.push(start_node);

    // Create the closed list of Nodes
    std::unordered_set<Node *, NodeHasher, ClosedListNodeComparator> closed_list = std::unordered_set<Node *, NodeHasher, ClosedListNodeComparator>();

    // Create a timer to limit the search time
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // while the open list is not empty
    while (!open_list.empty())
    {
        // Print the size of the open list
        // std::cout << "Open list size: " << open_list.size() << std::endl;

        // Get the node with the lowest f value
        Node *current_node = open_list.top();
        open_list.pop();

        // keep rempving nodes from the open list until the node is not in the closed list
        while (closed_list.find(current_node) != closed_list.end())
        {
            current_node = open_list.top();
            open_list.pop();
        }

        // Add the current node to the closed list
        closed_list.insert(current_node);

        // check if the current node x, y, and theta are the same as the goal x, y, and theta
        if (current_node->x == goal_node->x && current_node->y == goal_node->y)
        {
            // check if the goal angle is within pi/4 radians of the current node angle
            if (abs(current_node->theta - goal_node->theta) <= M_PI / 4)
            {
                goal_node = current_node;
                break;
            }

            // goal_node = current_node;
            // break;
        }

        // Get the neighbors of the current node
        std::vector<Node *> neighbors = get_neighbors(current_node, is_pallet_goal);

        // Print the number of neighbors
        // std::cout << "Number of neighbors: " << neighbors.size() << std::endl;

        // for each neighbor
        for (Node *neighbor : neighbors)
        {
            // check if the neighbor is in collision with an obstacle
            if (this->is_in_collision(neighbor, is_pallet_goal))
            {
                // if yes, then skip this neighbor
                continue;
            }

            // if the neighbor is not in the closed list add it to the open list
            if (closed_list.find(neighbor) == closed_list.end())
            {
                open_list.push(neighbor);
            }
        }

        // Check if the search time has exceeded the limit
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() > 10)
        {
            std::cout << "Search time exceeded" << std::endl;
            break;
        }
    }

    // check if found the path
    if (goal_node->parent == nullptr)
    {
        std::cout << "Could not find the path" << std::endl;
        return -1;
    }

    // construct the path
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>();
    Node *current_node = goal_node;
    while (current_node != nullptr)
    {
        std::vector<double> path_node = {(double)(current_node->x),
                                         (double)(current_node->y),
                                         current_node->theta,
                                         current_node->time,
                                         0};
        path.push_back(path_node);
        current_node = current_node->parent;
    }

    // reverse the path
    std::reverse(path.begin(), path.end());

    // set the tag of the last node based on the pallet_goal or dropoff_goal
    path.back()[4] = is_pallet_goal ? 1 : 2;

    // append this subpath to the main path
    for (int i = 0; i < path.size(); i++)
    {
        this->path.push_back(path[i]);
    }

    return 0;
}

int MotionPlanner::plan()
{
    // clear the path
    this->path = {};

    // use A* to find the path for the pallet goal
    if (this->a_star(true) == -1)
    {
        std::cout << "No path found for pallet goal" << std::endl;
        return -1;
    }

    std::cout << "Pallet goal path found" << std::endl;

    // use A* to find the path for the dropoff goal
    if (this->a_star(false) == -1)
    {
        std::cout << "No path found for dropoff goal" << std::endl;
        return -1;
    }

    std::cout << "Dropoff goal path found" << std::endl;

    return 0;
}