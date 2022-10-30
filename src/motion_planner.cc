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
}

void MotionPlanner::set_goals(std::vector<double> goals)
{
    // parse the goals vector into pallet and dropoff goals
    this->pallet_goal = {goals[0], goals[1], goals[2]};
    this->dropoff_goal = {goals[3], goals[4], goals[5]};
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
    std::vector<std::vector<double>> neighbor_dxdythetadtimes = this->angle_to_dxdythetadtimes[dir];

    // set the goal based on the pallet_goal or dropoff_goal
    std::vector<double> goal = is_pallet_goal ? this->pallet_goal : this->dropoff_goal;

    // For each neighbor, create a new node and add it to the list of neighbors
    for (auto &neighbor_dxdythetadtime : neighbor_dxdythetadtimes)
    {
        double dx = neighbor_dxdythetadtime[0];
        double dy = neighbor_dxdythetadtime[1];
        double theta = neighbor_dxdythetadtime[2];
        double time = neighbor_dxdythetadtime[3];


        // calculate the g and h values for the neighbor
        int g = curr_node->g_value + (int)time;
        // calculate h value using diagonal distance
        int h = std::max(abs(curr_node->x - goal[0]), abs(curr_node->y - goal[1]));

        // Create a new node
        Node *neighbor = new Node(curr_node->x + (int)dx, curr_node->y + (int)dy, theta, curr_node->time + time, g, h, curr_node);

        // Add the neighbor to the list of neighbors
        neighbors.push_back(neighbor);
    }

    return neighbors;
}

bool MotionPlanner::is_in_collision(Node *node, bool is_pallet_goal)
{
    // determine the robot collision footprint
    std::vector<std::vector<int>> robot_footprint;
    // set inflation radius to 1 for pallet goal and 2 for dropoff goal
    int inflation_radius = is_pallet_goal ? 1 : 2;
    for (int x = node->x - inflation_radius; x <= node->x + inflation_radius; x++)
    {
        for (int y = node->y - inflation_radius; y <= node->y + inflation_radius; y++)
        {
            robot_footprint.push_back({x, y});
        }
    }

    // determine the pallet collision footprint (if pallet goal) using the pallet goal
    std::vector<std::vector<int>> pallet_footprint;
    for (int x = this->pallet_goal[0] - 1; x <= this->pallet_goal[0] + 1; x++)
    {
        for (int y = this->pallet_goal[1] - 1; y <= this->pallet_goal[1] + 1; y++)
        {
            pallet_footprint.push_back({x, y});
        }
    }

    // determine the x and y of the front of the robot
    int front_x = node->x + cos(node->theta);
    int front_y = node->y + sin(node->theta);
    // convert to vector
    std::vector<int> front = {front_x, front_y};

    // Check if the robot is in static collision with the map.
    for (auto &p : robot_footprint)
    {
        // If robot is out of bounds, return true
        if (p[0] < 0 || p[0] >= this->map.size() || p[1] < 0 || p[1] >= this->map[0].size())
        {
            return true;
        }

        // If dropoff goal, ignore collisions with the pallet footprint
        if (!is_pallet_goal && std::find(pallet_footprint.begin(), pallet_footprint.end(), p) != pallet_footprint.end())
        {
            continue;
        }

        // If pallet goal and robot front x, front y, and p are on the pallet footprint, ignore collision
        if (is_pallet_goal &&
            std::find(pallet_footprint.begin(), pallet_footprint.end(), p) != pallet_footprint.end() &&
            std::find(pallet_footprint.begin(), pallet_footprint.end(), front) != pallet_footprint.end())
        {
            continue;
        }

        // If robot is in collision with the map, return true
        if (this->map[p[0]][p[1]] == 1)
        {
            // Print the point of collision
            return true;
        }
    }

    // Check if the robot is in dynamic collision with other robots.
    for (auto &robot_path : this->paths)
    {
        // For each point in the robot path
        for (auto &path_point : robot_path)
        {
            // If the robot is in a 1 second window of the path point, check for collision
            if (abs(node->time - path_point[3]) <= 1)
            {
                // Create a footprint for the other robot at the path point, inflated by 2
                std::vector<std::vector<int>> other_robot_footprint;
                for (int x = path_point[0] - 2; x <= path_point[0] + 2; x++)
                {
                    for (int y = path_point[1] - 2; y <= path_point[1] + 2; y++)
                    {
                        other_robot_footprint.push_back({x, y});
                    }
                }

                // Check if the two footprints overlap
                for (auto &p : robot_footprint)
                {
                    if (std::find(other_robot_footprint.begin(), other_robot_footprint.end(), p) != other_robot_footprint.end())
                    {
                        return true;
                    }
                }
            }
        }
    }

    // If no collision, return false
    return false;
}

int MotionPlanner::a_star(bool is_pallet_goal)
{
    // set the goal based on the pallet_goal or dropoff_goal
    std::vector<double> goal = is_pallet_goal ? this->pallet_goal : this->dropoff_goal;
    // set the start based on the pallet_goal or dropoff_goal
    std::vector<double> start = is_pallet_goal ? this->start : this->pallet_goal;
    // set the relative start time based on the pallet_goal or dropoff_goal
    double start_time = is_pallet_goal ? 0 : (this->path.back()[3] + 5);

    // Create the open list of Nodes
    std::priority_queue<Node *, std::vector<Node *>, OpenListNodeComparator> open_list = std::priority_queue<Node *, std::vector<Node *>, OpenListNodeComparator>();

    // Create the start and goal nodes
    int start_h_value = abs(start[0] - goal[0]) + abs(start[1] - goal[1]);
    Node *start_node = new Node((int)start[0], (int)start[1], start[2], start_time, 0, start_h_value, nullptr);
    int goal_h_value = 0;
    Node *goal_node = new Node((int)goal[0], (int)goal[1], goal[2], 0.0, 0, goal_h_value, nullptr);

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
            // if yes, then we have found the path
            goal_node = current_node;
            break;
        }

        // Get the neighbors of the current node
        std::vector<Node *> neighbors = get_neighbors(current_node, is_pallet_goal);

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

    // // use A* to find the path for the dropoff goal
    // if (this->a_star(false) == -1)
    // {
    //     std::cout << "No path found for dropoff goal" << std::endl;
    //     return -1;
    // }

    return 0;
}