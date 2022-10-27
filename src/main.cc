/**
 * @file main.cc
 * @author Prithu Pareek (ppareek@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-09-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <iostream>
#include "planner.hh"
#include <chrono>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// Function to generate a dummy map discretized in 10mm cells, takes in the map length and width in meters. Take in pallet positions (x,y, theta) and block off a 10mm radius around the pallets in the map.
// (x,y) are in mm.
std::vector<std::vector<int>> generate_map(int map_length_mm, int map_width_mm, std::vector<std::vector<double>> pallet_poses)
{

    // Initialize map
    std::vector<std::vector<int>> map(map_length_mm / 10, std::vector<int>(map_width_mm / 10, 0));

    // Block off pallet positions
    for (auto &p : pallet_poses)
    {
        int x = p[0] / 10;
        int y = p[1] / 10;
        int theta = p[2];

        // Block off 10mm radius around pallet
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                map[x + i][y + j] = 1;
            }
        }
    }

    return map;
}

// Function to print the map. Map is x by y, where x is the length and y is the width.
void print_map(std::vector<std::vector<int>> map)
{
    for (int y = 0; y < map[0].size(); y++)
    {
        for (int x = 0; x < map.size(); x++)
        {
            std::cout << map[x][y] << " ";
        }
        std::cout << std::endl;
    }
}

// Function to visualize the planned paths for each robot. Takes in the map, the robot paths, and the pallet positions.
// Color the pallets red, the robot paths blue, and the free space green.
void visualize_paths(std::vector<std::vector<int>> map, std::vector<std::vector<std::vector<double>>> robot_paths, std::vector<std::vector<double>> pallet_poses)
{
    // Initialize map
    std::vector<std::vector<int>> map_visual(map.size(), std::vector<int>(map[0].size(), -2));

    // Block off pallet positions
    for (auto &p : pallet_poses)
    {
        int x = p[0] / 10;
        int y = p[1] / 10;
        int theta = p[2];

        // Block off 10mm radius around pallet
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                map_visual[x + i][y + j] = -1;
            }
        }
    }

    // Color robot paths
    for (auto &robot_path : robot_paths)
    {
        for (int i = 0; i < robot_path.size(); i++)
        {
            std::vector<double> pose = robot_path[i];
            int x = pose[0] / 10;
            int y = pose[1] / 10;
            int theta = pose[2];

            map_visual[x][y] = i;
        }
    }

    // Print map
    for (int y = 0; y < map_visual[0].size(); y++)
    {
        for (int x = 0; x < map_visual.size(); x++)
        {
            if (map_visual[x][y] == -2)
            {
                std::cout << "\033[1;32m"
                          << " O "
                          << "\033[0m";
            }
            else if (map_visual[x][y] == -1)
            {
                std::cout << "\033[1;31m"
                          << " + "
                          << "\033[0m";
            }
            else
            {
                if (map_visual[x][y] < 10)
                {
                    std::cout << "\033[1;34m"
                              << " " << map_visual[x][y] << " "
                              << "\033[0m";
                }
                else
                {
                    std::cout << "\033[1;34m"
                              << map_visual[x][y] << " "
                              << "\033[0m";
                }
            }
        }
        std::cout << std::endl;
    }
}

// Verify that the planned paths do not overlap at the same time step.
bool verify_paths(std::vector<std::vector<std::vector<double>>> robot_paths)
{
    // Get the maximum path length
    int max_path_length = 0;
    for (auto &robot_path : robot_paths)
    {
        if (robot_path.size() > max_path_length)
        {
            max_path_length = robot_path.size();
        }
    }

    // Check for collisions
    for (int i = 0; i < max_path_length; i++)
    {
        std::vector<std::vector<double>> poses_at_time;
        for (auto &robot_path : robot_paths)
        {
            if (i < robot_path.size())
            {
                poses_at_time.push_back(robot_path[i]);
            }
        }

        for (int j = 0; j < poses_at_time.size(); j++)
        {
            for (int k = j + 1; k < poses_at_time.size(); k++)
            {
                if (poses_at_time[j][0] == poses_at_time[k][0] && poses_at_time[j][1] == poses_at_time[k][1])
                {
                    return false;
                }
            }
        }
    }

    return true;
}

void run_planner_test()
{
    // Generate a 500mm x 500mm map with pallets at (100, 100, 0)
    std::vector<std::vector<double>> pallet_poses = {{100, 100, 0}, {200, 200, 0}};
    std::vector<std::vector<int>> map = generate_map(500, 500, pallet_poses);

    // Create a planner with 1 robot
    Planner *planner = new Planner(2);

    // Set the map
    planner->set_map(map);

    // Set the robot poses
    std::vector<std::vector<double>> robot_poses = {{50, 50, 0}, {50, 150, 0}};
    planner->set_robot_poses(robot_poses);

    // Set the goal poses at (400, 400, 0)
    std::vector<std::vector<double>> goal_poses = {{400, 400, 0}, {200, 400, 0}};

    // Create the pallet and goal poses vector
    std::vector<std::vector<double>> pallet_and_goal_poses;
    for (int i = 0; i < pallet_poses.size(); i++)
    {
        pallet_and_goal_poses.push_back({pallet_poses[i][0], pallet_poses[i][1], pallet_poses[i][2], goal_poses[i][0], goal_poses[i][1], goal_poses[i][2]});
    }

    // Set the pallet and goal poses
    planner->set_pallet_and_goal_poses(pallet_and_goal_poses);

    // Time the planning in milliseconds
    auto start = std::chrono::high_resolution_clock::now();
    // Plan
    planner->plan();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Planning took " << duration.count() << " milliseconds" << std::endl;

    // Get the planned paths
    std::vector<std::vector<std::vector<double>>> paths = planner->get_paths();

    // Verify that the paths do not overlap
    bool valid_paths = verify_paths(paths);
    if (valid_paths)
    {
        std::cout << "Valid paths!" << std::endl;
    }
    else
    {
        std::cout << "Invalid paths!" << std::endl;
    }

    // Visualize the planned paths
    visualize_paths(map, paths, pallet_poses);
}

/**
 * @brief Generates robot paths given a map and a set of pallet and goal poses.
 *
 * @param map_size A vector containing the map size in the format [x, y] (mm).
 * @param robot_poses Poses of the robots in the format [x, y, theta] (mm, mm, radians).
 * @param pallet_poses Poses of the pallets in the format [x, y, theta] (mm, mm, radians).
 * @param goal_poses Poses of the goals in the format [x, y, theta] (mm, mm, radians).
 * @return std::vector<std::vector<std::vector<double>>>
 */
std::vector<std::vector<std::vector<double>>> Planner_GeneratePaths(std::vector<int> map_size, std::vector<std::vector<double>> robot_poses, std::vector<std::vector<double>> pallet_poses, std::vector<std::vector<double>> goal_poses)
{
    // Create a planner with correct number of robots
    Planner *planner = new Planner(robot_poses.size());

    // Generate a 500mm x 500mm map with pallets at (100, 100, 0)
    std::vector<std::vector<int>> map = generate_map(map_size[0], map_size[1], pallet_poses);

    // Set the map
    planner->set_map(map);

    // Set the robot poses
    planner->set_robot_poses(robot_poses);

    // Create the pallet and goal poses vector
    std::vector<std::vector<double>> pallet_and_goal_poses;
    for (int i = 0; i < pallet_poses.size(); i++)
    {
        pallet_and_goal_poses.push_back({pallet_poses[i][0], pallet_poses[i][1], pallet_poses[i][2], goal_poses[i][0], goal_poses[i][1], goal_poses[i][2]});
    }

    // Set the pallet and goal poses
    planner->set_pallet_and_goal_poses(pallet_and_goal_poses);

    // Plan
    planner->plan();

    // Get the planned paths
    std::vector<std::vector<std::vector<double>>> paths = planner->get_paths();

    return paths;
}

// Pybind for hello world function
PYBIND11_MODULE(planner, m)
{
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("run_planner_test", &run_planner_test, "A function which prints runs the planner world");

    m.def("Planner_GeneratePaths", &Planner_GeneratePaths, "A function which takes in map size, robot poses, pallet poses, and goal poses. Returns a vector of paths for each robot.");
}