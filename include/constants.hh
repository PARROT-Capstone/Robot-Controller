/**
 * @file constants.hh
 * @author Prithu Pareek (ppareek@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-11-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <cmath>

#ifndef CONSTANTS_HH
#define CONSTANTS_HH

// Constants for the map
const double H_VALUE_WEIGHT = 1;
const double G_VALUE_WEIGHT = 1;
const int ROBOT_FOOTPRINT_RADIUS = 6;
const int ROBOT_FOOTPRINT_RADIUS_WITH_PALLET = 7;
const int PALLET_FOOTPRINT_RADIUS = 3;
const int ROBOT_OFFSET = 1;

const double STRAIGHT_TIME_STEP = 0.35; // 1.1111 cm/s
const double TURN_TIME_STEP = 4;       // 2.5pi cm / 5 seconds = 1.57cm/s

const bool ALLOW_IN_PLACE_TURN = false;

const double TURN_COST = 5;
const double STRAIGHT_COST = 0.1;
const double IN_PLACE_TURN_COST = 0.5;

const double NO_TURN_THRESH = 4.0;

const double PALLET_WAIT_TIME = 3.0;
const double PALLET_RUNWAY_SLOWDOWN_FACTOR = 3.0;

const double GOAL_THETA_TOLERANCE = M_PI / 8.0;

const bool USE_HEURISTIC = true;

const bool DEBUG = false;

const int NUM_SAMPLED_STARTS = 10;
const double SEARCH_RADIUS_CM = 10;

const double MAX_SEARCH_TIME = 10;

#endif // CONSTANTS_HH
