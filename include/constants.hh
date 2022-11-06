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

#ifndef CONSTANTS_HH
#define CONSTANTS_HH

// Constants for the map
const double H_VALUE_WEIGHT = 1;
const double G_VALUE_WEIGHT = 1;
const int ROBOT_FOOTPRINT_RADIUS = 5;
const int ROBOT_FOOTPRINT_RADIUS_WITH_PALLET = 7;
const int PALLET_FOOTPRINT_RADIUS = 2;
const int ROBOT_OFFSET = 1;

const double STRAIGHT_TIME_STEP = 0.3;
const double TURN_TIME_STEP = 3.0;

const bool ALLOW_IN_PLACE_TURN = true;

const double TURN_COST = 2.5;
const double STRAIGHT_COST = 0.1;
const double IN_PLACE_TURN_COST = 0.5;

const double NO_TURN_THRESH = 2.0;

#endif // CONSTANTS_HH
