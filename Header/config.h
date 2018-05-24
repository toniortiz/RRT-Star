#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>

const bool GET_COORDINATES = false;
const bool SHOW_RRT = true;
const bool SHOW_PATH = true;
const bool REBUILD = true;

const float END_DIST_THRESHOLD = 15.0;
const float NODE_RADIUS = 4.0;

// RRTstar
const float DT = 1.0;
const int NUM_NODES = 5000;
const int MAX_ITERATIONS = 30000;
const float RRTSTAR_FACTOR = 0.1;
const float STEP_SIZE = 50.0;
const float RADIUS_NEIGHBORHOOD = 20.0;

// car constants
const float CAR_SPEED = 4.0;
const float MAX_STEERING_ANGLE = M_PI / 5;
const float CAR_LENGTH = 20.0;

// initial state
const float START_STATE_X = 100.0;
const float START_STATE_Y = 25.0;
const float START_STATE_ORIENT = 0;

// goal state
const float GOAL_STATE_X = 380.0;
const float GOAL_STATE_Y = 353.0;
const float GOAL_STATE_ORIENT = M_PI / 2;

const float WORLD_WIDTH = 712.0;
const float WORLD_HEIGHT = 534.0;

#endif // CONFIG_H
