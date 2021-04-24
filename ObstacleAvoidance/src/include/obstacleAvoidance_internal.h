#ifndef OBSTACLEAVOIDANCE_INTERNAL_H
#define OBSTACLEAVOIDANCE_INTERNAL_H

#define DEBUG_ENABLED // Comment out to disable Trace

#ifdef DEBUG_ENABLED
#define TRACE_LEVEL 3 // Greater than 0 to activate deep trace
#endif
#ifndef DEBUG_ENABLED
#define TRACE_LEVEL -1
#endif

#define MAX_OBSTACLE_PTS 100
// This part of the code is not available for mere mortals
#include "obstacleAvoidance.h"
#include <stdarg.h>
#include <string>
#include <iostream>
#include <math.h>

// Storage of Intermediate results
struct OcalculationState
{
    float oResultAngTheta;                           // Theta is angle between the X‐axis and the line from the point of the robot to the target
    float oResultAngPhi[N_MAX_OBSTACLES];            // phi is angle between the X‐axis and the line from the point of the robot to the obstacles
    float oResultFx;                                 // Total Force in X-direcion
    float oResultFy;                                 // Total Force in Y-direcion
    float attForce;                                  // Magnitude of Attraction force
    float repForce;                                  // Magnitude of Repulsion force
    float distRA;                                    // Shortest distance between robot and target
    float distRO[N_MAX_OBSTACLES];                   // Shortest distance between robot and obstacle
    uint obstaclePtsCount[N_MAX_OBSTACLES];          // Total number of positions in the path the obstacle should travel
    float obsPts[N_MAX_OBSTACLES][MAX_OBSTACLE_PTS]; // The position coordinates for the obstacle to travel along
    uint movCount;                                   // Position of Current obstacle calculator
};

// Paramters given by the USER that are required for the context of calculation
struct OcalculationContext
{
    float attCoefficientKa;              // Coefficient of Attraction (between the robot and the target)
    float repCoefficientKrep;            // Ceofficient of Repulsion (between the robot and the obstacle)
    float xRobot;                        // X-coordinate of Robot
    float yRobot;                        // Y-coordinate of Robot
    float xGoal;                         // X-coordinate of Goal
    float yGoal;                         // Y-coordinate of Goal
    float maxObstInfluence;              // Maximum range of influence of Obstacle on Robot
    int funcOrder;                       // Regulative factor
    float stepSize;                      // Stepsize for the robot to make next move
    int n_obstacles;                     // Total number of obstacles
    float xObstacle[N_MAX_OBSTACLES];    // Starting X-coordinates of obstacles
    float yObstacle[N_MAX_OBSTACLES];    // Starting Y-coordinates of obstacles
    float xObstacleInt[N_MAX_OBSTACLES]; // Middle X-coordinates of obstacles
    float yObstacleInt[N_MAX_OBSTACLES]; // Middle Y-coordinates of obstacles
    float xObstacleEnd[N_MAX_OBSTACLES]; // End X-coordinates of obstacles
    float yObstacleEnd[N_MAX_OBSTACLES]; // End Y-coordinates of obstacles
    o_envType_t envType;                 // Type of osbtacles environment
    o_obstMovementType_t obsMovType;     // Type of obstcales path movement

    struct OcalculationState *s; // Calculation State
};

// Debug Trace
void obaDebug_print(std::string str);
void obaDebug_printFormatted(const char *format, ...);

// Levels of Print Information as 1, 2 and 3
#define OBA_TRACE(str)   \
    if (TRACE_LEVEL > 0) \
        obaDebug_print(str);
#define OBA_TRACE_L1(format, ...) \
    if (TRACE_LEVEL >= 1)         \
        obaDebug_printFormatted(format, __VA_ARGS__);

#define OBA_TRACE_L2(format, ...) \
    if (TRACE_LEVEL >= 2)         \
        obaDebug_printFormatted(format, __VA_ARGS__);

#define OBA_TRACE_L3(format, ...) \
    if (TRACE_LEVEL >= 3)         \
        obaDebug_printFormatted(format, __VA_ARGS__);

#endif