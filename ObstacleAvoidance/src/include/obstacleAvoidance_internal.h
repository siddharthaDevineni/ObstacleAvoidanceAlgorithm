#ifndef OBSTACLEAVOIDANCE_INTERNAL_H
#define OBSTACLEAVOIDANCE_INTERNAL_H

#define DEBUG_ENABLED // Comment out to disable

#ifdef DEBUG_ENABLED
#define TRACE_LEVEL 3 // Greater than 0 to activate deep trace
#endif
#ifndef DEBUG_ENABLED
#define TRACE_LEVEL -1
#endif

// This part of the code is not available for mere mortals
#include "obstacleAvoidance.h"
#include <stdarg.h>
#include <string>
#include <iostream>

#define N_MAX_OBSTACLES 10

struct OcalculationState
{
    float oResultAngTheta;
    float oResultAngPhi[N_MAX_OBSTACLES];
    float oResultFx;
    float oResultFy;
    float attForce;
    float distRO[N_MAX_OBSTACLES];
};

struct OcalculationContext
{
    float attCoefficientKa;
    float repCoefficientKrep;
    float xRobot;
    float yRobot;
    float xGoal;
    float yGoal;
    float maxObstInfluence;
    int funcOrder;
    float stepSize;
    int n_obstacles;
    float xObstacle[N_MAX_OBSTACLES];
    float yObstacle[N_MAX_OBSTACLES];
    struct OcalculationState *s;
};

// Debug Trace
void obaDebug_print(std::string str);
void obaDebug_printFormatted(const char *format, ...);

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