#ifndef OBSTACLEAVOIDANCE_INTERNAL_H
#define OBSTACLEAVOIDANCE_INTERNAL_H

// This part of the code is not available for mere mortals
#include "obstacleAvoidance.h"

#define N_MAX_OBSTACLES 10

struct OcalculationState
{
    float oResultFx;
    float oResultFy;
    float attForce;
    float theta;
    float distRO[N_MAX_OBSTACLES];
    int n_obstacles = 0;
};

// Add debug trace

#endif