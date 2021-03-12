#ifndef OBSTACLEAVOIDANCE_INTERNAL_H
#define OBSTACLEAVOIDANCE_INTERNAL_H

// This part of the code is not available for mere mortals
#include "obstacleAvoidance.h"

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

// Add debug trace

#endif