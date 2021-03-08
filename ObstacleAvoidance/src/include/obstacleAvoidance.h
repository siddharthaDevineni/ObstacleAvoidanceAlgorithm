#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include "obstacleAvoidance_internal.h"

// Obstacle Avoidance Error List
typedef enum class o_errt
{
	err_no_error,
	err_invalid_input,
	err_null_input,
	err_obstaclecount_exceeded
};

// Result struct
struct Oresult
{

	float oResultFax;
	float oResultFay;
	float oResultFrx[N_MAX_OBSTACLES];
	float oResultFry[N_MAX_OBSTACLES];
	float oResultAng;
	float oResultNextX;
	float oResultNextY;
	o_errt oError;
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
	float xObstacle[N_MAX_OBSTACLES];
	float yObstacle[N_MAX_OBSTACLES];
	struct OcalculationState *s;
};

// Functions

/*
	 * 
	 * @param  
	 */
o_errt obaInitCalculationContext(float goalCoordinates[2], float robotCoordinates[2], float params[5], float *obstx,
								 float *obsty, OcalculationContext *ctx);
/*
	 * 
	 * @param  
	 */
o_errt obaFreeCalculationContext(OcalculationContext *ctx);
/*
	 * 
	 * @param  
	 */
o_errt obaInitResult(Oresult *res);

#endif