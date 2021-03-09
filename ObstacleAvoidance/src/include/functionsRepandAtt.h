#ifndef FUNCTIONSREPANDATT_H
#define FUNCTIONSREPANDATT_H

#include "obstacleAvoidance.h"
#include "obstacleAvoidance_internal.h"

class Forces
{

public:
	/*
	 * 
	 * @ The force of attraction between the Robot and the goal or tagret
	 */
	o_errt forceAtt(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ The force of repulsion between the Robot and the obstacles
	 */
	o_errt forceRep(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ The function calculates the Total force by adding the corresponding components of attraction and repulsion forces
	 */
	o_errt forceComp(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ Calculates the steering angle for direction (navigation) using Total force components
	 */
	o_errt forceAngle(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ Calculates the next forward step for the robot consisting of x and y coordinates as position
	 */
	o_errt nextStep(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ Caclculates the angle theta and phi for navigation
	 */
	o_errt angles(OcalculationContext *ctx, Oresult *out);
};

#endif