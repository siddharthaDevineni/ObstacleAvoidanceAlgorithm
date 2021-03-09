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
	 * 	Output: X and Y components of Attraction force
	 */
	o_errt forceAtt(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ The force of repulsion between the Robot and the obstacles
	 * Output: X and Y components of Repulsion force
	 */
	o_errt forceRep(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ The function calculates the Total force by adding the corresponding components of attraction and repulsion forces
	 * Output: X and Y components of Total force
	 */
	o_errt forceComp(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ Calculates the steering angle for direction (navigation) using Total force components
	 * Output: steering angle
	 */
	o_errt forceAngle(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ Calculates the next forward step for the robot consisting of x and y coordinates as position
	 * Output: X and Y coordinates of next positon of robot
	 */
	o_errt nextStep(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @ Caclculates the angle theta and phi for navigation
	 */
	o_errt angles(OcalculationContext *ctx, Oresult *out);
};

#endif