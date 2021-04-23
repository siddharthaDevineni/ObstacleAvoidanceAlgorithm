#ifndef FUNCTIONSREPANDATT_H
#define FUNCTIONSREPANDATT_H

#include "obstacleAvoidance.h"
#include "obstacleAvoidance_internal.h"

// Class of various force functions headers
class Forces
{

public:
	/*
	 * The force of attraction between the Robot and the goal or tagret
	 * Output: X and Y components of Attraction force
	 * @params: Calculation Context
	 * @params: Output result as struct 
	 */
	o_err_t force_Att(OcalculationContext *ctx, Oresult *out);

	/*
	 * @ The force of repulsion between the Robot and the obstacles
	 * Output: X and Y components of Repulsion force
	 * @params: Calculation context
	 * @params: Output result as struct
	 */
	o_err_t force_Rep(OcalculationContext *ctx, Oresult *out);

	/*
	 * @ The function calculates the Total force by adding the corresponding components of attraction and repulsion forces
	 * Output: X and Y components of Total force
	 * @params: Calculation context
	 * @params: Output result as struct
	 */
	o_err_t force_Comp(OcalculationContext *ctx, Oresult *out);

	/*
	 * @ Calculates the steering angle for direction (navigation) using Total force components
	 * Output: steering angle
	 * @params: Calculation context
	 * @params: Output result as struct
	 */
	o_err_t force_Angle(OcalculationContext *ctx, Oresult *out);

	/*
	 * @ Calculates the next forward step for the robot consisting of x and y coordinates as position
	 * Output: X and Y coordinates of next position of robot
	 * @params: Calculation context
	 * @params: Output result as struct
	 */
	o_err_t next_Step(OcalculationContext *ctx, Oresult *out);
};

#endif