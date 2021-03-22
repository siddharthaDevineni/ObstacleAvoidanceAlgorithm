#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

// Obstacle Avoidance Error List
typedef enum o_errt
{
	err_no_error,
	err_invalid_input,
	err_null_input,
	err_obstaclecount_exceeded,
	err_no_memory
};

// Result struct
struct Oresult
{

	float oResultFax;	  // Magnitude of Attraction force in X-direction
	float oResultFay;	  // Magnitude of Attraction force in Y-direction
	float oResultFrx[27]; // Magnitude of Repulsion force in X-direction for each obstacle
	float oResultFry[27]; // Magnitude of Repulsion force in Y-direction for each obstacle
	float oResultAng;	  // Steering angle for navigation of the robot
	float oResultNextX;	  // Next position of the robot X coordinate
	float oResultNextY;	  // Next position of the robot Y coordinate
	o_errt oError;		  // Obstacle Avoidance Error List
};

// Context must be Initialised for internal purposes
/*
	 * @params: coefficient of attraction, coefficient of repulsion, coordinates of robot, goal, obstacles, 
	 * number of obstacles, step size, function order, 
	 * calculation state (intermediate results):
	 * angle theta which is the angle between the X‐axis and the line from the point of the robot to the target
	 * phi is angle between the X‐axis and the line from the point of the robot to the obstacle
	 * Total Force in X-direcion
	 * Total Force in Y-direcion
	 * Magnitude of Attraction force
	 * Magnitude of repulsion force
	 * Step size for the robot
	 * Shortest distance between robot and target
	 * Shortest distance between robot and obstacle
	 */
struct OcalculationContext;

/*
	 * Parameters initialization for the program to run
	 * @Params: goal coordinates as an array
	 * @params: robot coordinates as an array
	 * @params: Array of [coefficient of attraction, coefficient of repulsion, step size, max influence of obstacle, 
	 * function order, number of obstacles]
	 * @params: Obstacles position as x - coordinates
	 * @params: Obstacles position as y - coordinates
	 * @params: Calculation context
	 */
o_errt obaInitCalculationContext(float goalCoordinates[2], float robotCoordinates[2], float params[5], float *obstx,
								 float *obsty, OcalculationContext *ctx);

/*
	 * Generates error if any of parameter provided is null or in other words no parameter provided 
	 * as mentioned in the error list 
	 * @param: Calculation context as mentioned in the OcalculationContext
	 */
o_errt obaFreeCalculationContext(OcalculationContext *ctx);

/*
	 * Initializes the Result values to 0
	 * @params: Output Results as mentioned in the Oresult
	 */
o_errt obaInitResult(Oresult *res);

#endif