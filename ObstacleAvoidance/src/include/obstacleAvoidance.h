#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#define N_MAX_OBSTACLES 30 // Maximum number of obstacles

// Obstacle Avoidance Error List
typedef enum o_errt
{
	err_no_error,			   // 0: No error
	err_invalid_input,		   // 1: Invalid input
	err_null_input,			   // 2: Null input
	err_obstaclecount_invalid, // 3: Number of obstacles invalid
	err_no_memory,			   // 4: No memory
	err_calculation_error	   // 5: Calculation error
};

// Obstacle Environment Type: Stationary or Dynamic
typedef enum o_envType
{
	env_stationary, // Stationary environment
	env_dynamic		// Dynamic environment
};

// Obstacle Path Movement Type: Linear or Quadratic
typedef enum o_obstMovementType
{
	obst_mov_linear,   // Linear path
	obst_mov_quadratic // Quadratic path
};

// Output Results struct
struct Oresult
{

	float oResultFax;						 // Magnitude of Attraction force in X-direction
	float oResultFay;						 // Magnitude of Attraction force in Y-direction
	float oResultFrx[N_MAX_OBSTACLES];		 // Magnitude of Repulsion force in X-direction for each obstacle
	float oResultFry[N_MAX_OBSTACLES];		 // Magnitude of Repulsion force in Y-direction for each obstacle
	float oResultAng;						 // Steering angle for navigation of the robot
	float oResultNextX;						 // Next position of the robot X coordinate
	float oResultNextY;						 // Next position of the robot Y coordinate
	float obstCoordinatesX[N_MAX_OBSTACLES]; // X-coordinates of Obstacles
	float obstCoordinatesY[N_MAX_OBSTACLES]; // Y-coordiantes of Obstacles

	o_errt oError; // Obstacle Avoidance Error List
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
* Initializes the Obstacle environment
* @params: End X-coordinates of obstacles
* @params: End Y-coordinates of obstacles
* @params: Calculation context
* @params: Type of osbtacles environment
* @params: Type of obstcales path movement
*/
o_errt obaInitEnvironment(float *obstEndx, float *obstEndy, OcalculationContext *ctx, o_envType envtype, o_obstMovementType movtype);

/*
* Free the used memory for the variables and paramters 
* @param: Calculation context as mentioned in the OcalculationContext
*/
o_errt obaFreeCalculationContext(OcalculationContext *ctx);

/*
* Initializes the Result values to 0
* @params: Output Results as mentioned in the Oresult
*/
o_errt obaInitResult(Oresult *res);

#endif