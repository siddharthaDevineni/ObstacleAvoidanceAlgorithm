#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

// Obstacle Avoidance Error List
typedef enum o_errt
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
	float oResultFrx[3];
	float oResultFry[3];
	float oResultAng;
	float oResultNextX;
	float oResultNextY;
	o_errt oError;
};

// Context must be Initialised for internal purposes
/*
	 * 
	 * @param  
	 */
struct OcalculationContext;

// Functions

/*
	 * 
	 * @ Paramters initiliaztion for the program to run 
	 */
o_errt obaInitCalculationContext(float goalCoordinates[2], float robotCoordinates[2], float params[5], float *obstx,
								 float *obsty, OcalculationContext *ctx);
/*
	 * 
	 * @ Generates error if any of parameter provided is null or in other words no parameter provided
	 */
o_errt obaFreeCalculationContext(OcalculationContext *ctx);
/*
	 * 
	 * @ Initializes the Result values to 0
	 */
o_errt obaInitResult(Oresult *res);

class Plot
{

public:
};

#endif