#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

// Obstacle Avoidance Error List
typedef enum class o_errt
{
	err_no_error,
	err_invalid_input
};

// Result struct
struct Oresult
{
	
	float oResultFax;
	float oResultFay;
	float oResultFrx;
	float oResultFry;
	float oResultFx;
	float oResultFy;
	float oResultAng;
	float oResultXf;
	float oResultYf;
	o_errt oError;
};

#endif