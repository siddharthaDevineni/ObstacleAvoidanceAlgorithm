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
	float oResultF;
	o_errt oError;
};

#endif