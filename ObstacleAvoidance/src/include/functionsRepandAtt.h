#ifndef FUNCTIONSREPANDATT_H
#define FUNCTIONSREPANDATT_H

#include "obstacleAvoidance.h"

class Forces
{

public:
	o_errt forceAttX(float ka, float x, float xg, Oresult *out);
	o_errt forceAttY(float ka, float y, float yg, Oresult *out);
	o_errt forceAtt(float ka, Oresult *out);
	o_errt forceRepX(float krep, float d, float x, float y, float xo, float yo, Oresult *out);
	o_errt forceRepY(float krep, float d, float x, float y, float xo, float yo, Oresult *out);
	o_errt forceRep(float Frx, float Fry, Oresult *out);
};

#endif