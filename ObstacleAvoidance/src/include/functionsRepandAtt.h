#ifndef FUNCTIONSREPANDATT_H
#define FUNCTIONSREPANDATT_H

#include "obstacleAvoidance.h"

class Forces
{

public:
	
	o_errt forceAtt(float ka, float x, float y, float xg, float yg, Oresult *out);
	o_errt forceRep(float krep, float G, float M, float x, float y,float xo, float yo, Oresult *out);
	o_errt forceComp(Oresult *out);
	o_errt forceAngle(Oresult *out);
	o_errt nextStep(float x, float y, float L, Oresult *out);
};

#endif