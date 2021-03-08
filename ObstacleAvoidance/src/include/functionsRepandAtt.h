#ifndef FUNCTIONSREPANDATT_H
#define FUNCTIONSREPANDATT_H

#include "obstacleAvoidance.h"

class FunctionRepandAtt
{

public:
	/*
	 * 
	 * @param  
	 */
	o_errt forceAtt(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @param  
	 */
	o_errt forceRep(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @param  
	 */
	o_errt forceComp(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @param  
	 */
	o_errt forceAngle(OcalculationContext *ctx, Oresult *out);

	/*
	 * 
	 * @param  
	 */
	o_errt nextStep(OcalculationContext *ctx, Oresult *out);
};

#endif