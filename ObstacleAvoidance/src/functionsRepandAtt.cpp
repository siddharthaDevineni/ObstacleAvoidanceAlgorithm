#include <iostream>
#include <math.h>
#include <algorithm>
#include "functionsRepandAtt.h"

o_errt Forces::angles(OcalculationContext *ctx, Oresult *out)
{
    ctx->s->oResultAngTheta = atan2(ctx->yGoal - ctx->yRobot, ctx->xGoal - ctx->xRobot);       // theta is angle between the X‐axis and the line from the point of the robot to the target
    ctx->s->oResultAngPhi = atan2(ctx->yObstacle - ctx->yRobot, ctx->xObstacle - ctx->xRobot); // phi is angle between the X‐axis and the line from the point of the robot to the obstacle
}

o_errt Forces::forceAtt(OcalculationContext *ctx, Oresult *out)
{
    if (ctx == nullptr)
    {
        return o_errt::err_null_input;
    }
    float Fa;
    float Ra = sqrt(pow((ctx->xRobot - ctx->xGoal), 2) + pow((ctx->yRobot - ctx->yGoal), 2)); // Shortest distance between robot and target
    Fa = ctx->attCoefficientKa * Ra;                                                          // Magnitude of Attraction force

    out->oResultFax = Fa * cos(ctx->s->oResultAngTheta); // X-component of Attraction force
    out->oResultFay = Fa * sin(ctx->s->oResultAngPhi);   // Y-component of Attraction force
    ctx->s->attForce = Fa;

    return o_errt::err_no_error;
}

float forceRepLineRO(float distRO, float maxObstInfluence, uint16_t funcOrder)
{
    return (pow(distRO, -1) - pow(maxObstInfluence, -1)) * pow(distRO, funcOrder) * pow(distRO, -3);
}
float forceRepLineRG(float distRO, float maxObstInfluence, uint16_t funcOrder)
{
    return pow((pow(distRO, -1) - pow(maxObstInfluence, -1)), 2) * pow(distRO, funcOrder);
}

o_errt Forces::forceRep(OcalculationContext *ctx, Oresult *out)
{
    float Fr = 0, Fr1, Fr2;
    for (int i = 0; i < ctx->s->n_obstacles; i++)
    {
        ctx->s->distRO[i] = sqrt(pow((ctx->xRobot - ctx->xObstacle[i]), 2) + pow((ctx->yRobot - ctx->yObstacle[i]), 2)); // Shortest distance between robot and obstacle
        if (ctx->s->distRO[i] <= ctx->maxObstInfluence)                                                                  // G represents safe distance from obstacle
        {
            Fr1 = forceRepLineRO(ctx->s->distRO[i], ctx->maxObstInfluence, ctx->funcOrder);      // Fr1 is force component in the direction of the line between the robot and the obstacle
            Fr2 = forceRepLineRG(ctx->s->distRO[i], ctx->maxObstInfluence, ctx->funcOrder);      // Fr2 is force component in the direction of the line between the robot and the target
            Fr = ctx->repCoefficientKrep * Fr1 + ctx->repCoefficientKrep * ctx->funcOrder * Fr2; // Magnitude of Repulsion force
        }

        out->oResultFrx[i] = Fr * cos(ctx->s->oResultAngTheta); // Component of repulsion in the direction of the x-axis
        out->oResultFry[i] = Fr * sin(ctx->s->oResultAngPhi);   // Component of repulsion in the direction of the y-axis
    }
    // Shortest distance between robot and obstacle

    out->oError = o_errt::err_no_error;

    return o_errt::err_no_error;
}
o_errt Forces::forceComp(OcalculationContext *ctx, Oresult *out)
{
    ctx->s->oResultFx = out->oResultFax;
    ctx->s->oResultFy = out->oResultFay;

    for (int i = 0; i < ctx->s->n_obstacles; i++)
    {
        ctx->s->oResultFx += out->oResultFrx[i]; // Total Force in X-direcion
        ctx->s->oResultFy += out->oResultFry[i]; // Total Force in Y-direcion
    }

    return o_errt::err_no_error;
}
o_errt Forces::forceAngle(OcalculationContext *ctx, Oresult *out)
{

    if (ctx->s->oResultFx > 0)
    {
        out->oResultAng = atan2(ctx->s->oResultFy, ctx->s->oResultFx); // Steering angle
    }
    else
    {
        out->oResultAng = M_PI + atan2(ctx->s->oResultFy, ctx->s->oResultFx);
    }

    return o_errt::err_no_error;
}
o_errt Forces::nextStep(OcalculationContext *ctx, Oresult *out)
{

    out->oResultNextX = ctx->xRobot + ctx->stepSize * cos(out->oResultAng); // Next position of the robot X coordinate
    out->oResultNextY = ctx->yRobot + ctx->stepSize * sin(out->oResultAng); // Next position of the robot Y coordinate

    return o_errt::err_no_error;
}
