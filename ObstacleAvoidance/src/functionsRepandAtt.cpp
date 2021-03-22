#include <iostream>
#include <math.h>
#include <algorithm>
#include "functionsRepandAtt.h"

o_errt Forces::forceAtt(OcalculationContext *ctx, Oresult *out)
{
    if (ctx == nullptr)
    {
        return o_errt::err_null_input;
    }
    float Fa = 0;
    float Ra = sqrt(pow((ctx->xRobot - ctx->xGoal), 2) + pow((ctx->yRobot - ctx->yGoal), 2)); // Shortest distance between robot and target
    Fa = ctx->attCoefficientKa * Ra;                                                          // Magnitude of Attraction force
    ctx->s->oResultAngTheta = atan2(ctx->yGoal - ctx->yRobot, ctx->xGoal - ctx->xRobot);      // theta is angle between the X‐axis and the line from the point of the robot to the target

    OBA_TRACE_L2("Attraction Theta: %f", (ctx->s->oResultAngTheta * 180 / 3.14));
    out->oResultFax = Fa * cos(ctx->s->oResultAngTheta); // X-component of Attraction force
    out->oResultFay = Fa * sin(ctx->s->oResultAngTheta); // Y-component of Attraction force

    ctx->s->attForce = Fa;
    OBA_TRACE_L2("Robot Coordinates:(%f,%f) Target Distance: (%f) Attraction force: (%f)", ctx->xRobot, ctx->yRobot, ctx->s->distRA, Fa);

    return o_errt::err_no_error;
}

float forceRepLineRO(float distRO, float maxObstInfluence, uint16_t funcOrder, float distRA)
{
    return (pow(distRO, -1) - pow(maxObstInfluence, -1)) * pow(distRA, funcOrder) * pow(distRA, -3);
}
float forceRepLineRG(float distRO, float maxObstInfluence, uint16_t funcOrder, float distRA)
{
    return pow((pow(distRO, -1) - pow(maxObstInfluence, -1)), 2) * pow(distRA, funcOrder);
}

o_errt Forces::forceRep(OcalculationContext *ctx, Oresult *out)
{
    float Fr, Fr1, Fr2;
    for (int i = 0; i < ctx->n_obstacles; i++)
    {
        ctx->s->distRO[i] = sqrt(pow((ctx->xRobot - ctx->xObstacle[i]), 2) + pow((ctx->yRobot - ctx->yObstacle[i]), 2)); // Shortest distance between robot and obstacle
        if (ctx->s->distRO[i] <= ctx->maxObstInfluence)                                                                  // G represents safe distance from obstacle
        {
            Fr1 = forceRepLineRO(ctx->s->distRO[i], ctx->maxObstInfluence, ctx->funcOrder, ctx->s->distRA); // Fr1 is force component in the direction of the line between the robot and the obstacle
            Fr2 = forceRepLineRG(ctx->s->distRO[i], ctx->maxObstInfluence, ctx->funcOrder, ctx->s->distRA); // Fr2 is force component in the direction of the line between the robot and the target
            Fr = ctx->repCoefficientKrep * Fr1 + ctx->repCoefficientKrep * ctx->funcOrder * Fr2;            // Magnitude of Repulsion force
        }
        else
        {
            Fr = 0;
        }
        float dot = ctx->xObstacle[i] * ctx->xRobot + ctx->yObstacle[i] * ctx->yRobot;
        float det = ctx->xObstacle[i] * ctx->yRobot - ctx->yObstacle[i] * ctx->xRobot;
        ctx->s->oResultAngPhi[i] = atan2(det, dot);              // phi is angle between the X‐axis and the line from the point of the robot to the obstacle
        ctx->s->repForce = Fr;                                   // Magnitude of repulsion force
        out->oResultFrx[i] = Fr * cos(ctx->s->oResultAngPhi[i]); // Component of repulsion in the direction of the x-axis
        out->oResultFry[i] = Fr * sin(ctx->s->oResultAngPhi[i]); // Component of repulsion in the direction of the y-axis
        OBA_TRACE_L2("Obstacle Coordinates: (%f,%f) Obstacle Distance: (%f) FR1: (%f) FR2: (%f) Repulsion force: (%f)", ctx->xObstacle[i], ctx->yObstacle[i], ctx->s->distRO[i], Fr1, Fr2, ctx->s->repForce);
    }

    // Shortest distance between robot and obstacle

    out->oError = o_errt::err_no_error;
    return o_errt::err_no_error;
}
o_errt Forces::forceComp(OcalculationContext *ctx, Oresult *out)
{
    ctx->s->oResultFx = out->oResultFax;
    ctx->s->oResultFy = out->oResultFay;

    for (int i = 0; i < ctx->n_obstacles; i++)
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
float funStepSize(float attForce, float repForce, float stepSize)
{
    /*if (repForce = 0)
    {
        return 0.1;
    }
    else
    {
        return attForce / repForce;
    }*/
    return stepSize;
}
o_errt Forces::nextStep(OcalculationContext *ctx, Oresult *out)
{
    out->oResultNextX = ctx->xRobot + funStepSize(ctx->s->attForce, ctx->s->repForce, ctx->stepSize) * cos(out->oResultAng); // Next position of the robot X coordinate
    out->oResultNextY = ctx->yRobot + funStepSize(ctx->s->attForce, ctx->s->repForce, ctx->stepSize) * sin(out->oResultAng); // Next position of the robot Y coordinate
                                                                                                                             /*if ((ctx->xRobot, ctx->yRobot) > (ctx->xGoal, ctx->yGoal))
        {
        out->oResultNextX = ctx->xRobot - funcstepSize * cos(out->oResultAng);
        out->oResultNextY = ctx->yRobot - funcstepSize * sin(out->oResultAng);
    }*/
    if (ctx->s->distRA <= 0.1)
    {
        out->oResultNextX = ctx->xGoal;
        out->oResultNextY = ctx->yGoal;
    }
    ctx->xRobot = out->oResultNextX;
    ctx->yRobot = out->oResultNextY;
    return o_errt::err_no_error;
}
