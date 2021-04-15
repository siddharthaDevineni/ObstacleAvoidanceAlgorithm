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
    ctx->s->distRA = sqrt(pow((ctx->xRobot - ctx->xGoal), 2) + pow((ctx->yRobot - ctx->yGoal), 2)); // Shortest distance between robot and target
    Fa = ctx->attCoefficientKa * ctx->s->distRA;                                                    // Magnitude of Attraction force
    ctx->s->oResultAngTheta = atan2(ctx->xGoal - ctx->xRobot, ctx->yGoal - ctx->yRobot);            // theta is angle between the X‐axis and the line from the point of the robot to the target

    OBA_TRACE_L2("Attraction Theta: %f", (ctx->s->oResultAngTheta * 180 / 3.14));
    out->oResultFax = Fa * cos(ctx->s->oResultAngTheta); // X-component of Attraction force
    out->oResultFay = Fa * sin(ctx->s->oResultAngTheta); // Y-component of Attraction force

    ctx->s->attForce = Fa;
    OBA_TRACE_L2("Robot Coordinates:(%f,%f) Target Distance: (%f) Attraction force: (%f) Fax: %f, Fay: %f",
                 ctx->xRobot, ctx->yRobot, ctx->s->distRA, ctx->s->attForce, out->oResultFax, out->oResultFay);

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
    if (ctx->envType == env_stationary)
    {
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

            float dot = (ctx->xRobot - ctx->xObstacle[i]) * 1 + 0;
            float det = 1 * (ctx->yRobot - ctx->yObstacle[i]) - 0;
            ctx->s->oResultAngPhi[i] = atan2(dot, det); // phi is angle between the X‐axis and the line from the point of the robot to the obstacle

            ctx->s->repForce = Fr;                                   // Magnitude of repulsion force
            out->oResultFrx[i] = Fr * cos(ctx->s->oResultAngPhi[i]); // Component of repulsion in the direction of the x-axis
            out->oResultFry[i] = Fr * sin(ctx->s->oResultAngPhi[i]); // Component of repulsion in the direction of the y-axis

            OBA_TRACE_L2("Obstacle Coordinates: (%f,%f) ObsAngle: %f Obstacle Distance: (%f) FRx: (%f) FRy: (%f) Repulsion force: (%f)",
                         ctx->xObstacle[i], ctx->yObstacle[i], ctx->s->oResultAngPhi[i] * 180 / 3.14, ctx->s->distRO[i],
                         out->oResultFrx[i], out->oResultFry[i], ctx->s->repForce);
        }
    }
    else if (ctx->envType == env_dynamic)
    {

        for (int j = 0; j < ctx->n_obstacles; j++)
        {
            int i = ctx->s->movCount;
            int loopCount = 0;
            while (i >= ctx->s->obstaclePtsCount[j] * 2)
            {
                i = i - (ctx->s->obstaclePtsCount[j] * 2);
                loopCount++;
            }
            if (loopCount % 2 != 0)
            {
                i = ctx->s->obstaclePtsCount[j] * 2 - i;
            }

            ctx->s->distRO[j] = sqrt(pow((ctx->xRobot - ctx->s->obsPts[j][i]), 2) + pow((ctx->yRobot - ctx->s->obsPts[j][i + 1]), 2)); // Shortest distance between robot and obstacle

            if (ctx->s->distRO[j] <= ctx->maxObstInfluence) // G represents safe distance from obstacle
            {
                Fr1 = forceRepLineRO(ctx->s->distRO[j], ctx->maxObstInfluence, ctx->funcOrder, ctx->s->distRA); // Fr1 is force component in the direction of the line between the robot and the obstacle
                Fr2 = forceRepLineRG(ctx->s->distRO[j], ctx->maxObstInfluence, ctx->funcOrder, ctx->s->distRA); // Fr2 is force component in the direction of the line between the robot and the target
                Fr = ctx->repCoefficientKrep * Fr1 + ctx->repCoefficientKrep * ctx->funcOrder * Fr2;            // Magnitude of Repulsion force
            }
            else
            {
                Fr = 0;
            }
            float dot = (ctx->xRobot - ctx->s->obsPts[j][i]);
            float det = 1 * (ctx->yRobot - ctx->s->obsPts[j][i + 1]);
            ctx->s->oResultAngPhi[j] = atan2(dot, det); // phi is angle between the X‐axis and the line from the point of the robot to the obstacle

            ctx->s->repForce = Fr;                                   // Magnitude of repulsion force
            out->oResultFrx[j] = Fr * cos(ctx->s->oResultAngPhi[j]); // Component of repulsion in the direction of the x-axis
            out->oResultFry[j] = Fr * sin(ctx->s->oResultAngPhi[j]); // Component of repulsion in the direction of the y-axis

            out->obstCoordinatesX[j] = ctx->s->obsPts[j][i];
            out->obstCoordinatesY[j] = ctx->s->obsPts[j][i + 1];

            OBA_TRACE_L2("Obstacle Coordinates: (%f,%f) ObsAngle: %f Obstacle Distance: (%f) FRx: (%f) FRy: (%f) Repulsion force: (%f)",
                         ctx->s->obsPts[j][i], ctx->s->obsPts[j][i + 1], ctx->s->oResultAngPhi[j] * 180 / 3.14, ctx->s->distRO[j],
                         out->oResultFrx[j], out->oResultFry[j], ctx->s->repForce);
        }

        ctx->s->movCount += 2; // Iterate by 2 positions
        OBA_TRACE_L1("Move Count: %d", ctx->s->movCount)
    }

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

    OBA_TRACE_L2("resultFx: %f, resultFy: %f",
                 ctx->s->oResultFx, ctx->s->oResultFy);

    return o_errt::err_no_error;
}
o_errt Forces::forceAngle(OcalculationContext *ctx, Oresult *out)
{

    out->oResultAng = atan2(ctx->s->oResultFx, ctx->s->oResultFy); // Steering angle

    OBA_TRACE_L2("resultAngle: %f",
                 out->oResultAng * 180 / 3.14);

    return o_errt::err_no_error;
}

o_errt Forces::nextStep(OcalculationContext *ctx, Oresult *out)
{
    out->oResultNextX = ctx->xRobot + ctx->stepSize * cos(out->oResultAng); // Next position of the robot X coordinate
    out->oResultNextY = ctx->yRobot + ctx->stepSize * sin(out->oResultAng); // Next position of the robot Y coordinate

    if (ctx->s->distRA <= 0.1)
    {
        out->oResultNextX = ctx->xGoal;
        out->oResultNextY = ctx->yGoal;
    }
    else
    {
        ctx->xRobot = out->oResultNextX;
        ctx->yRobot = out->oResultNextY;
    }

    return o_errt::err_no_error;
}
