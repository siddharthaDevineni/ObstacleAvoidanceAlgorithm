
#include "obstacleAvoidance.h"
#include "obstacleAvoidance_internal.h"

o_errt obaInitCalculationContext(float goalCoordinates[2], float robotCoordinates[2], float params[5], float *obstx,
                                 float *obsty, OcalculationContext *ctx)
{
    if (ctx == nullptr)
    {
        return o_errt::err_null_input;
    }
    ctx->xGoal = goalCoordinates[0];
    ctx->yGoal = goalCoordinates[1];
    ctx->xRobot = robotCoordinates[0];
    ctx->yRobot = robotCoordinates[1];
    ctx->attCoefficientKa = params[0];
    ctx->repCoefficientKrep = params[1];
    ctx->stepSize = params[2];
    ctx->maxObstInfluence = params[3];
    ctx->funcOrder = params[4];

    obaNobstacles(ctx, obstx, obsty);

    for (int i = 0; i < ctx->s->n_obstacles; i++)
    {
        ctx->xObstacle[i] = obstx[i];
        ctx->yObstacle[i] = obsty[i];
    }

    return o_errt::err_no_error;
}

o_errt obaFreeCalculationContext(OcalculationContext *ctx)
{
    delete ctx;
    ctx = nullptr;

    return o_errt::err_no_error;
}

o_errt obaNobstacles(OcalculationContext *ctx, float *obstx, float *obsty)
{
    if (sizeof(obstx) != sizeof(obsty))
    {
        return o_errt::err_invalid_input;
    }
    ctx->s->n_obstacles = sizeof(obstx) / sizeof(obstx[0]); // needs to check

    if (ctx->s->n_obstacles > N_MAX_OBSTACLES)
    {
        return o_errt::err_obstaclecount_exceeded;
    }
    return o_errt::err_no_error;
}

o_errt obaInitResult(Oresult *res)
{

    if (res == nullptr)
    {
        o_errt ::err_null_input;
    }

    res->oResultAng = 0.f;
    res->oResultFax = 0.f;
    res->oResultFay = 0.f;
    res->oResultFrx[N_MAX_OBSTACLES] = 0.f;
    res->oResultFry[N_MAX_OBSTACLES] = 0.f;
    res->oResultNextX = 0.f;
    res->oResultNextY = 0.f;
    res->oError = o_errt::err_no_error;

    return o_errt::err_no_error;
}