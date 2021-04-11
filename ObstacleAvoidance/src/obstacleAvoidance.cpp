#include <cstring>
#include "obstacleAvoidance.h"
#include "obstacleAvoidance_internal.h"

/**
 * DEBUG CODES
*
*/
void obaDebug_print(std::string str)
{
    std::cout << str << "\n";
}
void obaDebug_printFormatted(const char *format, ...)
{
    va_list vl;
    va_start(vl, format);
    vprintf(format, vl);
    printf("\n");
    va_end(vl);
}

/**
 * Obstacle InterPolation functions
*
*/
class ObaObstInterpolationInt
{

private:
    /**
     * @params nobstacles: number of obstacles
     * @params obstacleStart[N_MAX_OBSTACLES * 2]: X,Y start coordinates of the obstacles
     * 
     */

    o_errt oba_obst_movtype_individual(float obstacleStartPt[2], float obstacleEndPt[2], o_obstMovementType movtype, float *outObstPts, uint outobstCount)
    {
        if (movtype == obst_mov_linear)
        {
        }
        return o_errt::err_no_error;
    }

public:
    /**
     * @params nobstacles: number of obstacles
     * @params obstacleStart[N_MAX_OBSTACLES * 2]: X,Y start coordinates of the obstacles
     * 
     */
    o_errt oba_obst_movtype_internal(OcalculationContext *ctx)
    {

        return o_errt::err_no_error;
    }
};

/**
 * LIBRARY CODES
*
*/
o_errt obaInitCalculationContext(float goalCoordinates[2], float robotCoordinates[2], float params[6], float *obstx,
                                 float *obsty, OcalculationContext *ctx)
{
    if (ctx == nullptr)
    {
        return o_errt::err_null_input;
    }
    ctx->envType = env_stationary;
    ctx->xGoal = goalCoordinates[0];
    ctx->yGoal = goalCoordinates[1];
    ctx->xRobot = robotCoordinates[0];
    ctx->yRobot = robotCoordinates[1];

    OBA_TRACE_L2("Goal Coordinates: (%f,%f) RobotCoordinates:(%f,%f)", ctx->xGoal, ctx->yGoal, ctx->xRobot, ctx->yRobot);

    ctx->attCoefficientKa = params[0];
    ctx->repCoefficientKrep = params[1];
    ctx->stepSize = params[2];
    ctx->maxObstInfluence = params[3];
    ctx->funcOrder = params[4];
    ctx->n_obstacles = params[5];

    OBA_TRACE_L2("attCoefficientKa: %f, repCoefficientKrep: %f, stepSize: %f, maxObstInfluence: %f", ctx->attCoefficientKa,
                 ctx->repCoefficientKrep, ctx->stepSize, ctx->maxObstInfluence);
    OBA_TRACE_L2("funcOrder: %f, n_obstacles: %f", ctx->funcOrder, ctx->n_obstacles);

    OcalculationState *state = new OcalculationState;
    if (state == nullptr)
    {
        OBA_TRACE("State could not be allocated memory");
        return o_errt::err_no_memory;
    }
    ctx->s = state;
    ctx->s->movCount = 0;

    if (ctx->n_obstacles > N_MAX_OBSTACLES)
    {
        return o_errt::err_obstaclecount_invalid;
    }
    for (int i = 0; i < ctx->n_obstacles; i++)
    {
        ctx->xObstacle[i] = obstx[i];
        ctx->yObstacle[i] = obsty[i];
        OBA_TRACE_L3("xObstacle: %f, yObstacle: %f", ctx->xObstacle[i], ctx->yObstacle[i]);
    }

    OBA_TRACE("CTX initialised");

    return o_errt::err_no_error;
}

o_errt obaInitEnvironment(float *obstEndx, float *obstEndy, OcalculationContext *ctx, o_envType envtype, o_obstMovementType movtype)
{
    o_errt res;
    if (ctx == nullptr)
    {
        return o_errt::err_null_input;
    }
    if (envtype == env_dynamic)
    {
        ctx->envType = env_dynamic;

        for (int i = 0; i < ctx->n_obstacles; i++)
        {
            if (obstEndx != nullptr && obstEndy != nullptr)
            {
                ctx->xObstacleEnd[i] = obstEndx[i];
                ctx->yObstacleEnd[i] = obstEndy[i];
            }
            else
            {
                o_errt err_obstaclecount_invalid;
            }
        }

        ctx->obsMovType = movtype;
    }
    else if (envtype != env_stationary)
    {
        o_errt err_invalid_input;
    }

    // Calculate intermediate Obstacle coordinates
    ObaObstInterpolationInt obj;
    res = obj.oba_obst_movtype_internal(ctx);
    if (res != err_no_error)
    {
        return res;
    }

    OBA_TRACE("CTX Environment initialised");

    return o_errt::err_no_error;
}

o_errt obaFreeCalculationContext(OcalculationContext *ctx)
{
    delete ctx;
    ctx = nullptr;

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
