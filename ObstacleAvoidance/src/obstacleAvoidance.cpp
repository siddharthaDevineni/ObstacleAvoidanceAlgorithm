#include <cstring>
#include "obstacleAvoidance.h"
#include "obstacleAvoidance_internal.h"

/*
* DEBUG CODES
* @params string to print
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

/*
* Class Obstacle InterPolation functions
* for Linear or Quadratic paths
*/
class ObaObstInterpolationInt
{

private:
    /*
     * Obstacle movement type path generation (linear or dynamic) for each individual obstacle
     * @params: Start coordinates of obstacles
     * @params: End coordinates of obstacles
     * @params: Type of obstcales path movement
     * @params: Stepsize of the robot
     * @params: The position coordinates for the obstacles to travel along
     * @params: Total number of positions in the path the obstacle should travel
     */

    o_err_t oba_obst_movtype_individual(float obstacleStartPt[2], float obstacleEndPt[2], o_obstMovementType_t movtype, float stepsize, float *outObstPts, uint *outObstCount)
    {
        if (movtype == obst_mov_linear)
        {
            int numPoints = (float)sqrt(pow((obstacleStartPt[1] - obstacleEndPt[1]), 2) + pow((obstacleStartPt[0] - obstacleEndPt[0]), 2)) / stepsize;

            *outObstCount = std::min(numPoints, MAX_OBSTACLE_PTS);

            if (*outObstCount == MAX_OBSTACLE_PTS)
            {
                stepsize = sqrt(pow((obstacleStartPt[1] - obstacleEndPt[1]), 2) + pow((obstacleStartPt[0] - obstacleEndPt[0]), 2)) / MAX_OBSTACLE_PTS;
            }

            for (int i = 0; i < *outObstCount + 1; i++)
            {
                // Slope
                float obstPathSlope = atan2(obstacleEndPt[1] - obstacleStartPt[1], obstacleEndPt[0] - obstacleStartPt[0]);

                outObstPts[2 * i] = obstacleStartPt[0] + cos(obstPathSlope) * (stepsize * i);

                if ((obstacleEndPt[0] - obstacleStartPt[0]) != 0)
                {
                    *(outObstPts + (2 * i + 1)) = obstacleStartPt[1] + (obstacleEndPt[1] - obstacleStartPt[1]) * (*(outObstPts + (2 * i)) - obstacleStartPt[0]) / (obstacleEndPt[0] - obstacleStartPt[0]);
                }
                else
                {
                    *(outObstPts + (2 * i + 1)) = obstacleStartPt[1] + sin(obstPathSlope) * stepsize * i;
                }

                OBA_TRACE_L2("obstPathSlope: %f ", obstPathSlope * 180 / 3.14);

                OBA_TRACE_L2("Out Obstacle Points: (%f %f)", *(outObstPts + (2 * i)), *(outObstPts + (2 * i + 1)));
            }
        }

        return o_err_t::err_no_error;
    }

public:
    /*
     * Function to calculate positions of obstacle points for all the obstacles
     * @params: Calculation context ctx
     */
    o_err_t oba_obst_movtype_internal(OcalculationContext *ctx)
    {
        o_err_t res;
        for (int i = 0; i < ctx->n_obstacles; i++)
        {

            float obstStartPt[2] = {ctx->xObstacle[i], ctx->yObstacle[i]};
            float obstEndPt[2] = {ctx->xObstacleEnd[i], ctx->yObstacleEnd[i]};

            res = oba_obst_movtype_individual(obstStartPt, obstEndPt, ctx->obsMovType, ctx->stepSize, ctx->s->obsPts[i], &(ctx->s->obstaclePtsCount[i]));
            if (res != o_err_t::err_no_error)
            {
                return res;
            }
        }

        return o_err_t::err_no_error;
    }
};

/*
* Parameters initialization for the program to run
* @Params: goal coordinates as an array
* @params: robot coordinates as an array
* @params: Array of [coefficient of attraction, coefficient of repulsion, step size, max influence of obstacle, 
* function order, number of obstacles]
* @params: Obstacles position as x - coordinates
* @params: Obstacles position as y - coordinates
* @params: Calculation context
*/
o_err_t oba_Init_CalculationContext(float goalCoordinates[2], float robotCoordinates[2], float params[6], float *obstx,
                                    float *obsty, OcalculationContext *ctx)
{

    if (ctx == nullptr)
    {
        return o_err_t::err_null_input;
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

    // State initialization
    OcalculationState *state = new OcalculationState;

    if (state == nullptr)
    {
        OBA_TRACE("State could not be allocated memory");
        return o_err_t::err_no_memory;
    }
    ctx->s = state;

    if (ctx->n_obstacles > N_MAX_OBSTACLES)
    {
        return o_err_t::err_obstaclecount_invalid;
    }

    for (int i = 0; i < ctx->n_obstacles; i++)
    {
        ctx->xObstacle[i] = obstx[i];
        ctx->yObstacle[i] = obsty[i];
        OBA_TRACE_L3("xObstacle: %f, yObstacle: %f", ctx->xObstacle[i], ctx->yObstacle[i]);
    }

    OBA_TRACE("CTX initialised");

    return o_err_t::err_no_error;
}

o_err_t oba_Init_Environment(float *obstEndx, float *obstEndy, OcalculationContext *ctx, o_envType_t envtype, o_obstMovementType_t movtype)
{
    o_err_t res;
    if (ctx == nullptr)
    {
        return o_err_t::err_null_input;
    }
    if (envtype == env_dynamic)
    {
        ctx->envType = env_dynamic;

        if (ctx->s->obsPts == nullptr)
        {
            OBA_TRACE("obsPts could not be allocated memory");
            return o_err_t::err_no_memory;
        }
        for (int i = 0; i < ctx->n_obstacles; i++)
        {
            if (obstEndx != nullptr && obstEndy != nullptr)
            {
                ctx->xObstacleEnd[i] = obstEndx[i];
                ctx->yObstacleEnd[i] = obstEndy[i];
            }
            else
            {
                o_err_t err_obstaclecount_invalid;
            }
        }

        ctx->obsMovType = movtype;
        ctx->s->movCount = 0;
    }
    else if (envtype != env_stationary)
    {
        o_err_t err_invalid_input;
    }

    // Calculate intermediate Obstacle coordinates
    ObaObstInterpolationInt obj;
    res = obj.oba_obst_movtype_internal(ctx);
    if (res != err_no_error)
    {
        return res;
    }

    OBA_TRACE("CTX Environment initialised");

    return o_err_t::err_no_error;
}

o_err_t oba_Free_CalculationContext_Res(OcalculationContext *ctx, Oresult *res)
{
    delete ctx->s;

    delete ctx;
    delete res;

    return o_err_t::err_no_error;
}

o_err_t oba_Init_Result(Oresult *res)
{

    if (res == nullptr)
    {
        o_err_t ::err_null_input;
    }

    res->oResultAng = 0.f;
    res->oResultFax = 0.f;
    res->oResultFay = 0.f;
    res->oResultFrx[N_MAX_OBSTACLES] = 0.f;
    res->oResultFry[N_MAX_OBSTACLES] = 0.f;
    res->oResultNextX = 0.f;
    res->oResultNextY = 0.f;
    res->oError = o_err_t::err_no_error;

    return o_err_t::err_no_error;
}
