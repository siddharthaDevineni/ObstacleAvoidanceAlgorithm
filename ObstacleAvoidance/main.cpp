#include <iostream>
#include "functionsRepandAtt.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp; // Importing matplotlib-cpp

int main(int argc, char *argv[])
{
    float goalCoordinates[2] = {10, 10}; // position of target
    float robotCoordinates[2] = {0, 0};  // initial position of robot

    // Number of obstacles
    const int nObstaclesTotal = 2;
    float obstaclex[nObstaclesTotal] = {3, 7};
    float obstacley[nObstaclesTotal] = {1, 3};
    float xObstacleEnd[nObstaclesTotal] = {3, 7};
    float yObstacleEnd[nObstaclesTotal] = {5, 7};
    o_envType envType = env_dynamic;
    o_obstMovementType obsMovType = obst_mov_linear;
    // Paramters as explained in obstacleAvoidance
    float params[6] = {1.1, 100, 0.5, 0.5, 2, float(nObstaclesTotal)};

    // As explained in corresponding libraries
    // Obstacle Avoidance possible Error object creation as err
    o_errt err;

    // Object creation for the calculation context as ctx
    OcalculationContext *ctx = new OcalculationContext;

    // Creation of object for the Result as res
    Oresult *res = new Oresult;

    // Creation of object for Forces as force
    Forces force;

    // Function for initialization of parameters to run the program as explained in the library obstacleAvoidance
    err = obaInitCalculationContext(goalCoordinates, robotCoordinates, params, obstaclex, obstacley, ctx);
    if (err != o_errt ::err_no_error)
    {
        cout << "Error detected";
    }

    err = obaInitEnvironment(xObstacleEnd, yObstacleEnd, ctx, envType, obsMovType);
    if (err != o_errt ::err_no_error)
    {
        cout << "Error detected";
    }

    // Function takes the Results obtained from functionsRepandAtt to check errors
    err = obaInitResult(res);
    if (err != o_errt ::err_no_error)
    {
        cout << "Error detected";
    }
    vector<float> xR = {robotCoordinates[0]}; // x-coordinate of robot
    vector<float> yR = {robotCoordinates[1]}; // y-coordinate of robot
    vector<float> goalx = {ctx->xGoal};
    vector<float> goaly = {ctx->yGoal};

    if (ctx->envType = env_dynamic)
    {
        vector<float> xObs1 = {obstaclex[0], xObstacleEnd[0]}; // x-coordinate of robot
        vector<float> yObs1 = {obstacley[0], yObstacleEnd[0]}; // y-coordinate of robot

        while ((ctx->xRobot, ctx->yRobot) != (ctx->xGoal, ctx->yGoal))
        {

            err = force.forceAtt(ctx, res);   // Calculate attraction force between the robot and target
            err = force.forceRep(ctx, res);   // Calculate force of repulsion between the Robot and the obstacles
            err = force.forceComp(ctx, res);  // Calculate the total force by adding the corresponding components of attraction & repulsion forces
            err = force.forceAngle(ctx, res); // Calculate the steering angle for direction (navigation) using total force components
            err = force.nextStep(ctx, res);   // Calculate the next step for the robot consisting of x and y coordinates as its position

            xR.push_back(res->oResultNextX); // for plotting purpose
            yR.push_back(res->oResultNextY); // for plotting purpose
            vector<float> obsptsX;
            vector<float> obsptsY;
            ;
            for (int i = 0; i < nObstaclesTotal; i++)
            {
                obsptsX.push_back(res->obstCoordinatesX[i]);
                obsptsY.push_back(res->obstCoordinatesY[i]);
            }

            plt::plot(xR, yR);
            plt::plot(xObs1, yObs1);
            plt::annotate("Robot", ctx->xRobot, ctx->yRobot);

            plt::scatter(obsptsX, obsptsY, 'r');

            plt::scatter(goalx, goaly, 'g');
            plt::grid(true);
            plt::title("Robot's Path Planning in Dynamic Environment");
            plt::show();

            break;
        }
    }

    if (ctx->envType = env_stationary)
    {

        vector<float> xObs = {obstaclex[0], obstaclex[1]}; // x-coordinate of obstacles
        vector<float> yObs = {obstacley[0], obstacley[1]}; // y-coordinate of obstacles

        while ((ctx->xRobot, ctx->yRobot) != (ctx->xGoal, ctx->yGoal))
        {

            err = force.forceAtt(ctx, res);   // Calculate attraction force between the robot and target
            err = force.forceRep(ctx, res);   // Calculate force of repulsion between the Robot and the obstacles
            err = force.forceComp(ctx, res);  // Calculate the total force by adding the corresponding components of attraction & repulsion forces
            err = force.forceAngle(ctx, res); // Calculate the steering angle for direction (navigation) using total force components
            err = force.nextStep(ctx, res);   // Calculate the next step for the robot consisting of x and y coordinates as its position

            xR.push_back(res->oResultNextX); // for plotting purpose
            yR.push_back(res->oResultNextY); // for plotting purpose
            plt::plot(xR, yR);
            plt::annotate("Robot", ctx->xRobot, ctx->yRobot);
            plt::scatter(xObs, yObs, 'r');
            plt::scatter(goalx, goaly, 'g');
            plt::grid(true);
            plt::title("Robot's Path Planning in Stationary Environment");
            plt::show();

            break;
        }
    }

    err = obaFreeCalculationContext(ctx);
    cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
    return 0;
}