#include <iostream>
#include "functionsRepandAtt.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp; // Importing matplotlib-cpp

int main(int argc, char *argv[])
{
    float goalCoordinates[2] = {10, 10};                 // Position of target
    float robotCoordinates[2] = {0, 0};                  // Initial position of robot
    const int nObstaclesTotal = 3;                       // Number of obstacles
    float obstaclex[nObstaclesTotal] = {1, 5, 7.7};      // {obs1_x1, obs2_x1}
    float obstacley[nObstaclesTotal] = {3, 2, 6.3};      // {obs1_y1, obs2_y1}
    float xObstacleEnd[nObstaclesTotal] = {3, 5, 8.5};   // {obs1_x2, obs2_x2}
    float yObstacleEnd[nObstaclesTotal] = {1, 6.5, 9.7}; // {obs1_y2, obs2_y2}
    o_envType envType = env_dynamic;                     // Obstacle environment type dynamics
    o_obstMovementType obsMovType = obst_mov_linear;     // Obstacle path movement type linear

    // Paramters as explained in obstacleAvoidance
    float params[6] = {1.1, 100, 0.3, 1, 2, float(nObstaclesTotal)};

    // As explained in corresponding libraries:

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
    vector<float> xR = {robotCoordinates[0]}; // X-coordinate of robot
    vector<float> yR = {robotCoordinates[1]}; // Y-coordinate of robot
    vector<float> goalx = {ctx->xGoal};
    vector<float> goaly = {ctx->yGoal};

    if (ctx->envType = env_dynamic)
    {
        vector<float> xObs1 = {obstaclex[0], xObstacleEnd[0]}; // X-coordinates of start and end points of Obstacle1
        vector<float> yObs1 = {obstacley[0], yObstacleEnd[0]}; // Y-coordinates of start and end points of Obstacle1
        vector<float> xObs2 = {obstaclex[1], xObstacleEnd[1]}; // X-coordinates of start and end points of Obstacle2
        vector<float> yObs2 = {obstacley[1], yObstacleEnd[1]}; // X-coordinates of start and end points of Obstacle2
        vector<float> xObs3 = {obstaclex[2], xObstacleEnd[2]}; // X-coordinates of start and end points of Obstacle3
        vector<float> yObs3 = {obstacley[2], yObstacleEnd[2]}; // X-coordinates of start and end points of Obstacle3
        while ((ctx->xRobot, ctx->yRobot) != (ctx->xGoal, ctx->yGoal))
        {

            err = force.forceAtt(ctx, res);   // Calculate attraction force between the robot and target
            err = force.forceRep(ctx, res);   // Calculate force of repulsion between the Robot and the obstacles
            err = force.forceComp(ctx, res);  // Calculate the total force by adding the corresponding components of attraction & repulsion forces
            err = force.forceAngle(ctx, res); // Calculate the steering angle for direction (navigation) using total force components
            err = force.nextStep(ctx, res);   // Calculate the next step for the robot consisting of x and y coordinates as its position

            xR.push_back(res->oResultNextX); // For plotting purpose
            yR.push_back(res->oResultNextY); // For plotting purpose
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
            plt::plot(xObs2, yObs2);
            plt::plot(xObs3, yObs3);
            // plt::annotate("Robot", ctx->xRobot, ctx->yRobot);

            plt::scatter(obsptsX, obsptsY, 'r');

            plt::scatter(goalx, goaly, 'g');
            plt::grid(true);
            // plt::pause(0.01);
            plt::title("Robot's Path Planning in Dynamic Environment");
            plt::show();
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
        }
    }

    err = obaFreeCalculationContext(ctx);
    cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
    return 0;
}