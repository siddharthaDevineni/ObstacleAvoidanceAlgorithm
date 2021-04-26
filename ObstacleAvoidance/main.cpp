#include <iostream>
#include "functionsRepandAtt.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp; // Importing matplotlib-cpp

int main(int argc, char *argv[])
{
    float goalCoordinates[2] = {10, 10};          // Position of target
    float robotCoordinates[2] = {0, 0};           // Initial position of robot
    const int nObstaclesTotal = 2;                // Number of obstacles
    float obstaclex[nObstaclesTotal] = {1, 5};    // {obs1_x1, obs2_x1, obs3_x1}
    float obstacley[nObstaclesTotal] = {5, 7};    // {obs1_y1, obs2_y1, obs3_y1}
    float xObstacleEnd[nObstaclesTotal] = {4, 8}; // {obs1_x2, obs2_x2, obs3_x2}
    float yObstacleEnd[nObstaclesTotal] = {5, 4}; // {obs1_y2, obs2_y2, obs3_y2}
    // o_envType_t envType = env_stationary;                // Obstacle environment type stationary
    o_envType_t envType = env_dynamic; // Obstacle environment type dynamics
    // o_obstMovementType_t obsMovType = obst_mov_linear; // Obstacle path movement type linear
    o_obstMovementType_t obsMovType = obst_mov_quadratic; // Obstacle path movement type quadratic

    // Paramters as explained in obstacleAvoidance
    float params[6] = {1.1, 100, 0.1, 0.75, 2, float(nObstaclesTotal)};

    // As explained in corresponding libraries:

    // Obstacle Avoidance possible Error object creation as err
    o_err_t err;

    // Object creation for the calculation context as ctx
    OcalculationContext *ctx = new OcalculationContext;

    // Creation of object for the Result as res
    Oresult *res = new Oresult;

    // Creation of object for Forces as force
    Forces force;

    // Function for initialization of parameters to run the program as explained in the library obstacleAvoidance
    err = oba_Init_CalculationContext(goalCoordinates, robotCoordinates, params, obstaclex, obstacley, ctx);
    if (err != o_err_t ::err_no_error)
    {
        cout << "Error detected";
    }
    // Function takes the Results obtained from functionsRepandAtt to check errors
    err = oba_Init_Result(res);
    if (err != o_err_t ::err_no_error)
    {
        cout << "Error detected";
    }

    err = oba_Init_Environment(xObstacleEnd, yObstacleEnd, ctx, envType, obsMovType, res);
    if (err != o_err_t ::err_no_error)
    {
        cout << "Error detected";
    }

    vector<float> xR = {robotCoordinates[0]}; // X-coordinate of robot
    vector<float> yR = {robotCoordinates[1]}; // Y-coordinate of robot
    vector<float> goalx = {goalCoordinates[0]};
    vector<float> goaly = {goalCoordinates[1]};

    if (ctx->envType == env_dynamic)
    {
        vector<float> xObs1; // X-coordinates of start and end points of Obstacle1
        vector<float> yObs1; // Y-coordinates of start and end points of Obstacle1
        vector<float> xObs2; // X-coordinates of start and end points of Obstacle2
        vector<float> yObs2; // X-coordinates of start and end points of Obstacle2
        if (obsMovType == obst_mov_quadratic)
        {
            int max = (res->obsPts.at(0).size() > res->obsPts.at(1).size()) ? res->obsPts.at(0).size() : res->obsPts.at(1).size();
            for (int i = 0; i < max / 2; i++)
            {
                if (i < res->obsPts.at(0).size() / 2)
                {
                    xObs1.push_back(res->obsPts[0][2 * i]);
                    yObs1.push_back(res->obsPts[0][2 * i + 1]);
                }
                if (i < res->obsPts.at(1).size() / 2)
                {
                    xObs2.push_back(res->obsPts[1][2 * i]);
                    yObs2.push_back(res->obsPts[1][2 * i + 1]);
                }
            }
        }

        // xObs3 = {obstaclex[2], xObstacleEnd[2]}; // X-coordinates of start and end points of Obstacle3
        // yObs3 = {obstacley[2], yObstacleEnd[2]}; // X-coordinates of start and end points of Obstacle3

        while ((res->oResultNextX, res->oResultNextY) != (goalCoordinates[0], goalCoordinates[1]))
        {
            err = force.force_Att(ctx, res);   // Calculate attraction force between the robot and target
            err = force.force_Rep(ctx, res);   // Calculate force of repulsion between the Robot and the obstacles
            err = force.force_Comp(ctx, res);  // Calculate the total force by adding the corresponding components of attraction & repulsion forces
            err = force.force_Angle(ctx, res); // Calculate the steering angle for direction (navigation) using total force components
            err = force.next_Step(ctx, res);   // Calculate the next step for the robot consisting of x and y coordinates as its position

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
            plt::clf();
            plt::plot(xR, yR);

            plt::plot(xObs1, yObs1);
            plt::plot(xObs2, yObs2);
            // plt::plot(xObs3, yObs3);
            plt::scatter(obsptsX, obsptsY, 'r');
            plt::scatter(goalx, goaly, 'g');
            plt::grid(true);
            plt::pause(0.01);
            plt::title("Robot's Path Planning in Dynamic Environment");
            // plt::show();
        }
    }

    if (ctx->envType == env_stationary)
    {

        vector<float> xObs = {obstaclex[0], obstaclex[1], obstaclex[2]}; // x-coordinate of obstacles
        vector<float> yObs = {obstacley[0], obstacley[1], obstaclex[2]}; // y-coordinate of obstacles

        while ((res->oResultNextX, res->oResultNextY) != (goalCoordinates[0], goalCoordinates[1]))
        {

            err = force.force_Att(ctx, res);   // Calculate attraction force between the robot and target
            err = force.force_Rep(ctx, res);   // Calculate force of repulsion between the Robot and the obstacles
            err = force.force_Comp(ctx, res);  // Calculate the total force by adding the corresponding components of attraction & repulsion forces
            err = force.force_Angle(ctx, res); // Calculate the steering angle for direction (navigation) using total force components
            err = force.next_Step(ctx, res);   // Calculate the next step for the robot consisting of x and y coordinates as its position

            xR.push_back(res->oResultNextX); // for plotting purpose
            yR.push_back(res->oResultNextY); // for plotting purpose
            plt::clf();
            plt::plot(xR, yR);
            plt::annotate("Robot", ctx->xRobot, ctx->yRobot);
            plt::scatter(xObs, yObs, 'r');
            plt::scatter(goalx, goaly, 'g');
            plt::grid(true);
            plt::pause(0.01);
            plt::title("Robot's Path Planning in Stationary Environment");

            // plt::show();
        }
    }

    err = oba_Free_CalculationContext_Res(ctx, res);
    cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
    return 0;
}