#include <iostream>
#include "functionsRepandAtt.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp; // Importing matplotlib-cpp

int main(int argc, char *argv[])
{

   float goalCoordinates[2] = {10, 10};
   float robotCoordinates[2] = {0, 0};

   // number of obstacles

   const int nObstaclesTotal = 2;
   float obstaclex[nObstaclesTotal] = {1.5, 3};
   float obstacley[nObstaclesTotal] = {1.3, 3};

   /*    ctx->xGoal = goalCoordinates[0];
    ctx->yGoal = goalCoordinates[1];
    ctx->xRobot = robotCoordinates[0];
    ctx->yRobot = robotCoordinates[1];

    OBA_TRACE_L2("Goal Coordinates: (%f,%f) RobotCoordinates:(%f,%f)", ctx->xGoal, ctx->yGoal, ctx->xRobot, ctx->yRobot);

    ctx->attCoefficientKa = params[0];
    ctx->repCoefficientKrep = params[1];
    ctx->stepSize = params[2];
    ctx->maxObstInfluence = params[3];
    ctx->funcOrder = params[4];
    ctx->n_obstacles = params[5];*/

   // paramters as explained in obstacleAvoidance
   float params[6] = {1.1, 100, 0.2, 0.5, 2, float(nObstaclesTotal)};

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

   // Function takes the Results obtained from functionsRepandAtt to check errors
   err = obaInitResult(res);
   if (err != o_errt ::err_no_error)
   {
      cout << "Error detected";
   }

   cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;

   int i = 0;
   vector<float> xR = {robotCoordinates[0]};
   vector<float> yR = {robotCoordinates[1]};
   vector<float> xObs = {obstaclex[0], obstaclex[1], goalCoordinates[0]};
   vector<float> yObs = {obstacley[0], obstacley[1], goalCoordinates[1]};

   while ((ctx->xRobot, ctx->yRobot) != (ctx->xGoal, ctx->yGoal))
   {
      err = force.forceAtt(ctx, res);
      err = force.forceRep(ctx, res);
      err = force.forceComp(ctx, res);
      err = force.forceAngle(ctx, res);
      err = force.nextStep(ctx, res);

      xR.push_back(ctx->xRobot);
      yR.push_back(ctx->yRobot);

      plt::plot(xR, yR);
      plt::scatter(xObs, yObs, 'r');
      plt::grid(true);
      plt::title("Robot's Path Planning in Obstacle Avoidance");
      plt::show();

      if (i > 15)
      {
         break;
      }
      i++;
   }

   cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
   return 0;
}