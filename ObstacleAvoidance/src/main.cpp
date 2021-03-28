#include <iostream>
#include "functionsRepandAtt.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp; // Importing matplotlib-cpp

int main(int argc, char *argv[])
{
   // position of target
   float goalCoordinates[2] = {10, 10};

   // initial position of robot
   float robotCoordinates[2] = {0, 0};

   // number of obstacles
   int nObstaclesTotal = 2;
   float obstaclex[nObstaclesTotal] = {1.5, 3};
   float obstacley[nObstaclesTotal] = {1.3, 3};

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

   // for plotting purposes
   vector<float> xR = {robotCoordinates[0]};                              // x-coordinate of robot
   vector<float> yR = {robotCoordinates[1]};                              // y-coordinate of robot
   vector<float> obsx = {obstaclex[0], obstaclex[1], goalCoordinates[0]}; // x-coordinate of obstacles
   vector<float> obsy = {obstacley[0], obstacley[1], goalCoordinates[1]}; // y-coordinate of obstacles

   vector<float> goalx = {ctx->xGoal};
   vector<float> goaly = {ctx->yGoal};
   plt::figure();
   // while robot not yet reached the target
   while ((ctx->xRobot, ctx->yRobot) != (ctx->xGoal, ctx->yGoal))
   {
      err = force.forceAtt(ctx, res);   // Calculate attraction force between the robot and target
      err = force.forceRep(ctx, res);   // Calculate force of repulsion between the Robot and the obstacles
      err = force.forceComp(ctx, res);  // Calculate the total force by adding the corresponding components of attraction & repulsion forces
      err = force.forceAngle(ctx, res); // Calculate the steering angle for direction (navigation) using total force components
      err = force.nextStep(ctx, res);   // Calculate the next step for the robot consisting of x and y coordinates as its position

      xR.push_back(ctx->xRobot); // for plotting purpose
      yR.push_back(ctx->yRobot); // for plotting purpose

      plt::plot(xR, yR);
      plt::xlabel("X-axis");
      plt::ylabel("Y-axis");
      plt::scatter(xR, yR, 30);
      plt::scatter(obsx, obsy, 'r');
      plt::scatter(goalx, goaly, 'g');
      plt::grid(true);
      plt::pause(0.01);
      plt::title("Robot's Path Planning in Obstacle Avoidance");
      plt::show();
   }
   return 0;
}