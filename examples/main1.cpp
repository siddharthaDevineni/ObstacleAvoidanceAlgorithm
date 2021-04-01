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
   int nObstaclesCurve = 24;
   int nObstaclesTotal = 27;
   float obstaclex[nObstaclesTotal];
   float obstacley[nObstaclesTotal];
   float obstaclexf = 0.5f;

   for (int i = 0; i < nObstaclesCurve; i++)
   {
      obstacley[i] = (pow((1 / obstaclexf), 2) + 0.5);
      obstaclex[i] = obstaclexf;
      obstaclexf += 0.1;
   }

   // random dot obstacles
   obstaclex[24] = {6};
   obstacley[24] = {3.5};
   obstaclex[25] = {7};
   obstacley[25] = {6.5};
   obstaclex[26] = {8.5};
   obstacley[26] = {7.2};
   // paramters as explained in obstacleAvoidance
   float params[6] = {1.1, 100, 0.1, 1, 2, float(nObstaclesTotal)};

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
   vector<float> xR;   // x-coordinate of robot
   vector<float> yR;   // y-coordinate of robot
   vector<float> obsx; // x-coordinate of obstacles
   vector<float> obsy; // y-coordinate of obstacles

   for (int i = 0; i < params[5]; i++)
   {
      obsx.push_back(obstaclex[i]);
      obsy.push_back(obstacley[i]);
   }
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
      xR.push_back(ctx->xRobot);        // for plotting purpose
      yR.push_back(ctx->yRobot);        // for plotting purpose
      plt::plot(xR, yR);
      plt::xlabel("X-axis");
      plt::ylabel("Y-axis");
      plt::scatter(xR, yR, 30);
      //plt::annotate("Robot", ctx->xRobot, ctx->yRobot);
      plt::scatter(obsx, obsy, 'r');
      plt::scatter(goalx, goaly, 'g');
      plt::grid(true);
      plt::pause(0.01);
      plt::title("Robot's Path Planning in Obstacle Avoidance");
   }
   //plt::show();
   plt::save("master_plot.png");
   return 0;
}