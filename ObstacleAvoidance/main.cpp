#include <iostream>
#include "functionsRepandAtt.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char *argv[])
{
   float goalCoordinates[2] = {10, 10};
   float robotCoordinates[2] = {0, 0};
   //float obstaclex[3] = {2.5, 3.9, 7};
   //float obstacley[3] = {2.2, 3.9, 5};
   int nObstaclesCurve = 24;
   int nObstaclesTotal = 27;
   float obstaclex[nObstaclesTotal];
   float obstacley[nObstaclesTotal];
   float obstaclexf = 0.5f;
   //float obstaclex = 0.7; obstaclex <= 3.5; obstaclex = obstaclex + 0.1
   for (int i = 0; i < nObstaclesCurve; i++)
   {

      obstacley[i] = (pow((1 / obstaclexf), 2) + 0.5);
      obstaclex[i] = obstaclexf;
      obstaclexf += 0.1;
   }
   obstaclex[24] = {6};
   obstacley[24] = {3.5};
   obstaclex[25] = {7};
   obstacley[25] = {6.5};
   obstaclex[26] = {8.5};
   obstacley[26] = {7.2};

   float params[6] = {1.1, 100, 0.1, 1, 2, float(nObstaclesTotal)};
   o_errt err;
   OcalculationContext *ctx = new OcalculationContext;
   Oresult *res = new Oresult;
   Forces force;

   err = obaInitCalculationContext(goalCoordinates, robotCoordinates, params, obstaclex, obstacley, ctx);
   if (err != o_errt ::err_no_error)
   {
      cout << "Error detected";
   }
   err = obaInitResult(res);
   if (err != o_errt ::err_no_error)
   {
      cout << "Error detected";
   }

   cout << " the program works good " << endl;

   cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
   //float q = (ctx->xRobot, ctx->yRobot);
   //float qGoal = (ctx->xGoal, ctx->yGoal);
   //vector<float> x = {ctx->xRobot, ctx->xGoal, ctx->xObstacle[2]};
   //vector<float> y = {ctx->yRobot, ctx->yGoal, ctx->yObstacle[2]};
   //plt::hold(true);
   //int i = 0;
   vector<float> xR;
   vector<float> yR;
   //vector<float> obsx = {obstaclex[0], obstaclex[1], obstaclex[2]};
   //vector<float> obsy = {obstacley[0], obstacley[1], obstacley[2]};
   vector<float> obsx;
   vector<float> obsy;

   for (int i = 0; i < params[5]; i++)
   {
      obsx.push_back(obstaclex[i]);
      obsy.push_back(obstacley[i]);
   }
   vector<float> goalx = {ctx->xGoal};
   vector<float> goaly = {ctx->yGoal};
   plt::figure();
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
      plt::scatter(xR, yR, 30);
      plt::annotate("Robot", ctx->xRobot, ctx->yRobot);
      plt::scatter(obsx, obsy, 'r');
      //plt::annotate("Obstacle", obsx, obsy);
      plt::scatter(goalx, goaly, 'g');
      plt::grid(true);
      plt::title("Robot's Path Planning in Obstacle Avoidance");
      plt::show();
   }
   plt::save("master_plot.png");
   return 0;
}