#include <iostream>
#include "functionsRepandAtt.h"

#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp;

//plt::backend('agg');

int main(int argc, char *argv[])
{

   float goalCoordinates[2] = {10, 10};
   float robotCoordinates[2] = {0, 0};
   float obstaclex[3] = {2.5, 3, 7};
   float obstacley[3] = {3, 4, 6};
   // attCoefficientKa = params[0]
   // repCoefficientKrep = params[1]
   // stepSize = params[2]
   // maxObstInfluence = params[3]
   // funcOrder = params[4]
   // n_obstacles = params[5]
   float params[6] = {100, 100, 0.8, 4, 2, 3};
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
   vector<float> obsx = {obstaclex[0], obstaclex[1], obstaclex[2]};
   vector<float> obsy = {obstacley[0], obstacley[1], obstacley[2]};
   vector<float> goalx = {ctx->xGoal};
   vector<float> goaly = {ctx->yGoal};
   plt::figure();
   while ((ctx->xRobot, ctx->yRobot) < (ctx->xGoal, ctx->yGoal))
   {
      err = force.forceAtt(ctx, res);
      err = force.forceRep(ctx, res);
      err = force.forceComp(ctx, res);
      err = force.forceAngle(ctx, res);
      err = force.nextStep(ctx, res);
      /*if ((ctx->xRobot, ctx->yRobot) >= (ctx->xGoal, ctx->yGoal))
      {
         (ctx->xRobot, ctx->yRobot) == (ctx->xGoal, ctx->yGoal);
      }*/
      //q = (res->oResultNextX, res->oResultNextY);
      xR.push_back(ctx->xRobot);
      yR.push_back(ctx->yRobot);
      plt::plot(xR, yR);
      plt::scatter(xR, yR, 100);
      plt::annotate("Robot", ctx->xRobot, ctx->yRobot);
      plt::scatter(obsx, obsy, 'r');
      //plt::annotate("Obstacle", obsx, obsy);
      plt::scatter(goalx, goaly, 'g');
      //plt::annotate("Goal", goalx, goaly);
      plt::grid(true);
      plt::title("Robot's Path Planning in Obstacle Avoidance");
      plt::show();
      /*i++;
      if (i > 50)
      {
         break;
      }*/
   }

   return 0;
}