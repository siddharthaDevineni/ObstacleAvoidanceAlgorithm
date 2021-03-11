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
   float obstaclex[3] = {2.5, 3, 7};
   float obstacley[3] = {3, 4, 6};
   float params[5] = {1.1, 100, 2, 5, 2};

   OcalculationContext *ctx = new OcalculationContext;
   Oresult *res = new Oresult;

   o_errt err;
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
   Forces force;
   err = force.forceAtt(ctx, res);
   if (err != o_errt ::err_no_error)
   {
      cout << "Error detected";
   }

   err = force.forceRep(ctx, res);
   err = force.forceComp(ctx, res);
   err = force.forceAngle(ctx, res);
   err = force.nextStep(ctx, res);

   cout << "the program works good " << endl;

   cout << res->oResultFax << ' ' << res->oResultFay << ' ' << res->oResultAng << '\n';
   cout << "Error: " << res->oError << endl;

   cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
   //float q = (ctx->xRobot, ctx->yRobot);
   //float qGoal = (ctx->xGoal, ctx->yGoal);
   //vector<float> x = {ctx->xRobot, ctx->xGoal, ctx->xObstacle[2]};
   //vector<float> y = {ctx->yRobot, ctx->yGoal, ctx->yObstacle[2]};
   //plt::hold(true);

   while ((ctx->xRobot, ctx->yRobot) != (ctx->xGoal, ctx->yGoal))
   {
      force.forceAtt(ctx, res);
      force.forceRep(ctx, res);
      force.forceComp(ctx, res);
      force.nextStep(ctx, res);
      //q = (res->oResultNextX, res->oResultNextY);
      vector<float> x = {ctx->xRobot, ctx->xGoal, obstaclex[0], obstaclex[1], obstaclex[2], res->oResultNextX};
      vector<float> y = {ctx->yRobot, ctx->yGoal, obstacley[0], obstacley[1], obstacley[2], res->oResultNextY};
      plt::scatter(x, y, 100);
   }
   plt::grid(true);
   plt::show();
   return 0;
}