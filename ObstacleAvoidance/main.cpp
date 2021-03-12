#include <iostream>
#include "functionsRepandAtt.h"

#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp;

namespace plt = matplotlibcpp;

int main(int argc, char *argv[])
{

   float goalCoordinates[2] = {10, 10};
   float robotCoordinates[2] = {0, 0};
   float obstaclex[3] = {2.5, 3, 7};
   float obstacley[3] = {3, 4, 6};
   float params[6] = {1.1, 100, 2, 5, 2, 3};
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
   int i = 0;
   while ((ctx->xRobot, ctx->yRobot) != (ctx->xGoal, ctx->yGoal))
   {
      err = force.forceAtt(ctx, res);
      err = force.forceRep(ctx, res);
      err = force.forceComp(ctx, res);
      err = force.forceAngle(ctx, res);
      err = force.nextStep(ctx, res);
      //q = (res->oResultNextX, res->oResultNextY);
      vector<float> x = {ctx->xRobot, ctx->xGoal, obstaclex[0], obstaclex[1], obstaclex[2], res->oResultNextX};
      vector<float> y = {ctx->yRobot, ctx->yGoal, obstacley[0], obstacley[1], obstacley[2], res->oResultNextY};
      // plt::scatter(x, y, 100,color=['blue','red','red','red','green']);

      plt::grid(true);
      plt::show();
      i++;
      if (i > 7)
      {
         break;
      }
   }
   vector<float> x = {1, 2};
   vector<float> y = {4, 3};
   vector<float> z = {'r', 'g'};
   plt::scatter(x, 'green');
   plt::show();
   return 0;
}