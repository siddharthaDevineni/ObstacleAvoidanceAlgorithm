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
   //float obstaclex[3] = {2.5, 3.9, 7};
   //float obstacley[3] = {2.2, 3.9, 5};
   float obstaclex[24];
   float obstacley[24];
   for (float obstaclex = 0.7; obstaclex <= 3.5; obstaclex = obstaclex + 0.1)
   {
      obstacley[] = (pow((1 / obstaclex), 2) + 0.5);
   }
   //cout << sizeof(obstaclex);
   // attCoefficientKa = params[0]
   // repCoefficientKrep = params[1]
   // stepSize = params[2]
   // maxObstInfluence = params[3]
   // funcOrder = params[4]
   // n_obstacles = params[5]
   float params[6] = {1.1, 100, 0.1, 0.11, 2, 3};
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
   vector<float> obsx = {obstaclex[0], obstaclex[1], obstaclex[2], obstaclex[3], obstaclex[4], obstaclex[5], obstaclex[6], obstaclex[7], obstaclex[8], obstaclex[9], obstaclex[10], obstaclex[11], obstaclex[12], obstaclex[13], obstaclex[14], obstaclex[15], obstaclex[16], obstaclex[17], obstaclex[18], obstaclex[19], obstaclex[20], obstaclex[21], obstaclex[22], obstaclex[23]};
   vector<float> obsy = {obstacley[0], obstacley[1], obstacley[2], obstacley[3], obstacley[4], obstacley[5], obstacley[6], obstacley[7], obstacley[8], obstacley[9], obstacley[10], obstacley[11], obstacley[12], obstacley[13], obstacley[14], obstacley[15], obstacley[16], obstacley[17], obstacley[18], obstacley[19], obstacley[20], obstacley[21], obstacley[22], obstacley[23]};
   //vector<float> obsy = {obstacley[0], obstacley[1], obstacley[2]};
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
      xR.push_back(ctx->xRobot);
      yR.push_back(ctx->yRobot);
      plt::plot(xR, yR);
      plt::scatter(xR, yR);
      plt::annotate("Robot", ctx->xRobot, ctx->yRobot);
      plt::scatter(obsx, obsy, 'r');
      //plt::annotate("Obstacle", obsx, obsy);
      plt::scatter(goalx, goaly, 'g');
      //plt::annotate("Goal", goalx, goaly);
      plt::grid(true);
      plt::title("Robot's Path Planning in Obstacle Avoidance");
      plt::show();
   }

   return 0;
}