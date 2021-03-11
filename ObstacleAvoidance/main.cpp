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
   float obstaclex[3] = {2, 3.5, 7.3};
   float obstacley[3] = {3.4, 4.4, 2.5};
   float params[5] = {1.1, 100, 0.1, 5, 2};

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
   err = force.angles(ctx, res);
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

   //fig = plt.figure()  # an empty figure with no Axes

   cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
   //plt::plot({1, 3, 2, 4});
   std::vector<double> x = {0.1, 0.2, 0.5};
   std::vector<double> y = {0.1, 0.2, 0.5};
   plt::plot(x, y, "ro");

   plt::show();
   return 0;
}