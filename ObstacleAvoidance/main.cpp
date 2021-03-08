#include <iostream>
#include "functionsRepandAtt.h"
#include "obstacleAvoidance.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char *argv[])
{
   float ka = 0.5; //coefficient of attraction
   float krep = 1; // coefficient of repulsion
   float d = 1;    // coefficient of influential force
   int n = 3;      // number of obstacles
   float x = 0;
   float y = 0; // initial position of the object
   float xg = 10;
   float yg = 10; // position of target
   float xo[3] = {2, 3.5, 7.3};
   float yo[3] = {3.4, 4.4, 2.5}; // position of obstacles
   int i = 0;                     //starting index of object
   float TotalAtt = 0;
   float TotalRepX = 0;
   float TotalRepY = 0;
   float TotalRep = 0;
   float TotalForce = 0;
   o_errt errors;
   Oresult out;
   while ((x, y) != (xg, yg))
   {
      Forces obj;
      //errors = obj.forceAtt(ka,x, y, xg, yg, &out);
      //break;
      TotalAtt = obj.forceAtt(ka, &out);
      for (int j = 0; j < size(xo); j++)
      {
         TotalRepX = TotalRepX + obj.forceRepX(krep, d, x, y, xo[j], yo[j], &out); // sum of all the X-components of Frep for obstacles
         TotalRepY = TotalRepY + obj.forceRepY(krep, d, x, y, xo[j], yo[j], &out); // sum of all the Y-components of Frep for obstacles
      }
      TotalRep = sqrt(pow(TotalRepX, 2) + pow(TotalRepY, 2)); //Total Repulsive force of obstacles
      TotalForce = TotalAtt + TotalRep; // Total Force at point x,y
   }
   cout << "the program works good " << endl;

   //fig = plt.figure()  # an empty figure with no Axes

   cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
   plt::plot({1, 3, 2, 4});
   plt::show();
   return 0;
}