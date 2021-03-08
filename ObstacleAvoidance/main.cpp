#include <iostream>
#include "functionsRepandAtt.h"
#include "obstacleAvoidance.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char *argv[])
{
   float ka = 1.1;   //coefficient of attraction
   float krep = 100; // coefficient of repulsion
   float M = 2;
   float G = 5;   // coefficient of influential force
   float L = 0.1; // step size
   int n = 3;     // number of obstacles
   float x = 0;
   float y = 0; // initial position of the object
   float q = (x, y);
   float xg = 10;
   float yg = 10; // position of target
   float qg = (xg, yg);
   float xo[3] = {2, 3.5, 7.3};
   float yo[3] = {3.4, 4.4, 2.5}; // position of obstacles
   int i = 0;                     // starting index of object
   Forces force;
   Oresult out;
   o_errt errors;
   float Repx[n];
   float Repy[n];
   float ForceX;
   float ForceY;
   while (q != qg)
   {
      force.forceAtt(ka, x, y, xg, yg, &out);
      for (int j = 0; j < n; j++)
      {
         o_errt forceRep(krep, G, M, x, y, xo[j], yo[j], &out);
         ForceX[j] = out.oResultFax + out.oResultFrx;
         ForceY[j] = out.oResultFay + out.oResultFry;
      }
      
   }

   //cout << "the program works good " << endl;

   //fig = plt.figure()  # an empty figure with no Axes

   //cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
   plt::plot({1, 3, 2, 4});
   plt::show();
   return 0;
}