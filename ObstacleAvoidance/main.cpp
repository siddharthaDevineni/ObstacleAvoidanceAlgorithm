#include <iostream>
#include "functionsRepandAtt.h"
#include "obstacleAvoidance.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;

int main(int argc, char *argv[])
{
   int n = 3; // number of obstacles
   float x = 0;
   float y = 0; // initial position of the object
   float xg = 10;
   float yg = 10; // position of target
   int i = 0;     //starting index of object
   float TotalAtt = 0;
   float TotalRep = 0;
   float TotalForce = 0;
   o_errt errors;
   Oresult out;
   while ((x, y) != (xg, yg))
   {
      Forces obj;
      errors = obj.forceAtt(x, y, xg, yg, &out);

      break;
   }
   

   cout << "the program works good " << endl;
   cout << argv[0] << " VERSION " << OBSTACLEAVOIDANCE_VERSION_MAJOR << "." << OBSTACLEAVOIDANCE_VERSION_MINOR << endl;
   return 0;
}