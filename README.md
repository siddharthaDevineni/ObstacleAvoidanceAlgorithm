# Obstacle Avoidance Algorithm in C++
- [Obstacle Avoidance Algorithm in C++](#obstacle-avoidance-algorithm-in-c)
  - [Description](#description)
  - [Usage](#usage)
  - [Technologies](#technologies)
  - [Setup](#setup)
  - [Roadmap](#roadmap)
  - [References](#references)
  - [License](#license)

## Description
This library is about path planning for a mobile robot to reach its destination by avoiding obstacles resulting in a safe navigation as an optimal path.

## Usage
A comprehensive example:
```c++
#include <iostream>
#include "functionsRepandAtt.h"
#include "matplotlibcpp.h"
#include "ObstacleAvoidanceConfig.h"

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char *argv[])
{
    // position of target
    float goalCoordinates[2] = {10, 10};

    // initial position of robot
    float robotCoordinates[2] = {0, 0};

    // number of obstacles
    int   nObstacles = 24;
    float obstaclex[nObstacles];
    float obstacley[nObstacles];
    float obstaclexf = 0.7f;

    // Creation of custom obstacles
     for (int i = 0; i < nObstacles; i++)
    {
      obstacley[i] = (pow((1 / obstaclexf), 2) + 0.5);
      obstaclex[i] = obstaclexf;
      obstaclexf += 0.1;
    }

    // Parameters such as Coefficient of attraction, Coefficient of repulsion, Step size of robot, Max obstacle influence, Order of function, Number of obstacles
    float params[6] = {1.1, 100, 0.2, 1, 2, float(nObstacles)};

    // Obstacle Avoidance possible Error object creation as err
    o_errt err;

    // Object creation for the calculation context as ctx
    OcalculationContext *ctx = new OcalculationContext;

    // Creation of object for the Result  as res
    Oresult *res = new Oresult;

    // Creation of object for Forces as force
    Forces force;

    /*
	 * Function for initialization of parameters to run the program
	 * Output: error if any missing or incorrect entry of parameters
	 * @params: Position of goal as its coordinates
	 * @params: Position of robot as its coordinates
     * @params: Required parameters for functions
     * @params: X coordinates of obstacles
     * @params: Y coordinates of obstacles
     * @params: Calculation context object ctx
	 */
    err = obaInitCalculationContext(goalCoordinates, robotCoordinates, params, obstaclex, obstacley, ctx);
    if (err != o_errt ::err_no_error)
    {
      cout << "Error detected";
    }

    /*
	 * @ Function takes the Result obtained from the functionsRepandAtt.h
	 * Output: generates error if any abnormal output values
     * @params: results
	 */
    err = obaInitResult(res);
    if (err != o_errt ::err_no_error)
    {
      cout << "Error detected";
    }

    // for plotting purposes using matplotlib
    vector<float> xR;
    vector<float> yR;
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
```
**Result:**
![Image of Robot](..ObstacleAvoidanceAlgorithm/Figure_1.png)

## Technologies
Project is created with:
* C++ version: 14
* CMake version: 3.17

## Setup
To run this project
* Install CMake with a minimum version of 3.10
* Add libraries [functionsRepandAtt](../src/include/functionsRepandAtt.h) and [obstacleAvoidance](../src/include/obstacleAvoidance.h)
* Install [Matplotlib-cpp](https://github.com/lava/matplotlib-cpp) which is a very easy to use C++ plotting library for plotting and visualization purposes.


## Roadmap
- [x] Robot avoids random stationary obstacles (static environment)
- [ ] Robot avoids random moving obstacles (dynamic environment)
- [ ] Implementation of interpolation functions and control strategies to calculate optimal path like shortest distance and time


## References
* [Obstacle avoidance of mobile robots using modified artificial potential field algorithm](https://doi.org/10.1186/s13638-019-1396-2)

* [Path planning for autonomous mobile robot using the Potential Field method](https://doi.org/10.1109/ASET.2017.7983725)

## License
[MIT](https://choosealicense.com/licenses/mit/)


Thanks :v:

