#include <iostream>
#include <math.h>
#include <algorithm>
#include "functionsRepandAtt.h"

o_errt Forces::forceAtt(float ka, float x, float y, float xg, float yg, Oresult *out)
{
    float Fa, Fax, Fay, theta;
    float Ra = sqrt(pow((x - xg), 2) + pow((y - yg), 2)); // Shortest distance between robot and target
    Fa = ka * Ra;                                         // Magnitude of Attraction force
    Fax = Fa * cos(theta);                                // X-component of Attraction force
    Fay = Fa * sin(theta);                                // Y-component of Attraction force

    out->oResultFax = Fax;
    out->oResultFay = Fay;
    out->oError = o_errt::err_no_error;

    return o_errt::err_no_error;
}

o_errt Forces::forceRep(float krep, float G, float M, float x, float y, float xo, float yo, Oresult *out)
{
    float Frx, Fry, Fr, Fr1, Fr2, phi, p;
    p = sqrt(pow((x - xo), 2) + pow((y - yo), 2)); // Shortest distance between robot and obstacle
    if (p <= G)                                    // G represents safe distance from obstacle
    {
        Fr1 = (pow(p, -1) - pow(G, -1)) * pow(p, M) * pow(p, -3); // Fr1 is force component in the direction of the line between the robot and the obstacle
        Fr2 = pow((pow(p, -1) - pow(G, -1)), 2) * pow(p, M);      // Fr2 is force component in the direction of the line between the robot and the target
        Fr = krep * Fr1 + M * krep * Fr2;                         // Magnitude of Repulsion force
    }
    else
    {
        Fr = 0;
    }
    Frx = Fr * cos(phi); // Component of repulsion in the direction of the x-axis
    Fry = Fr * sin(phi); // Component of repulsion in the direction of the y-axis

    out->oResultFrx = Frx;
    out->oResultFry = Fry;
    out->oError = o_errt::err_no_error;

    return o_errt::err_no_error;
}
o_errt Forces::forceComp(Oresult *out)
{
    float Fx, Fy;
    Oresult obj;
    Fx = obj.oResultFax + obj.oResultFrx; // Total Force in X-direcion
    Fy = obj.oResultFay + obj.oResultFry; // Total Force in Y-direcion
    out->oResultFx = Fx;
    out->oResultFy = Fy;
    out->oError = o_errt::err_no_error;

    return o_errt::err_no_error;
}
o_errt Forces::forceAngle(Oresult *out)
{
    float ang;
    Oresult obj;
    if (obj.oResultFx > 0)
        ang = atan2(obj.oResultFy, obj.oResultFx); // Steering angle
    else
        ang = M_PI + atan2(obj.oResultFy, obj.oResultFx);
    out->oResultAng = ang;
    out->oError = o_errt::err_no_error;
}
o_errt Forces::nextStep(float x, float y, float L, Oresult *out)
{
    float xf, yf;
    Oresult obj;
    xf = x + L * cos(obj.oResultAng); // Next position of the robot X coordinate
    yf = y + L * sin(obj.oResultAng); // Next position of the robot Y coordinate
    out->oResultXf = xf;
    out->oResultYf = yf;
    out->oError = o_errt::err_no_error;
}