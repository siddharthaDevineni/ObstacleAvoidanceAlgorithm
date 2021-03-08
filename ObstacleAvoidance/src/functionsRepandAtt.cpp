#include <iostream>
#include <math.h>
#include <algorithm>
#include "functionsRepandAtt.h"

o_errt Forces::forceAttX(float ka, float x, float xg, Oresult *out)
{
    float Fax;
    Fax = -ka * (x - xg); // X-component of Fatt : Fa_x
}
o_errt Forces::forceAttY(float ka, float y, float yg, Oresult *out)
{
    float Fay;
    Fay = -ka * (y - yg); // Y-component of Fatt : Fa_y
}
o_errt Forces::forceAtt(float ka, Oresult *out)
{
    float Fa, Fax, Fay, Ftheta;
    Fa = sqrt(pow(Fax, 2) + pow(Fay, 2)); //Total attraction force Fa: Magnitude of Attractive force
    Ftheta = atan2(Fay, Fax);             // Direction of Fatt: arctan(y/x)

    out->oResultF = Fa;
    out->oError = o_errt::err_no_error;

    return o_errt::err_no_error;
}

o_errt Forces::forceRepX(float krep, float d, float x, float y, float xo, float yo, Oresult *out)
{

    float p, Frx, Fry, Fr, Frtheta;
    p = sqrt(pow((x - xo), 2) + pow((y - yo), 2));
    if (p <= d)
    {
        Frx = krep * (pow(p, -1) - pow(d, -1)) * (pow(p, -2)) * (x - xo) * sqrt(pow((x - xo), -2));
    }
    else
    {
        Frx = 0;
    }
}
o_errt Forces::forceRepY(float krep, float d, float x, float y, float xo, float yo, Oresult *out)
{
    float p, Frx, Fry, Fr, Frtheta;
    p = sqrt(pow((x - xo), 2) + pow((y - yo), 2));
    if (p <= d)
    {
        Fry = krep * (pow(p, -1) - pow(d, -1)) * (pow(p, -2)) * (y - yo) * sqrt(pow((y - yo), -2));
    }
    else
    {
        Fry = 0;
    }
}
o_errt Forces::forceRep(float Frx, float Fry, Oresult *out)
{
    float Frx, Fry, Fr, Frtheta;
    Fr = sqrt(pow(Frx, 2) + pow(Fry, 2)); // Total repulsiom force Fr : Magnitude of Repulsive Force
    Frtheta = atan2(Fry, Frx);

    out->oResultF = Fr;
    out->oError = o_errt::err_no_error;

    return o_errt::err_no_error;
}
