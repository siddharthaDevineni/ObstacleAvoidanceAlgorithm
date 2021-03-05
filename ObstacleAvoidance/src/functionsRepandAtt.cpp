#include<iostream>
#include<math.h>
#include<algorithm>
#include "functionsRepandAtt.h"




o_errt Forces::forceAtt(float x, float y, float xg, float yg, Oresult* out)
{
    int ka;
    float Fax,Fay,Fa;
    Fax = -ka*(x - xg); // X-component of Fatt : Fa_x
    Fay = -ka*(y - yg); // Y-component of Fatt : Fa_y

    Fa = sqrt(pow(Fax,2)+pow(Fay,2)); //Total attraction force Fa

    out->ofloatresult = Fa;
    return o_errt::err_no_error;
}

o_errt Forces::forceRep(float x, float y, float xo, float yo, Oresult* out)
{
    int kr,ro;
    float p, Frx, Fry, Fr;
    p = sqrt(pow((x-xo),2)+pow((y-yo),2));
    if(p<=ro)
            {
                Frx = kr*(pow(p,-1)-pow(ro,-1))*(pow(p,-2))*(x-xo)*pow(abs(x-xo),-1);
            }
    else
        {
            Frx = 0;
        }
    if(p<=ro)
            {
                Fry = kr*(pow(p,-1)-pow(ro,-1))*(pow(p,-2))*(y-yo)*pow(abs(y-yo),-1);
            }
    else
        {
            Fry = 0;
        }
    Fr = sqrt(pow(Frx,2)+pow(Fry,2)); // Total repulsiom force Fr

    out->ofloatresult = Fr;
    return o_errt::err_no_error;
}

