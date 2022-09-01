/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.

Dcon't you miss the danger ...
*/
#include "farol_gimmicks_library/FarolGimmicks.h"
 

// @.@ Method for converting spherical coordinates into cartesian. Used mainly with usbl fixes
void FarolGimmicks::spherical_to_cartesian(double bearing, double elevation, double range, double *out_pos_cart){
	
    out_pos_cart[0]=sin(bearing)*range*cos(elevation);
    out_pos_cart[1]=cos(bearing)*range*cos(elevation);
    out_pos_cart[2]=range*sin(elevation);

    ROS_WARN("X rel cartesian: %.2f | Y rel cartesian: %.2f | Depth rel cartesian: %f",out_pos_cart[0], out_pos_cart[1], out_pos_cart[2]);
}



// @.@ Method for returning sign of a value 1: + | 0: 0 | -1: -
int FarolGimmicks::signVal(double v)
{
  	return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

// @.@ Method for wrapping angles to specified range. 
double FarolGimmicks::wrap2pi(double theta, const int mode){
    if (mode == 1 || mode == 0)
        ;
    else
        return 0;

    theta = fmod(theta + (mode * PI), 2 * PI);
    if (theta < 0)
        theta += 2 * PI;
    theta = theta - mode * PI;
    return theta ;
}

// @.@ Wrap angle between [0, 2PI]
double FarolGimmicks::wrapTo2pi(double in)
{
  return in < 0 ? 2 * PI + in : in;
}

// @.@ Difference between two angles
double FarolGimmicks::angleDiff(double a, double b)
{
    double aux = fmod(a - b + PI, 2 * PI);
    if (aux < 0) aux += (2 * PI);
    aux = aux - PI;
    return aux;
}

