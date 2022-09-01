/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico 

Don't you miss the danger
*/

#pragma once

/**
 * @brief Contains Section variables for path followin algorithms
 * 
 * @note nVehicle is always -1, probably not being used
 */
class Section {
  public:
    int type;          ///< 1= WP; 2=Line; 3=Arc; 4=Depth
    double xi;         ///< initial x of section
    double yi;         ///< initial y of section
    double xc;         ///< x of center of arc (-1 if line or point)
    double yc;         ///< y of center of arc (-1 if line or point)
    double xe;         ///< ending x of section
    double ye;         ///< ending y of section
    float velocity;    ///< velocity desired of the vehicle
    int adirection;    ///< -1 if vehicle is turning clockwise, -1 otherwise (only applied to arcs)
    float radius;      ///< adius of the arc
    float heading;     ///< yaw of the vehicle
    float time;        ///< only used for point, depth and alt, time to use the reference
    int nVehicle;      ///< number of the vehicle (possible id)
    double gamma_s;    ///< Starting gamma (not normalized)
    double gamma_e;    ///< Ending gamma (not normalized) 
    float depth;       ///< Depth of the section
    Section() {
        type = 0;
        xi = 0;
        yi = 0;
        xc = -1;
        yc = -1;
        xe = 0;
        ye = 0;
        velocity = 0;
        adirection = 0;
        radius = 0;
        heading = -1;
        time = -1;
        nVehicle = -1;
        gamma_s = 0;
        gamma_e = 0; 
        depth = 0.0;
    }
};
