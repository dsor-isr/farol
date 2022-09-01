/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico

Don't you miss the danger
*/

#pragma once

/**
 * @brief Contains Formation variables for cooperative path following 
 * 
 */
class Formation {
  public:
    int id;     ///< vehicle id         
    double x;   ///< x value to change xi and xe in section 
    double y;   ///< y value to change yi and ye in section
    Formation() {
        id = 0;
        x = 0;
        y = 0;
    }
};
