#pragma once

#include "Speed.h"

/** 
 *  @brief     Class that implements a constant speed value requirement. 
 *             This class will receive in its constructor the desired velocity 
 *             for the speed in the inertial frame and will implement the 
 *             corresponding speed in the path frame
 *  @details   This class is used as a part of the speeds  library
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class ConstVehicleSpeed : public Speed {
  
  public:
    
    
    /**
     * @brief  Constructor for the ConstVehicleSpeed class. Receives as a parameter
     * a double which represents the desired speed of the vehicle in the inertial frame
     *
     * @param vehicle_speed  The desired speed for the vehicle in m/s
     * @param default_val  The desired default_value when the vehicle is outside the gamma range
     */
    ConstVehicleSpeed(double vehicle_speed, double default_val); 

   /**
     * @brief  Method to get the desired velocity for the virtual target on the path
     * given the path parameter (given by the value of gamma)
     *
     * @param gamma  The path parameter
     * @param tangent_norm  The norm of the tangent to the path
     *
     * @return  A double with the desired speed
     */
    double getVd(double gamma, double tangent_norm) override;

    /**
     * @brief  Method to get the desired acceleration for the virtual target on the path
     * given the path parameter (given by the value of gamma)
     *
     * @param gamma  The value of the path parameter
     * @param tangent_norm  The norm of the tangent to the path
     *
     * @return  A double with the desired acceleration
     */
    double get_d_Vd(double gamma, double tangent_norm) override;
    
    /**
     * @brief  Method to get the default desired velocity for safety
     * when we are doing path following and want to have a backup value
     *
     * @param gamma  The value of the path parameter
     * @param tangent_norm  The norm of the tangent to the path in gamma
     *
     * @return  A double with the default desired speed
     *
     */
    double getDefaultVd(double gamma, double tangent_norm) override;


  private:
    
    /**
     * @brief  Attribute used to store the desired vehicle speed
     */
    double vehicle_speed_{0.0};
    
    /**
     * @brief  The default speed to be used if we have less speed sections then
     * path sections in the path. This is the value that will be used for the other 
     * path sections (the paths that do not have a corresponding speed sections)
     * and if this is the last speed section
     */
    double default_speed_{0.0};
};
