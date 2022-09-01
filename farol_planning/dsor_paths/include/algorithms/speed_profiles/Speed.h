#pragma once


/** 
 *  @brief     Abstract class to serve as the base for speed as a function of gamma.
 *  @details   Since this class is abstract it cannot be instantiated. It must be inherited.
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class Speed {

  public:
    
    /**
     * @brief  Method to get the desired velocity for the virtual target on the path
     * given the path parameter (given by the value of gamma)
     *
     * @param gamma  The path parameter
     * @param tangent_norm  The norm of the tangent to the path in gamma
     *
     * @return  A double with the desired speed
     *
     * NOTE: This method is pure virtual which means it must be implemented by a 
     * class that inherits Speed
     */
    virtual double getVd(double gamma, double tangent_norm) = 0;

    /**
     * @brief  Method to get the desired acceleration for the virtual target on the path
     * given the path parameter (given by the value of gamma)
     *
     * @param gamma  The value of the path parameter
     * @param tangent_norm  The norm of the tangent to the path in gamma
     *
     * @return  A double with the desired acceleration
     *
     * NOTE: This method is pure virtual which means it must be implemented by a class
     * that inherits Speed
     */
    virtual double get_d_Vd(double gamma, double tangent_norm) = 0;

    /**
     * @brief  Method to get the default desired velocity for safety
     * when we are doing path following and want to have a backup value
     * 
     * @param gamma  The value of the path parameter
     * @param tangent_norm  The norm of the tangent to the path in gamma
     *
     * @return  A double with the default desired speed
     *
     * NOTE: THis method is pura virtual which means it must be implemented by a class
     * that inherits Speed
     */
    virtual double getDefaultVd(double gamma, double tangent_norm) = 0;

    /**
     * @brief  Virtual destructor for the abstract class
     */
    virtual ~Speed();
};

