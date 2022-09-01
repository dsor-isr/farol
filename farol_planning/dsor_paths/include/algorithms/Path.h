#pragma once

#include "PathSection.h"
#include "Speed.h"
#include <array>
#include <optional>
#include <tuple>
#include <utility>
#include <Eigen/Core>

/**
 * @brief  Enum to define 2 types of path:
 * 
 * - Complex - can be used to make compositions of multiple sections)
 * - Simple  - only contains one section
 */
typedef enum {SIMPLE_PATH, COMPLEX_PATH} TypePath_t;

/** 
 *  @brief     The Path class! This class implements all the meethod to 
 *             get the desired position, derivative, second derivatives, tangent,
 *             curvature and derivative norm.
 *
 *             This class stores a list of sections, and switches between sections
 *             based on the gamma value passed to the functions
 *
 *  @details   This class has the option to have a single section (for example, appropriate
 *             for a Bernoulli) and the option to have multiple section (for example, lines
 *             and arcs to make lawnmowers)
 *  @author    Marcelo Jacinto
 *  @author    Joao Quintas
 *  @author    Joao Cruz
 *  @author    Hung Tuan
 *  @version   1.0a
 *  @date      2021
 *  @copyright MIT
 */
class Path {

  public:
  

    /**
     * @brief  The constructor for the path class.
     */
     Path();

    /**
     * @brief  The destructor for the path class. Frees all the memory allocated
     * for each path section inside the vector of section
     */
    ~Path();

    
    /**
     * @brief  Method to check wether the path contains path sections or not
     *
     * @return  A boolean true if the path contains path sections in the vector 
     */
    bool isEmpty();

    /**
     * @brief  Method to add a path section to the path
     *
     * @param *path_section  A Pointer to a PathSection object (can be Line, Arc2D, 
     * Bernoulli, etc...)
     *
     * @return true if path was added with success or false if not
     */
    bool addPathSection(PathSection * path_section);

    /**
     * @brief - Method to add a speed section to the path
     *
     * @param speed_section  A Pointer to a SpeedSection object (can be ConstRabbitSpeed,
     * ConstVehicleSpeed, ...)
     *
     * @return  true if the speed was added with success or false if not
     */
    bool addSpeedSection(Speed * speed_section);

    /**
     * @brief Method to get the path section corresponding to a given gamma
     *
     * @param gamma - A double with the path parameter
     *
     * @return The PathSection object corresponding to the given gamma and the
     * internal gamma corresponding to that path section
     */
    std::tuple<PathSection*, double> getPathSection(double gamma);
   
    /**
     * @brief - Method to retrieve the position in the path given the 
     * path parameter gamma 
     *
     * @param gamma - A double with the path parameter
     *
     * @return A vector (3x1) with [x(gamma), y(gamma), z(gamma)]
     */
    std::optional<Eigen::Vector3d> eq_pd(double gamma);
    
    /**
     * @brief - Method to retrieve the derivative of the position in the 
     * path given the path parameter gamma
     *
     * @param gamma - A double with the path parameter
     *
     * @return A vector (3x1) with [x_dot(gamma, y_dot(gamma), z_dot(gamma)]
     */
    std::optional<Eigen::Vector3d> eq_d_pd(double gamma);
    
    /**
     * @brief - Method to retrieve the second derivative of the position in
     * the path given the path parameter gamma
     *
     * @param gamma - A double with the path parameter
     *
     * @return A vector (3x1) with [x_ddot(gamma), y_ddot(gamma), z_ddot(gamma)]
     */
    std::optional<Eigen::Vector3d> eq_dd_pd(double gamma);

    /**
     * @brief - Get the tangent to the path, given the gamma parameter 
     *
     * @param gamma - A double with the path parameter
     *
     * @return A double with the tangent size  
     */
    std::optional<double> tangent(double gamma);
    
    /**
     * @brief - The curvature of the path, given the gamma parameter 
     *
     * @param gamma - A double with the path parameter
     *
     * @return A double with the curvature of the path
     */
    std::optional<double> curvature(double gamma);
    
    /**
     * @brief - The norm of the derivative of the path 
     *
     * @param gamma - A double with the path parameter
     *
     * @return A double with the norm of the derivative of the path
     */
    std::optional<double> derivative_norm(double gamma);

    /**
     * @brief  Method to get the desired speed profile for a particular part of the path
     * given the path parameter gamma
     *
     * @param gamma  A double with the path parameter
     *
     * @return  A double with the value of vd
     */
    std::optional<double> eq_vd(double gamma);

    /**
     * @brief  Method to get the desired acceleration profile for a paritcular part of the path given the path parameter
     *
     * @param gamma  A double with the path parameter
     *
     * @return  A double with the valud of d_vd
     */
    std::optional<double> eq_d_vd(double gamma);
 
    /**
     * @brief  Method to retrieve the minimum and maximum gamma values allowed for the
     * current path
     *
     * @return  A pair with 2 doubles with the first being the minimum value and the second
     * the maximum value
     */
    std::pair<double, double> getMinMaxGamma();
    
    /**
     * @brief  Method to get the gamma corresponding to the closest point on
     * the path, relative to the coordinate passed as argument
     *
     * @param coordinate  The coordinate of the vehicle
     *
     * @return  A double with the gamma of the path corresponding to the closest
     * point on the path
     */
    std::optional<double> getClosestGamma(Eigen::Vector3d &coordinate);

    /** 
     * @brief  List of PathSections to follow 
     */
    std::vector<PathSection*> sections_;

  private:
    
    /** 
     * @brief  Variable to store which type of path we have.
     * By the default the path is COMPLEX_PATH, but it can become SIMPLE_PATH
     * when a new section is added.
     */
    TypePath_t type_of_path_{COMPLEX_PATH}; 

    /** 
     * @brief  A vector of pairs to store the gamma limits of each section 
     */
    std::vector<std::pair<double, double>> sections_limits_;

    /** 
     * @brief  A vector to store the velocity profiles *
     */
    std::vector<Speed*> speeds_; 

    /**
     * @brief  Auxiliar method that implements binary search to get the index
     * in the vector that contains the section respective to the given gamma.
     *
     * Since this method implements a binary search, in the worst case scenario
     * the cost is O(log(n))
     *
     * @param gamma  The path parameter value
     * @param left  The int with the left index on the vector 
     * @param right  The int with the right index on the vector
     *
     * @return  The index on the vector where we can find the path section
     * corresponding to the given vector.
     */
    int getIndexOfSection(double gamma, int left, int right);
 
    /**
     * @brief  Auxiliar variable to store the gamma that has the closest point
     * of the vehicle to the path. This value is updated every time the getClosestPoint
     * method is called. The getClosestPoint will never search for a gamma smaller than this
     * one
     */
    double gamma_closest_point_{0.0};

    /**
     * @brief  Auxiliar variable to check if it is the first time checking for the closest
     * point on the path to the vehicle (since we assume that the vehicle moves in a continuous manner
     * we then make the calculations as a progression from the calculation of the first
     * point closer to the path 
     */
    bool first_check_on_closest_point_{true};
    
    /**
     * @brief  Auxiliar variable to store the section that minimizes the error between the vehicle 
     * current position and the path
     */
    unsigned int index_section_smallest_error_{0};
};
