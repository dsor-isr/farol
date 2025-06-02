#include "Bezier.h"
#include <math.h>
#include <stdexcept>
#include <cstdio>
#include <iostream>
#include <ros/ros.h>

Bezier::Bezier(const Eigen::VectorXd &px, const Eigen::VectorXd &py, int bez_deg, double z, double Tf) : PathSection(false)
{

    // Resize Control_points_ to 2 rows and px.size() columns
    this->Control_points_.resize(2, px.size());

    for (size_t i = 0; i < px.size(); ++i)
    {
        this->Control_points_(0, i) = px[i]; // First row: x-coordinates
        this->Control_points_(1, i) = py[i]; // Second row: y-coordinates
    }
    this->z_axis_ = z;
    /* Set the gamma max for this path to be between 0 and 1 */
    this->setMinGammaValue(0.0);
    this->setMaxGammaValue(1.0);
    
    this->Tf_ = Tf;
}

/* Compute the path section equation */
Eigen::Vector3d Bezier::eq_pd(double t)
{

    Eigen::Vector3d pd_t;
    //ROS_INFO("Gamma value: %f", t);
    /* Make sure the path parameter is betwen 0 and 1*/
    t = this->limitGamma(t);
    
    int numCPoints = Control_points_.cols();
    Eigen::Vector2d Pos = deCasteljau(t, Control_points_, numCPoints, numCPoints);

    /* Store position */
    pd_t[0] = Pos.x();
    pd_t[1] = Pos.y();
    pd_t[2] = this->z_axis_;
    
    return pd_t;
}

/* Compute the derivative of the path section */
Eigen::Vector3d Bezier::eq_d_pd(double t)
{

    Eigen::Vector3d d_pd_t;

    /* Make sure the path parameter is betwen 0 and 1*/
    t = this->limitGamma(t);

    Eigen::Matrix2Xd Der_P = bezier_derivative(Control_points_);
    int numCPoints = Der_P.cols();
    Eigen::Vector2d Der = deCasteljau(t, Der_P, numCPoints, numCPoints);

    /* Store Derivative */
    d_pd_t[0] = Der.x();
    d_pd_t[1] = Der.y();
    d_pd_t[2] = this->z_axis_;

    return d_pd_t;
}

/* Compute the second derivative of the path section */
Eigen::Vector3d Bezier::eq_dd_pd(double t)
{

    Eigen::Vector3d dd_pd_t;

    /* Make sure the path parameter is betwen 0 and 1*/
    t = this->limitGamma(t);

    Eigen::Matrix2Xd Der_P = bezier_derivative(Control_points_);
    Eigen::Matrix2Xd Sec_der_P = bezier_derivative(Der_P);
    int numCPoints = Sec_der_P.cols();
    Eigen::Vector2d Der = deCasteljau(t, Sec_der_P, numCPoints, numCPoints);

    /* Store Derivative */
    dd_pd_t[0] = Der.x();
    dd_pd_t[1] = Der.y();
    dd_pd_t[2] = this->z_axis_;

    return dd_pd_t;
}

/**
 * Method to return the closest point to the path
 * By default just calls the Gradient Descent algorithm
 *
 * TODO: Implement GJK algorithm...
 */

double Bezier::getClosestPointGamma(Eigen::Vector3d &coordinate)
{

    Eigen::Vector2d P = coordinate.head<2>(); // Extracts [x, y]

    Eigen::Matrix2Xd sum = Control_points_.colwise() - P;

    Eigen::Matrix2Xd square = multiply_Bezier(sum, sum);

    auto result = GJK(square, 1e10, 1e-6, 0, 1);

    auto sq_dist= deCasteljau(result.second, square, square.cols(), square.cols());
    ROS_INFO("gOT INTO gjk");
    ROS_INFO("Squared distance to closest: %lf", sq_dist);

    return result.second;
}

/* Compute control points of the derivative of a Bézier Curve */
Eigen::Matrix2Xd Bezier::bezier_derivative(const Eigen::Matrix2Xd &P)
{
    int dim = P.rows();
    int n = P.cols();
    int degree = n - 1;

    Eigen::Matrix2Xd P_derivative(dim, degree);
    for (int i = 0; i < degree; ++i)
    {
        P_derivative.col(i) = degree * (P.col(i + 1) - P.col(i));
    }
    return P_derivative;
}

/* GJK algorithm implemantation */
std::pair<double, double> Bezier::GJK(const Eigen::Matrix2Xd &P, double alpha, double epsilon, double u_start, double u_end)
{
    Eigen::VectorXd sq_Dist = P.row(0) + P.row(1);

    // Find the lower and upper bounds of the distance
    double lower = sq_Dist.minCoeff(); // Minimum of the summed rows
    double upper = std::min(sq_Dist(0), sq_Dist(sq_Dist.cols() - 1));

    // Update alpha if the upper bound is smaller than the current alpha
    if (upper < alpha)
    {
        alpha = upper;
    }

    // Check if the difference between the bounds is smaller than the tolerance
    if (upper - lower < epsilon)
    {
        // Return the midpoint of the current segment as the u-value
        double u_val = (u_start + u_end) / 2;
        return {alpha, u_val};
    }
    else
    {
        // Subdivide the Bézier curve at u = 0.5
        Eigen::Matrix2Xd A(2, P.cols()), B(2, P.cols());
        auto Parts = divide_bezier(P, 0.5);
        A = Parts.first;
        B = Parts.second;

        // Recursively find the minimum distance in the left part (A)
        auto resultLeft = GJK(A, alpha, epsilon, u_start, (u_start + u_end) / 2);
        double alpha1 = resultLeft.first;
        double u_left = resultLeft.second;

        // Recursively find the minimum distance in the right part (B)
        auto resultRight = GJK(B, alpha, epsilon, (u_start + u_end) / 2, u_end);
        double alpha2 = resultRight.first;
        double u_right = resultRight.second;

        // Return the closest point (with the smaller alpha value)
        if (alpha1 < alpha2)
        {
            return {alpha1, u_left}; // Closest point was in the left part
        }
        else
        {
            return {alpha2, u_right}; // Closest point was in the right part
        }
    }
}

/* Divide Bezier into two parts */
std::pair<Eigen::Matrix2Xd, Eigen::Matrix2Xd> Bezier::divide_bezier(const Eigen::Matrix2Xd &P, double u)
{
    int n = P.cols(); // Number of control points

    // Initialize P1 and P2
    Eigen::Matrix2Xd P1(P.rows(), n);
    Eigen::Matrix2Xd P2(P.rows(), n);

    // Loop to perform the division using Casteljau's algorithm
    for (int i = 0; i < n; ++i)
    {
        P1.col(i) = deCasteljau(u, P.leftCols(i + 1), i + 1, i + 1);
        P2.col(i) = deCasteljau(u, P.rightCols(n - i), n - i, n - i);
    }

    return {P1, P2};
}

/* De Casteljau's algorithm to get specific points on the curve*/
Eigen::Vector2d Bezier::deCasteljau(double u, const Eigen::Matrix2Xd &pts, int i, int j)
{
    if (i == 1)
    {
        return pts.col(j - 1);
    }
    else
    {
        Eigen::Vector2d left = deCasteljau(u, pts, i - 1, j);
        Eigen::Vector2d right = deCasteljau(u, pts, i - 1, j - 1);
        return u * left + (1 - u) * right;
    }
}

Eigen::Matrix2Xd Bezier::multiply_Bezier(const Eigen::Matrix2Xd &P1, const Eigen::Matrix2Xd &P2)
{
    int np1 = P1.cols() - 1;
    int np2 = P2.cols() - 1;

    Eigen::Matrix2Xd product_P = Eigen::Matrix2Xd::Zero(P1.rows(), np1 + np2 + 1);

    for (int i = 0; i <= np1; ++i)
    {
        for (int j = 0; j <= np2; ++j)
        {
            double val1 = std::tgamma(np1 + 1) / (std::tgamma(i + 1) * std::tgamma(np1 - i + 1));
            double val2 = std::tgamma(np2 + 1) / (std::tgamma(j + 1) * std::tgamma(np2 - j + 1));
            double val3 = std::tgamma(np1 + np2 + 1) / (std::tgamma(i + j + 1) * std::tgamma(np1 + np2 - i - j + 1));

            product_P.col(i + j) += P2.col(j).cwiseProduct(P1.col(i)) * (val1 * val2 / val3);
        }
    }
    return product_P;
}