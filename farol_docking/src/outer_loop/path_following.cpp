/** 
 *  @file   outer_loop.cpp 
 *  @brief  outer_loop algorithm source file
 *  @author Ravi Regalo
 *  @date   
*/
#include <farol_docking/outer_loop/path_following.hpp>  


PathFollowing::PathFollowing() {
}

void PathFollowing::configure(std::string param, double value){
}


//TODO make this actually good
Eigen::VectorXd PathFollowing::path_following(double Dt){
    cross_track_ = inertial_pos_[1];
    Ka_ = 1/Dt;

    // velocity profile
    U_ = 0.3;//0.3 + 0.2 * std::tanh(0.5 * (x_ - 8));
     
    //  sigma dynamics with anti-windup 
    sigma_dot_ = cross_track_ + Ka_ * (-K1_ / U_ * cross_track_ - K2_ / U_ * sigma_ - sigma_e(-K1_ / U_ * cross_track_ - K2_ / U_ * sigma_));
    if(std::abs(sigma_dot_ < 10))
        sigma_ += sigma_dot_ * Dt;

    // u = -K1/U*e - K2/U*sigma 
    yaw_correction_ = -K1_ / U_ * cross_track_ - K2_ / U_ * sigma_;

    // psi_d = path_psi + asin(sat(u)) 
    yaw_d_ = wrapToPi(M_PI + std::asin(sigma_e(yaw_correction_)));
    
    return Eigen::Vector2d(U_, yaw_d_);
} 

