#include "DockingController.h"


DockingController::DockingController() {
  initialized_ = false;
  sigma_ = 0.0;
  sigma_dot_ = 0.0;
  controller_state_ =1;
}

void DockingController::tune(std::vector<double> params){
    //    2 * ksi * w0
    K1_ = 2*params[0]*params[1];
    //    w0 * w0
    K2_ = params[1]*params[1];
}

void DockingController::update_state(double x, double y, double yaw, double z = 0){
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    z_ = z;
}
void DockingController::update_speed(double u){
    u_ = u;
}

// @.@ Where the magic happens
bool DockingController::compute_update(double Dt)
{
    double u_ref,v_ref,yaw_ref;

    if(controller_state_ == 0)
    {   // Homing Phase

        // TODO: implement this
        u_ref = 0;
        v_ref = 0;
        yaw_ref = 0;
        output_ << u_ref, v_ref, yaw_ref;
        return true; 

    }else if(controller_state_ == 1)
    {   // Final Docking Phase
        
        cross_track_ = -y_;
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
        
        if(fully_actuated_){
            Eigen::Vector2d V_d(U_ * std::cos(yaw_d_), U_ * std::sin(yaw_d_));
            Eigen::Vector2d V_d_b = Rot2D(yaw_) * V_d;
            u_ref = V_d_b(0);
            v_ref = V_d_b(1);
            yaw_ref = M_PI;
        }else{
            u_ref = U_;
            v_ref = 0;
            yaw_ref = yaw_d_;
        }
        output_ << u_ref, v_ref, yaw_ref;
        return true;
    }
    return false;
    
}

