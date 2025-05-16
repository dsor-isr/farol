/** 
 *  @file   docking_filter.cpp 
 *  @brief  Docking Filter Algorithmic part source file
 *  @author Ravi Regalo
 *  @date   
*/
#include <farol_docking/controller/docking_controller.hpp>  


DockingController::DockingController() {
  phase_ = "idle";  
}

void DockingController::configure(std::string param, double value){

}

// TODO implement this shit
void DockingController::state_transition(){
    // if(controller_.x_ > 0.1 & controller_.x_ < 2  & !reached_close_){
    //     end_time_ = new_time;
    //     reached_close_=true;
    //     ROS_ERROR_STREAM(end_time_);
    //   }
}

//TODO make this actually good
Eigen::VectorXd DockingController::path_following(double Dt){
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

bool DockingController::compute(double Dt)
{
    double u_ref,v_ref,yaw_ref;

    if(phase_ == "idle"){
        return true; 
    }
    if(phase_ == "aproximation"){
        Eigen::VectorXd pf_refs;
        pf_refs = path_following(Dt);
        output_.data = Eigen::Vector3d(pf_refs[0], pf_refs[1], 0); 
        output_.frame_id = "inertial_frame";
        return true; 
    }
    else if(phase_ == "homing1")
    {   
        Eigen::VectorXd pf_refs;
        pf_refs = path_following(Dt);
        output_.data = Eigen::Vector3d(pf_refs[0], pf_refs[1], 0); 
        output_.frame_id = "dock_frame";
        return true;
    }
    else if(phase_ == "homing2"){
        Eigen::VectorXd output(6);
        output << 0,0,0,0,0,0;
        output_.data = output; 
        output_.frame_id = "dock_frame";
        return true;
    }
    else if(phase_ == "docking"){
        Eigen::VectorXd output(6);
        output << 10,0,0,0,0,0;
        output_.data = output; 
        output_.frame_id = "actuators";
        return true;
    }
    else if(phase_ == "finished"){
        return true;
    }

    return false;
}

