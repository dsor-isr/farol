#include "RelativeHeading.h"

/* The constructor for the RelativeHeading Path Following Controller */
RelativeHeading::RelativeHeading(double kx, double ky, double kz, const Eigen::Vector2d &p_sat, double yaw_offset, ros::Publisher surge_pub, ros::Publisher sway_pub, ros::Publisher yaw_pub, ros::Publisher rabbit_pub) {

    /* Initiate the gains of the controller */
    this->kx_=kx;
    this->ky_=ky;
    this->kz_=kz;
    this->p_sat_=p_sat;
    this->yaw_offset_=yaw_offset;


    /* Save the publishers, to latter publish the data */
    this->surge_pub_=surge_pub;
    this->sway_pub_=sway_pub;
    this->yaw_pub_=yaw_pub;
    this->rabbit_pub_=rabbit_pub;
}

/* Method used to set the path following gains */
bool RelativeHeading::setPFGains(std::vector<double> gains) {

    /* Handle the case where the number of gains received does not coincide with the
     * number of gains of the algorithm
     */
    if(gains.size() != 4)
        return false;

    /* Initiate the gains of the controller */
    this->kx_=gains[0];
    this->ky_=gains[1];
    this->kz_=gains[2];
    this->yaw_offset_=gains[3];

    return true; //gains where updated successfully
}

/* Method that implements the control law */
void RelativeHeading::callPFController(double dt) {

    /* Get the vehicle position and orientation in the 2D plane */
    Eigen::Vector2d pos;
    Eigen::Vector2d pos_d;
    double yaw;

    /* Get information from the vehicle */
    pos << this->vehicle_state_.eta1[0], this->vehicle_state_.eta1[1];
    pos_d << this->path_state_.pd[0], this->path_state_.pd[1];
    yaw = this->vehicle_state_.eta2[2];

    /* Compute the rotation matrix from the inertial frame to the body frame */
    Eigen::Matrix2d Rbi;
    Rbi << cos(yaw), sin(yaw), 
    -sin(yaw), cos(yaw);

    /* Compute the position error in the body frame */
    Eigen::Vector2d ep;
    ep=Rbi*(pos-pos_d);

    /* PF Control law */
    Eigen::Vector2d d_pd;
    d_pd << this->path_state_.d_pd[0], this->path_state_.d_pd[1];

    /* Auxiliary variables */
    double vd = this->path_state_.vd;
    double d_vd = this->path_state_.d_vd;
    double vc = this->path_state_.vc;
    double psi = this->path_state_.psi;

    /* Compute the error between the virtual target speed and its desired speed */
    double z=this->gamma_dot_-vd-vc;
    Eigen::Vector2d ud;

    Eigen::Matrix2d K_gain;
    K_gain << this->kx_, 0.0, 
              0.0, this->ky_;
    
    // Example of logging
    //ROS_INFO_STREAM(K_gain);

    /* Compute desired surge and sway */
    Eigen::Vector2d ep_aux;
    ep_aux << this->p_sat_[0]*std::tanh(ep[0]/this->p_sat_[0]), this->p_sat_[1]*std::tanh(ep[1]/this->p_sat_[1]);
    //ROS_INFO_STREAM(this->p_sat_);
    //ROS_INFO_STREAM(ep);
    //ROS_INFO_STREAM(ep_aux);
    ud = -(K_gain * ep_aux) + (Rbi * d_pd * vd);

    
    /* Save the control values to be published later */
    this->desired_surge_ = ud[0];
    this->desired_sway_ = ud[1];

    /* Compute desired yaw */
    this->desired_yaw_ = (psi+this->yaw_offset_)*180.0/3.1415;

    /* Virtual Target Control Law */
    double aux = (ep.transpose()*Rbi*d_pd);
    this->gamma_ddot_ = - kz_*z+this->gamma_dot_ * d_vd+aux/(this->path_state_.tangent_norm);

    // Integrate to get the virtual target position
    this->gamma_dot_ += this->gamma_ddot_*dt;
    this->gamma_ += this->gamma_dot_*dt;

    /* Saturate the values of gamma */
    if (this->gamma_ < this->path_state_.gamma_min) this->gamma_ = this->path_state_.gamma_min;

    /* Path following values for debug */
    Eigen::Matrix2d Rti;
    Eigen::Vector2d ept;
    Rti << cos(psi), sin(psi), 
    -sin(psi), cos(psi);
    ept = Rti*(pos-pos_d);
    pfollowing_debug_.algorithm = "RelativeHeading";
    pfollowing_debug_.cross_track_error = ept[1];
    pfollowing_debug_.along_track_error = ept[0];
    pfollowing_debug_.yaw = vehicle_state_.eta2[2];
    pfollowing_debug_.psi = path_state_.psi;
    pfollowing_debug_.gamma = gamma_;
}

/* Method to publish_private the data computed from the algorithm */
void RelativeHeading::publish_private(){

    /* Publish the control references */
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->sway_pub_, this->desired_sway_);
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->yaw_pub_, this->desired_yaw_);

    /* Publish the virtual targets value */
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method that will run in the first iteration of the algorithm */
void RelativeHeading::start() {

    /* Publish the initial virtual target value to get the data from the path */
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool RelativeHeading::stop() {

    /**
     * Check if the gamma is greater then the gamma max of the path
     * If so, we have reached the end
     */
    if(this->gamma_ >= this->path_state_.gamma_max) return true;

    return false;
}

/* Method to reset all the algorithm data when the path following restarts */
bool RelativeHeading::reset() {

    /* Reset the velocity references */
    this->desired_surge_ = 0.0;
    this->desired_sway_ = 0.0;
    this->desired_yaw_ = 0.0;

    /* Reset the virtual target */
    this->gamma_ddot_ = 0.0;
    this->gamma_dot_ = 0.0;
    this->gamma_ = 0.0;

    return true;
}

/* Method to reset the virtual target of the vehicle (gamma) */
bool RelativeHeading::resetVirtualTarget(float value) {

    /* Reset the virtual target */
    this->gamma_ddot_ = 0.0;
    this->gamma_dot_ = 0.0;
    this->gamma_ = value;

    return true;
}
