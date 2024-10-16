/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "CfNode.h"
#include "CfAlgorithm.h"

// @.@ Constructor
CfNode::CfNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

  // Initialize state and matrices
  std_msgs::Empty aux;
  resetCallback(aux);
  timer_.start();
}

// @.@ Destructor
CfNode::~CfNode() {

  // +.+ shutdown publishers
  state_pub_.shutdown();

  // +.+ shutdown subscribers
  sub_reset_.shutdown();
  sub_position_.shutdown();
  sub_velocity_.shutdown();

  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}



// @.@ Member helper function to set up subscribers
void CfNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for CfNode");
  sub_reset_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/reset", "reset"), 10, &CfNode::resetCallback, this);
  sub_tuning_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/tuning", "tuning"), 10, &CfNode::tuningCallback, this);
  sub_estimator_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/estimator", "estimator"), 10, &CfNode::estimatorCallback, this);
  sub_no_measures_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/no_measures", "no_measures"), 10, &CfNode::nomeasuresCallback, this);

  sub_position_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 10, &CfNode::measurementCallback, this);
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &CfNode::measurementCallback, this);
  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &CfNode::measurementCallback, this);
}


// @.@ Member helper function to set up publishers
void CfNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for CfNode");

  state_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "state"), 10);
  State_pub_ = nh_private_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state_cf", "State_cf"), 10);
  meas_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/measure", "measure"), 10);
}


// @.@ Member helper function to set up services
void CfNode::initializeServices() {
  ROS_INFO("Initializing Services for CfNode");

}


// @.@ Member helper function to set up the timer
void CfNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / node_frequency_), &CfNode::timerIterCallback, this);
}

// @.@ Member helper to load parameters from parameter server
void CfNode::loadParams() {
  ROS_INFO("Load the CfNode parameters");

  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);

  // Be careful, estimator and no_measures are supposed to work better with pnly GNSS or only USBL measures
  estimator_ = false;
  no_measures_ = false;

  wn_ = FarolGimmicks::getParameters<double>(nh_private_, "wn", 0.04);
  csi_ = FarolGimmicks::getParameters<double>(nh_private_, "csi", 0.7);

  identity4_ = Eigen::Matrix4d::Identity();
                
  identity2_ = Eigen::Matrix2d::Identity();

  process_v_ << 0.0, 0.0, 1.0, 0.0,   // Process velocity matrix
               0.0, 0.0, 0.0, 1.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0;

  input_.resize(4, 2);
  input_ << 1.0, 0.0,    // input matrix
            0.0, 1.0,
            0.0, 0.0,
            0.0, 0.0;

  reject_counter_max_usbl_top_ = FarolGimmicks::getParameters<int>(nh_private_, "sensors/usbl_top/reject_counter", 5);
  outlier_tolerance_usbl_top_ = FarolGimmicks::getParameters<double>(nh_private_, "sensors/usbl_top/outlier_tolerance", 5.0);
}

void CfNode::resetCallback(const std_msgs::Empty &msg){

  state_ << 4290794.4300000000,     // Position - x old value = 4290822.1983094830
            491936.56000000000,     //            y old value = 491906.60293571133
            0.0000000000000000,     // Current velocity - vcx
            0.0000000000000000;     //                    vcy
  
  last_state_ = state_;

  yaw_ = 0.0;
  velocity_ << 0.0, 0.0;

  error_ << 0.0, 0.0;
  measure_est_ << state_(0), state_(1);

  last_dvl_ << 0.0, 0.0;

  last_gnss_ << 0.0, 0.0;

  last_usbl_ << 0.0, 0.0;

  last_ahrs_ = 0.0;

  last_K_time_ = 0.0;
}

void CfNode::initialReset(double easting, double northing){
  state_ << easting,                // Position - x
            northing,               //            y
            0.0000000000000000,     // Current velocity - vcx
            0.0000000000000000;     //                    vcy
  
  last_state_ = state_;

  yaw_ = 0.0;
  velocity_ << 0.0, 0.0;
  error_ << 0.0, 0.0;
  measure_est_ << state_(0), state_(1);

  last_dvl_ << 0.0, 0.0;
  last_gnss_ << 0.0, 0.0;
  last_usbl_ << 0.0, 0.0;
  last_ahrs_ = 0.0;
  last_K_time_ = 0.0;
}

void CfNode::tuningCallback(const cf::Tuning &msg){
  if(msg.wn > 0.0 && msg.csi > 0.0){
    wn_ = msg.wn;
    csi_ = msg.csi;
  }else{
    ROS_WARN("Invalid wn and csi values");
  }
}

void CfNode::estimatorCallback(const std_msgs::Empty &msg){
  if(estimator_){
    estimator_ = false;
  }else{
    estimator_ = true;
  }
}

void CfNode::nomeasuresCallback(const std_msgs::Empty &msg){
  if(no_measures_){
    no_measures_ = false;
  }else{
    no_measures_ = true;
  }
}

void CfNode::measurementCallback(const dsor_msgs::Measurement &msg) {
  // check the type of measurement, then save it
  for(int i = 0; i < (int) sensor_list_.size(); i++){
    if(msg.header.frame_id.find(sensor_list_[i]) != std::string::npos) measurements_.push_back(msg);
  }

  // reset filter to first measurement
  if (wait_first_pos_meas_ && msg.header.frame_id.find("usbl_top") != std::string::npos) {
    wait_first_pos_meas_ = false;

    initialReset(msg.value[0], msg.value[1]);
  }
}

void CfNode::estimation(double delta_t){
  // get newest dvl  measurement
  dsor_msgs::Measurement dvl_msg, ahrs_msg;
  dvl_msg.header.stamp.sec = 0;
  ahrs_msg.header.stamp.sec = 0;

  std::string aux;
  bool received_dvl = false, received_ahrs = false;
  for(int i = 0; i < (int) measurements_.size(); i++){
    aux = measurements_[i].header.frame_id;
    // Verify if dvl message and if measure is within range of the CF cycle time interval
    if(measurements_[i].header.stamp <= kf_time_){
      if(aux.find("dvl_bt") != std::string::npos){
        received_dvl = true;
        // Save measurement if newest or discard it
        if(measurements_[i].header.stamp > dvl_msg.header.stamp){
          dvl_msg = measurements_[i];
          measurements_.erase(measurements_.begin() + i);
        }else{
          measurements_.erase(measurements_.begin() + i);
        }
      }else if(aux.find("ahrs") != std::string::npos){
        received_ahrs = true;
        // Save measurement if newest or discard it
        if(measurements_[i].header.stamp > ahrs_msg.header.stamp){
          ahrs_msg = measurements_[i];
          measurements_.erase(measurements_.begin() + i);
        }else{
          measurements_.erase(measurements_.begin() + i);
        }
      }
    }
  }

  if(received_dvl){
    double outlier_error;
    outlier_error = sqrt(pow(dvl_msg.value[0] - last_dvl_.coeff(0), 2) + pow(dvl_msg.value[1] - last_dvl_.coeff(1), 2));
    if(outlier_error > outlier_dvl_){
      received_dvl = false;
      ROS_WARN("DVL Outlier detected on CF: %lf %lf %lf", dvl_msg.value[0], dvl_msg.value[0], outlier_error);
    }
    last_dvl_ << dvl_msg.value[0], dvl_msg.value[2];
  }

  if(received_ahrs){
    double outlier_error;
    outlier_error = abs(ahrs_msg.value[2] - yaw_);
    if(outlier_error > outlier_ahrs_){
      received_ahrs = false;
      ROS_WARN("AHRS Outlier detected on CF: %lf %lf", ahrs_msg.value[2], outlier_error);
    }
    last_ahrs_ = ahrs_msg.value[2];
  }

  // prepare dvl input and process matrices
  Eigen::Matrix4d process_matrix;
  process_matrix = identity4_ + process_v_ * delta_t;

  Eigen::MatrixXd input_matrix;
  input_matrix.resize(input_.rows(), input_.cols());
  input_matrix = input_ * delta_t;
  
  // Update yaw angle if new measure
  if(received_ahrs){
    yaw_ = ahrs_msg.value[2];
  }
  
  // Update dvl if new measure
  if(received_dvl){
    // Convert from body to inertial frame
    velocity_ << dvl_msg.value[0] * cos(yaw_) - dvl_msg.value[1] * sin(yaw_),
                dvl_msg.value[0] * sin(yaw_) + dvl_msg.value[1] * cos(yaw_);
  }

  // Add all velocity components (here add only bias and dvl input)
  total_velocity_ << state_(2), state_(3);
  total_velocity_ += velocity_;

  // calculate new state (inertial frame)
  /****************************************************
   * [x     ] = [1 0 dt 0] [x     ] + [dt 0] [dvl_x]
   * [y     ]   [0 1 0 dt] [y     ]   [0 dt] [dvl_y]
   * [bias_x]   [0 0 1  0] [bias_x]   [0  0]
   * [bias_y]   [0 0 0  1] [bias_y]   [0  0]
  *****************************************************/
  // state_ = process_matrix * state_ + input_matrix * velocity_;
  state_ = process_matrix * last_state_ + input_matrix * velocity_;
}

void CfNode::correction(double delta_t){
  
  // get measurements
  dsor_msgs::Measurement gnss_msg, usbl_msg;
  gnss_msg.header.stamp.sec = 0;
  usbl_msg.header.stamp.sec = 0;

  std::string aux;
  bool received_gnss = false, received_usbl = false;
  for(int i = 0; i < (int) measurements_.size(); i++){
    aux = measurements_[i].header.frame_id;
    // Verify if gnss or usbl message and if measure is within range of the CF cycle time interval
    if(measurements_[i].header.stamp <= kf_time_){
      if(aux.find("gnss") != std::string::npos){
        received_gnss = true;
        // Save measurement if newest or discard it
        if(measurements_[i].header.stamp > gnss_msg.header.stamp){
          gnss_msg = measurements_[i];
          measurements_.erase(measurements_.begin() + i);
        }else{
          measurements_.erase(measurements_.begin() + i);
        }
      }else if(aux.find("usbl_top") != std::string::npos){
        received_usbl = true;
        // Save measurement if newest or discard it
        if(measurements_[i].header.stamp > usbl_msg.header.stamp){
          usbl_msg = measurements_[i];
          measurements_.erase(measurements_.begin() + i);
        }else{
          measurements_.erase(measurements_.begin() + i);
        }
      }
    }else{
      ROS_WARN("Received measurement ahead of time");
    }
  }

  if(received_gnss && received_usbl){
    // calculate outlier error for gps and usbl
    double outlier_error;
    if(gnss_msg.header.stamp.sec != 0 && usbl_msg.header.stamp.sec != 0){
      outlier_error = sqrt(pow(gnss_msg.value[0] - last_state_(0), 2) + pow(gnss_msg.value[1] - last_state_(1), 2));
      if(outlier_error > outlier_gnss_){
        received_gnss = false;
        gnss_msg.header.stamp.sec = 0;
        ROS_WARN("GNSS Outlier detected on CF");
      }
      last_gnss_ << gnss_msg.value[0], gnss_msg.value[1];

      outlier_error = sqrt(pow(usbl_msg.value[0] - last_state_(0), 2) + pow(usbl_msg.value[1] - last_state_(1), 2));
      if(outlier_error > outlier_usbl_){
        received_usbl = false;
        usbl_msg.header.stamp.sec = 0;
        ROS_WARN("USBL Outlier detected on CF");
      }
      last_usbl_ << usbl_msg.value[0], usbl_msg.value[1];
    }
  }
      // if(gnss_msg.header.stamp.sec == 0 && usbl_msg.header.stamp.sec == 0) received = false;
  if(received_gnss){
    double outlier_error;
    // calculate outlier error for gps
    if(gnss_msg.header.stamp.sec != 0){
      outlier_error = sqrt(pow(gnss_msg.value[0] - last_state_(0), 2) + pow(gnss_msg.value[1] - last_state_(1), 2));
      if(outlier_error > outlier_gnss_){
        received_gnss = false;
        ROS_WARN("GNSS Outlier detected on CF");
      }
      last_gnss_ << gnss_msg.value[0], gnss_msg.value[1];
    }
  }

  if(received_usbl){
    double outlier_error;
    // calculate outlier error for usbl
    if(usbl_msg.header.stamp.sec != 0){
      outlier_error = sqrt(pow(usbl_msg.value[0] - last_state_(0), 2) + pow(usbl_msg.value[1] - last_state_(1), 2));
      if(outlier_error > outlier_tolerance_usbl_top_){
        if (reject_counter_usbl_top_ < reject_counter_max_usbl_top_) {
          reject_counter_usbl_top_ += 1;
          received_usbl = false;
          ROS_WARN("USBL Outlier detected on CF");
        } else {
          reject_counter_usbl_top_ = 0;
          received_usbl = true;
          ROS_WARN("USBL Outlier accepted on CF");
        }
      }
      last_usbl_ << usbl_msg.value[0], usbl_msg.value[1];
    }
  }

  // Update USBL or GNSS Estimator whether it's on or off
  if(received_usbl && !no_measures_) measure_est_ << usbl_msg.value[0], usbl_msg.value[1];
  if(received_gnss && !no_measures_) measure_est_ << gnss_msg.value[0], gnss_msg.value[1];
  if(estimator_){
    // Run Estimator
    measure_est_ = measure_est_ + velocity_ * delta_t;
  }

  // Calculate gains from wn and csi
  double K_delta_t;
  Eigen::MatrixXd time_constant, K_k;
  time_constant.resize(4,4);

  float k1 = 2 * csi_ * wn_;
  float k2 = wn_ * wn_;

  if(received_gnss || received_usbl || estimator_){
    // If estimator is on, the filter runs all the same frequency
    // If estimator is off, calculate the time between GNSS or USBL measures
    ros::Time new_K_time = ros::Time::now();
    if(estimator_){
      K_delta_t = delta_t;
    }else{
      if(last_K_time_ == 0){
        received_gnss = false;
        received_usbl = false;
        K_delta_t = 0;
      }
      else K_delta_t = new_K_time.toSec() - last_K_time_;
      last_K_time_ = new_K_time.toSec();
    }

    // check if gnss, usbl or both and build C and gain K matrices
    Eigen::MatrixXd C;
    Eigen::VectorXd measures;
    
    if(gnss_msg.header.stamp.sec != 0 && usbl_msg.header.stamp.sec != 0){
      Eigen::Matrix2d zero_matrix = Eigen::Matrix2d::Zero();

      C.resize(4,4);
      C << identity2_, zero_matrix,
           identity2_, zero_matrix;

      measures.resize(4);
      measures << gnss_msg.value[0],
                  gnss_msg.value[1],
                  usbl_msg.value[0],
                  usbl_msg.value[1];

      K_k.resize(4,4);
      K_k << k1, 0.0, k1, 0.0,
             0.0, k1, 0.0, k1,
             k2, 0.0, k2, 0.0,
             0.0, k2, 0.0, k2;
    // Gives priority to gnss estimation
    }else if(gnss_msg.header.stamp.sec != 0 || estimator_){
      C.resize(2,4);
      C << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0;

      measures.resize(2);
      if(estimator_){
        measures = measure_est_;
      }else{
        measures << gnss_msg.value[0],
                    gnss_msg.value[1];
      }

      K_k.resize(4,2);
      K_k << k1, 0.0,
             0.0, k1,
             k2, 0.0,
             0.0, k2;
    }else if(usbl_msg.header.stamp.sec != 0 || estimator_){
      C.resize(2,4);
      C << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0;

      measures.resize(2);
      if(estimator_){
        measures = measure_est_;
      }else{
        measures << usbl_msg.value[0],
                    usbl_msg.value[1];
      }

      K_k.resize(4,2);
      K_k << k1, 0.0,
             0.0, k1,
             k2, 0.0,
             0.0, k2;
    }

    // Calculate (add) all velocities to output total velocity inertial vector
    Eigen::MatrixXd aux_matrix;
    aux_matrix.resize(2,4);
    aux_matrix << 1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0;
    
    // error_ = measures - C * state_;
    error_ = measures - C * last_state_;
    total_velocity_ += aux_matrix * K_k * error_;
    
    // calculate gains
    time_constant << delta_t, 0.0, 0.0, 0.0,
                   0.0, delta_t, 0.0, 0.0,
                   0.0, 0.0, K_delta_t, 0.0,
                   0.0, 0.0, 0.0, K_delta_t;
    K_k = time_constant * K_k;
  }else{
    // No measurements and no estimator
    
    K_k.resize(4,2);
    K_k << k1, 0.0,
           0.0, k1,
           0.0, 0.0,
           0.0, 0.0;

    // Calculate (add) all velocities to output total velocity inertial vector
    Eigen::MatrixXd aux_matrix;
    aux_matrix.resize(2,4);
    aux_matrix << 1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0;
    total_velocity_ += aux_matrix * K_k * error_;

    time_constant << delta_t, 0.0, 0.0, 0.0,
                   0.0, delta_t, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0;

    K_k = time_constant * K_k;
  }

  // Publish GNSS or USBL Measure or estimation
  auv_msgs::NavigationStatus meas_pub_msg;
  meas_pub_msg.header.stamp = ros::Time::now();
  meas_pub_msg.position.east = measure_est_(1);
  meas_pub_msg.position.north = measure_est_(0);
  meas_pub_.publish(meas_pub_msg);
  
  // calculate state correction
  state_ += K_k * error_;
}


// @.@ Where the magic should happen.
void CfNode::timerIterCallback(const ros::TimerEvent &event) {
  ros::Time new_time = ros::Time::now();
  double delta_t = (new_time - kf_time_).toSec();
  // ROS_WARN("delta_t = %lf",delta_t);

  kf_time_ = new_time;

  // Main calculations
  estimation(delta_t);
  correction(delta_t);

  last_state_ = state_;

  // publish state
  auv_msgs::NavigationStatus state_msg;
  state_msg.position.east = state_(1);    // Vehicle position in East
  state_msg.position.north = state_(0);   // Vehicle position in North
  // Vehicle velocity expressed in body frame
  state_msg.body_velocity.x = total_velocity_(0) * cos(yaw_) + total_velocity_(1) * sin(yaw_);
  state_msg.body_velocity.y = - total_velocity_(0) * sin(yaw_) + total_velocity_(1) * cos(yaw_);
  // Vehicle yaw
  double yaw = yaw_ / (2.0 * M_PI) * 360.0;
  if(yaw < 0.0) yaw = yaw + 360.0;
  state_msg.orientation.z = yaw;
  // Vehicle velocity expressed in inertial frame (sum of dvl velocities, biases and corrections)
  state_msg.seafloor_velocity.x = total_velocity_(1);
  state_msg.seafloor_velocity.y = total_velocity_(0);
  state_msg.position_variance.east = error_(1);   // Error between measured and filtered position in East
  state_msg.position_variance.north = error_(0);  // Error between measured and filtered position in North
  state_msg.orientation_variance.x = state_(3);   // Vehicle velocity bias relative to East
  state_msg.orientation_variance.y = state_(2);   // Vehicle velocity bias relative to North
  state_msg.status = 59;
  state_msg.header.stamp = kf_time_;
  state_pub_.publish(state_msg);

  // publish state to console
  farol_msgs::mState State_msg;
  State_msg.X = state_(1);    // East
  State_msg.Y = state_(0);    // North
  State_msg.Vx = state_(3);   // bias in eastt
  // outlier rejection mahalanobys distance NOT HERE BUT SOMEWHERE
}


/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "cf_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type CfNode");

  // +.+ instantiate an CfNode class object and pass in pointers to nodehandle public and private for constructor to use
  CfNode cf(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
