/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "CkfNode.h"
#include "CkfAlgorithm.h"

// @.@ Constructor
CkfNode::CkfNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

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
CkfNode::~CkfNode() {

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
void CkfNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for CkfNode");
  sub_reset_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/reset", "reset"), 10, &CkfNode::resetCallback, this);

  sub_position_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 10, &CkfNode::measurementCallback, this);
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &CkfNode::measurementCallback, this);
  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &CkfNode::measurementCallback, this);
}


// @.@ Member helper function to set up publishers
void CkfNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for CkfNode");

  state_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "state"), 10);
  State_pub_ = nh_private_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state_ckf", "State_ckf"), 10);
}


// @.@ Member helper function to set up services
void CkfNode::initializeServices() {
  ROS_INFO("Initializing Services for CkfNode");

}


// @.@ Member helper function to set up the timer
void CkfNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / node_frequency_), &CkfNode::timerIterCallback, this);
}

// @.@ Member helper to load parameters from parameter server
void CkfNode::loadParams() {
  ROS_INFO("Load the CkfNode parameters");

  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);

  // sensor configuration 
  // HARD CODED SENSORS!!!! BEWARE
  for(int i = 0; i < (int) sensor_list_.size(); i++){
    struct sensor_config sc;
    sc.name = sensor_list_[i];
    if(sc.name == "gnss"){
      sc.config.push_back(0);       // position in x
      sc.config.push_back(1);       // position in y
      sc.noise.resize(2, 2);
      sc.noise << 0.001, 0.0,       // noise
                  0.0, 0.001;
      sc.outlier_tolerance = 0.2;
      sc.reject_counter = 8;
    }else if(sc.name == "usbl"){
      sc.config.push_back(0);       // position in x
      sc.config.push_back(1);       // position in y
      sc.noise.resize(2, 2);
      sc.noise << 1.0, 0.0,         // noise
                  0.0, 1.0;
      sc.outlier_tolerance = 1.0;
      sc.reject_counter = 10;
      sc.outlier_increase = 0.3;
    }else if(sc.name == "dvl_bt"){
      sc.config.push_back(0);         // velocity in x
      sc.config.push_back(1);         // velocity in y
      sc.noise.resize(2, 2);
      sc.noise << 0.0225, 0.0,
                  0.0, 0.0225;        // noise
      sc.outlier_tolerance = 0.2;
      sc.reject_counter = 200;
    }/*else if(sc.name == "ahrs"){
      sc.config.push_back(2);     // yaw
      sc.config.push_back(5);     // yaw_rate
      sc.noise.push_back(0.001);  // noise in yaw
      sc.noise.push_back(1.0);    // noise in yaw_rate
      sc.outlier_tolerance = 0.5;
      sc.reject_counter = 12;
    }*/
    sensors_.push_back(sc);
  }
  
  // indentity4_ << 1.0, 0.0, 0.0, 0.0,    // Identity auxiliary matrix
  //                0.0, 1.0, 0.0, 0.0,
  //                0.0, 0.0, 1.0, 0.0,
  //                0.0, 0.0, 0.0, 1.0;

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
  
  process_cov_ << 0.1, 0.0, 0.0, 0.0,    // process covariance
                  0.0, 0.1, 0.0, 0.0,
                  0.0, 0.0, 0.1, 0.0,
                  0.0, 0.0, 0.0, 0.1;

  Eigen::Matrix4d input_noise;
  struct sensor_config aux;

  for(int i = 0; i < (int) sensors_.size(); i++){
    if(sensors_[i].name == "dvl_bt"){
      aux = sensors_[i];
      break;
    }
  }

  Eigen::Matrix2d zero_matrix;
  zero_matrix << 0.0, 0.0,
                0.0, 0.0;

  input_noise << aux.noise, zero_matrix,
                 zero_matrix, zero_matrix;

  complementary_cov_ = process_cov_ + input_noise;
}

void CkfNode::resetCallback(const std_msgs::Empty &msg){
  predict_cov_ << 1.0, 0.0, 0.0, 0.0,   // Set the intial predict covariance here
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 0.1, 0.0,
                  0.0, 0.0, 0.0, 0.1;

  state_ << 4290794.43,//4290822.1983094830,     // Position -         x
            491936.56,//491906.60293571133,     //                    y
            0.0000000000000000,     // Current velocity - vcx
            0.0000000000000000;     //                    vcy

  yaw_ = 0.0;
  velocity_ << 0.0, 0.0;

  update_cov_ << 1.0, 0.0, 0.0, 0.0,   // Set the intial update covariance here
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 0.1, 0.0,
                 0.0, 0.0, 0.0, 0.1;

  last_dvl_ << 0.0, 0.0;

  last_gnss_ << 0.0, 0.0;

  last_usbl_ << 0.0, 0.0;
}

void CkfNode::measurementCallback(const dsor_msgs::Measurement &msg) {
  // check the type of measurement, then save it
  measurements_.push_back(msg);
}

void CkfNode::predict(double delta_t){
  // get newest dvl  measurement
  dsor_msgs::Measurement dvl_msg, ahrs_msg;
  dvl_msg.header.stamp.sec = 0;
  ahrs_msg.header.stamp.sec = 0;

  std::string aux;
  bool received_dvl = false, received_ahrs = false;
  for(int i = 0; i < (int) measurements_.size(); i++){
    aux = measurements_[i].header.frame_id;
    // Verify if dvl message and if measure is within range of the CKF cycle time interval
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
    outlier_error = abs(sqrt(pow(dvl_msg.value[0], 2) + pow(dvl_msg.value[1], 2)) - sqrt(pow(last_dvl_.coeff(0), 2) + pow(last_dvl_.coeff(1), 2)));
    if(outlier_error > outlier_dvl_){
      received_dvl = false;
      ROS_WARN("DVL Outlier detected on CKF: %lf %lf %lf", dvl_msg.value[0], dvl_msg.value[0], outlier_error);
    }
    last_dvl_ << dvl_msg.value[0], dvl_msg.value[2];
  }

  if(received_ahrs){
    double outlier_error;
    outlier_error = abs(ahrs_msg.value[2] - yaw_);
    if(outlier_error > outlier_ahrs_){
      received_ahrs = false;
      ROS_WARN("AHRS Outlier detected on CKF: %lf %lf", ahrs_msg.value[2], outlier_error);
    }
    last_ahrs_ = ahrs_msg.value[2];
  }

  // prepare dvl input and process matrices

  Eigen::Matrix4d process_matrix, dvl_cov;
  process_matrix = identity4_ + process_v_ * delta_t;

  Eigen::MatrixXd input_matrix;
  input_matrix.resize(input_.rows(), input_.cols());
  input_matrix = input_ * delta_t;
  
  if(received_ahrs){
    yaw_ = ahrs_msg.value[2];
  }
  
  struct sensor_config dvl_sensor;
  if(received_dvl){
    // Convert from body to inertial frame
    velocity_ << dvl_msg.value[0] * cos(yaw_) - dvl_msg.value[1] * sin(yaw_),
                dvl_msg.value[0] * sin(yaw_) + dvl_msg.value[1] * cos(yaw_);

    ROS_WARN("DVL Measurement: %lf, %lf", velocity_[0], velocity_[1]);
    // ROS_WARN("AHRS Measurement: %lf", ahrs_msg.value[2]);

    // get dvl sensor PROBLEM IS HERE
    for(int i = 0; i < (int) sensors_.size(); i++){
      if(sensors_[i].name.find("dvl_bt") != std::string::npos){
        dvl_sensor = sensors_[i];
      }
    }

    Eigen::Matrix2d zero_matrix;
    zero_matrix << 0.0, 0.0,
                   0.0, 0.0;

    dvl_cov << dvl_sensor.noise, zero_matrix,
               zero_matrix, zero_matrix;
  }
  // }else{
  //   // ROS_WARN("No DVL measurement");
  //   // velocity << last_dvl,
  //   //             last_dvl;

  //   dvl_cov << 0.0, 0.0, 0.0, 0.0,
  //              0.0, 0.0, 0.0, 0.0,
  //              0.0, 0.0, 0.0, 0.0,
  //              0.0, 0.0, 0.0, 0.0;
  // }

  // calculate new state and covariance matrix
  state_ = process_matrix * state_ + input_matrix * velocity_;
  predict_cov_ = process_matrix * update_cov_ * process_matrix.transpose() + process_cov_ + dvl_cov * delta_t;
}

void CkfNode::update(double delta_t){
  
  // get measurements
  dsor_msgs::Measurement gnss_msg, usbl_msg;
  gnss_msg.header.stamp.sec = 0;
  usbl_msg.header.stamp.sec = 0;

  std::string aux;
  bool received = false;
  for(int i = 0; i < (int) measurements_.size(); i++){
    aux = measurements_[i].header.frame_id;
    // Verify if gnss or usbl message and if measure is within range of the CKF cycle time interval
    if(measurements_[i].header.stamp <= kf_time_){
      if(aux.find("gnss") != std::string::npos){
        received = true;
        // Save measurement if newest or discard it
        if(measurements_[i].header.stamp > gnss_msg.header.stamp){
          gnss_msg = measurements_[i];
          measurements_.erase(measurements_.begin() + i);
        }else{
          measurements_.erase(measurements_.begin() + i);
        }
      }else if(aux.find("usbl") != std::string::npos){
        received = true;
        // Save measurement if newest or discard it
        if(measurements_[i].header.stamp > usbl_msg.header.stamp){
          usbl_msg = measurements_[i];
          measurements_.erase(measurements_.begin() + i);
        }else{
          measurements_.erase(measurements_.begin() + i);
        }
      }
    }
  }

  if(received){
    // calculate outlier error for gps and usbl
    double outlier_error;
    if(gnss_msg.header.stamp.sec != 0 && usbl_msg.header.stamp.sec != 0){
      outlier_error = abs(sqrt(pow(gnss_msg.value[0], 2) + pow(gnss_msg.value[1], 2)) - sqrt(pow(last_gnss_.coeff(0), 2) + pow(last_gnss_.coeff(1), 2)));
      if(outlier_error > outlier_gnss_){
        gnss_msg.header.stamp.sec = 0;
        ROS_WARN("GNSS Outlier detected on CKF");
      }
      last_gnss_ << gnss_msg.value[0], gnss_msg.value[1];

      outlier_error = abs(sqrt(pow(usbl_msg.value[0], 2) + pow(usbl_msg.value[1], 2)) - sqrt(pow(last_usbl_.coeff(0), 2) + pow(last_usbl_.coeff(1), 2)));
      if(outlier_error > outlier_usbl_){
        usbl_msg.header.stamp.sec = 0;
        ROS_WARN("USBL Outlier detected on CKF");
      }
      last_usbl_ << usbl_msg.value[0], usbl_msg.value[1];

      if(gnss_msg.header.stamp.sec == 0 && usbl_msg.header.stamp.sec == 0) received = false;
    // calculate outlier error for gps
    }else if(gnss_msg.header.stamp.sec != 0){
      outlier_error = abs(sqrt(pow(gnss_msg.value[0], 2) + pow(gnss_msg.value[1], 2)) - sqrt(pow(last_gnss_.coeff(0), 2) + pow(last_gnss_.coeff(1), 2)));
      if(outlier_error > outlier_gnss_){
        received = false;
        ROS_WARN("GNSS Outlier detected on CKF");
      }
      last_gnss_ << gnss_msg.value[0], gnss_msg.value[1];
    // calculate outlier error for usbl
    }else if(usbl_msg.header.stamp.sec != 0){
      outlier_error = abs(sqrt(pow(usbl_msg.value[0], 2) + pow(usbl_msg.value[1], 2)) - sqrt(pow(last_usbl_.coeff(0), 2) + pow(last_usbl_.coeff(1), 2)));
      if(outlier_error > outlier_usbl_){
        received = false;
        ROS_WARN("USBL Outlier detected on CKF");
      }
      last_usbl_ << usbl_msg.value[0], usbl_msg.value[1];
    }
    
    
  }

  if(received){
    // get gnss and usbl sensor
    struct sensor_config gnss_sensor, usbl_sensor;
    for(int i = 0; i < (int) sensors_.size(); i++){
      if(sensors_[i].name.find("usbl") != std::string::npos){
        usbl_sensor = sensors_[i];
      }else if(sensors_[i].name.find("gnss") != std::string::npos){
        gnss_sensor = sensors_[i];
      }
    }

    // check if gnss, usbl or both
    Eigen::MatrixXd R, C;
    Eigen::VectorXd measures;
    Eigen::MatrixXd K_k, invert_K;

    // identity.resize(4,4);
    // identity << 1.0, 0.0, 0.0, 0.0,
    //             0.0, 1.0, 0.0, 0.0,
    //             0.0, 0.0, 1.0, 0.0,
    //             0.0, 0.0, 0.0, 1.0;

    float wn = 0.4;
    float csi = 0.6;
    float k1 = 2 * csi * wn;
    float k2 = wn * wn;
    // ROS_WARN("gain = %lf", k2);
    
    if(gnss_msg.header.stamp.sec != 0 && usbl_msg.header.stamp.sec != 0){
      Eigen::Matrix2d zero_matrix = Eigen::Matrix2d::Zero();
      // zero_matrix << 0.0, 0.0,
      //               0.0, 0.0;
      R.resize(4,4);
      R << gnss_sensor.noise, zero_matrix,
           zero_matrix, usbl_sensor.noise;

      C.resize(4,4);
      C << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0;

      measures.resize(4);
      measures << gnss_msg.value[0],
                  gnss_msg.value[1],
                  usbl_msg.value[0],
                  usbl_msg.value[1];

      invert_K.resize(4,4);
      K_k.resize(4,4);
      // K_k << 75.396, 0.0, 75.396, 0.0,
      //      0.0, 75.396, 0.0, 75.396,
      //      3947.6089, 0.0, 3947.6089, 0.0,
      //      0.0, 3947.6089, 0.0, 3947.6089;
      K_k << k1, 0.0, k1, 0.0,
             0.0, k1, 0.0, k1,
             k2, 0.0, k2, 0.0,
             0.0, k2, 0.0, k2;
    }else if(gnss_msg.header.stamp.sec != 0){
      R.resize(2,2);
      R << gnss_sensor.noise;

      C.resize(2,4);
      C << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0;

      measures.resize(2);
      measures << gnss_msg.value[0],
                  gnss_msg.value[1];

      invert_K.resize(2,2);
      K_k.resize(4,2);
      // K_k << 75.396, 0.0,
      //      0.0, 75.396,
      //      3947.6089, 0.0,
      //      0.0, 3947.6089;
      
      K_k << k1, 0.0,
             0.0, k1,
             k2, 0.0,
             0.0, k2;
    }else if(usbl_msg.header.stamp.sec != 0){
      R.resize(2,2);
      R << usbl_sensor.noise;

      C.resize(2,4);
      C << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0;

      measures.resize(2);
      measures << usbl_msg.value[0],
                  usbl_msg.value[1];

      invert_K.resize(2,2);
      K_k.resize(4,2);
      // K_k << 75.396, 0.0,
      //        0.0, 75.396,
      //      3947.6089, 0.0,
      //      0.0, 3947.6089;
      K_k << k1, 0.0,
             0.0, k1,
             k2, 0.0,
             0.0, k2;
    }
    
    // calculate kalman gains
    invert_K = C * predict_cov_ * C.transpose() + R * delta_t;
    // K_k = predict_cov_ * C.transpose() * invert_K.inverse();
    K_k = K_k * delta_t;

    // calculate state correction and covariance matrix
    state_ = state_ + K_k * (measures - C * state_);
    update_cov_ = (identity4_ - K_k * C) * predict_cov_;
  }else{
    // No measurements, no Update necessary
    ROS_WARN("No Update");
    update_cov_ = predict_cov_;
  }
}


// @.@ Where the magic should happen.
void CkfNode::timerIterCallback(const ros::TimerEvent &event) {
  ros::Time new_time = ros::Time::now();
  double delta_t = (new_time - kf_time_).toSec();
  // ROS_WARN("delta_t = %lf",delta_t);

  kf_time_ = new_time;
  predict(delta_t);
  update(delta_t);

  // publish state
  auv_msgs::NavigationStatus state_msg;
  state_msg.position.east = state_(1);
  state_msg.position.north = state_(0);
  state_msg.body_velocity.x = state_(3);
  state_msg.body_velocity.y = state_(2);
  state_msg.status = 59;
  state_pub_.publish(state_msg);

  // publish state to console
  farol_msgs::mState State_msg;
  State_msg.X = state_(1);    // East
  State_msg.Y = state_(0);    // North
  State_msg.Vx = state_(3);   // bias in east
  State_msg.Vy = state_(2);   // bias in north
  State_msg.Yaw = yaw_;
  State_pub_.publish(State_msg);

  // outlier rejection mahalanobys distance NOT HERE BUT SOMEWHERE
}


/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "ckf_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type CkfNode");

  // +.+ instantiate an CkfNode class object and pass in pointers to nodehandle public and private for constructor to use
  CkfNode ckf(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
