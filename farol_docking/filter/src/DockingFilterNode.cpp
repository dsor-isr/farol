/*
 * Developer: Ravi Regalo ravi.regalo@tecnico.ulisboa.pt Instituto Superior Tecnico
 */
#include "DockingFilterNode.h"


// ( . Y . ) Function to load a parameter that is a matrix
Eigen::MatrixXd load_matrix_parameter(ros::NodeHandle &_nh, std::string const &parameter_name, Eigen::MatrixXd default_value){
		Eigen::MatrixXd parameter = default_value;
		std::vector<double> temp;
    if (_nh.getParam(parameter_name, temp)) {
        int size = std::sqrt(temp.size()); // Matrix is square, so size = sqrt(vector length)
        if (size * size == temp.size()) {
            Eigen::MatrixXd parameter = Eigen::Map<Eigen::MatrixXd>(temp.data(), size, size);
            // ROS_IFO_STREAM("Loaded process covariance matrix:\n" << parameter);
        } else {
            ROS_ERROR("Invalid process covariance matrix size.");
        }
    } else {
        ROS_ERROR("Failed to load process covariance.");
    }
    return parameter;
}

// ( . Y . ) Function to convert a vector in 3D space from (range, bearin, elevation) -> (x, y, z)
Eigen::Vector3d rbe_to_xyz(Eigen::Vector3d rbe) {
  Eigen::Vector3d res; 
  res(0) = rbe(0) * cos(rbe(2)) * cos(rbe(1));
  res(1) = rbe(0) * cos(rbe(2)) * sin(rbe(1));
  res(2) = rbe(0) * sin(rbe(2));
  return res;
}

// ( . Y . ) Constructor
DockingFilterNode::DockingFilterNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();
  timer_.start();
}

// ( . Y . ) Destructor
DockingFilterNode::~DockingFilterNode() {

  // +.+ shutdown publishers
  state_pub_.shutdown();

  // +.+ shutdown subscribers
  sub_velocity_.shutdown();
  sub_orientation_.shutdown();
  sub_usbl_fix_.shutdown();
  sub_usbl_accoms_.shutdown();   

  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}


void DockingFilterNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for DockingFilterNode");
  //sub_reset_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "filter/topics/subscribers/reset", "reset"), 10, &DockingFilterNode::resetCallback, this);
  // sub_tuning_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/tuning", "tuning"), 10, &DockingFilterNode::tuningCallback, this);
  sub_velocity_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/velocity", "velocity"), 10, &DockingFilterNode::measurement_callback, this);
  sub_orientation_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/orientation", "orientation"), 10, &DockingFilterNode::measurement_callback, this);
  sub_position_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/position", "position"), 10, &DockingFilterNode::measurement_callback, this);
  sub_usbl_fix_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/usbl_fix", "usbl_fix"), 10, &DockingFilterNode::usbl_callback, this);
  sub_usbl_accoms_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/usbl_accoms", "usbl_accoms"), 10, &DockingFilterNode::usbl_callback, this);
  sub_reset_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/reset", "reset"), 10, &DockingFilterNode::reset_callback, this);
}


void DockingFilterNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for DockingFilterNode");
  state_pub_ = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "docking_state"), 10);
  console_state_pub_ = nh_private_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state_console", "State_docking"), 10);
  debug_pub_ = nh_private_.advertise<farol_docking::FilterDebug>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug", "filter_debug"), 10);
  debug2_pub_ = nh_private_.advertise<geometry_msgs::Point>("usbl_debug", 10);
}


void DockingFilterNode::initializeServices() {
  ROS_INFO("Initializing Services for DockingFilterNode");
}


void DockingFilterNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / node_frequency_), &DockingFilterNode::timerIterCallback, this);
}


void DockingFilterNode::loadParams() {
  ROS_INFO("Load the DockingFilterNode parameters");
  node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);
  debug_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", false);
  usbl_outlier_rejection_ = FarolGimmicks::getParameters<double>(nh_private_, "usbl_outlier_rejection", false);
  h_filter_.usbl_outlier_rejection_ = usbl_outlier_rejection_;
  r_filter_.usbl_outlier_rejection_ = usbl_outlier_rejection_;
  v_filter_.usbl_outlier_rejection_ = usbl_outlier_rejection_;
  dvl_outlier_rejection_ = FarolGimmicks::getParameters<double>(nh_private_, "dvl_outlier_rejection", false);
  
  // Horizontal filter parameters
  h_filter_.measurement_noise_ = load_matrix_parameter(nh_private_, "horizontal/measurement_noise", 0.1*Eigen::Matrix2d::Identity());
  h_filter_.process_noise_ = load_matrix_parameter(nh_private_, "horizontal/process_noise", Eigen::Matrix4d::Identity());
  h_filter_.state_cov_ = load_matrix_parameter(nh_private_, "horizontal/initial_cov", Eigen::Matrix4d::Identity());
  h_filter_.initial_state_cov_ = h_filter_.state_cov_;
  h_filter_.outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "horizontal/outlier_treshold", 4.61);
  ROS_INFO_STREAM("horizontal/measurement_noise:"<< h_filter_.measurement_noise_);
  ROS_INFO_STREAM("horizontal/process_noise:"<< h_filter_.process_noise_);
  // Rotational filter parameters
  r_filter_.measurement_noise_ = FarolGimmicks::getParameters<double>(nh_private_, "rotational/measurement_noise", 0.1);
  r_filter_.process_noise_ = FarolGimmicks::getParameters<double>(nh_private_, "rotational/process_noise", 1);
  r_filter_.state_cov_ = FarolGimmicks::getParameters<double>(nh_private_, "rotational/initial_cov", 1);
  r_filter_.initial_state_cov_ = r_filter_.state_cov_;
  r_filter_.outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "rotational/outlier_treshold", 2.71);
  ROS_INFO_STREAM("rotational/measurement_noise:"<< r_filter_.measurement_noise_);
  ROS_INFO_STREAM("rotational/process_noise:"<< r_filter_.process_noise_);

  // maybe later vertical filter as well
  v_filter_.measurement_noise_ = FarolGimmicks::getParameters<double>(nh_private_, "vertical/measurement_noise", 0.1);
  v_filter_.process_noise_ = FarolGimmicks::getParameters<double>(nh_private_, "vertical/process_noise", 1);
  v_filter_.state_cov_ = FarolGimmicks::getParameters<double>(nh_private_, "vertical/initial_cov", 0.3);
  v_filter_.initial_state_cov_ = v_filter_.state_cov_;
  v_filter_.outlier_threshold_ = FarolGimmicks::getParameters<double>(nh_private_, "vertical/outlier_treshold", 2.71);
  ROS_INFO_STREAM("vertical/measurement_noise:"<< v_filter_.measurement_noise_);
  ROS_INFO_STREAM("vertical/process_noise:"<< v_filter_.process_noise_);
}


void DockingFilterNode::reset_callback(const std_msgs::Empty &msg){
  h_filter_.reset();
  r_filter_.reset();
  v_filter_.reset();
}

void DockingFilterNode::measurement_callback(const dsor_msgs::Measurement &msg) {
  if (msg.header.frame_id.find("ahrs") != std::string::npos && !msg.value.empty()) 
  {
    if(!r_filter_.initialized_) // keep only last message if not initialized
    {   
      r_filter_.input_buffer_.clear();        
      r_filter_.input_time_buffer_.clear();  
    }
    if(yaw_derivative_){
      //compute yaw_rate from yaw:
      double time = ros::Time::now().toSec();
      double Dt = time - prev_yaw_time_;
      r_filter_.input_buffer_.push_back((msg.value[2]- prev_yaw_)/Dt);        
      r_filter_.input_time_buffer_.push_back(time);  
      prev_yaw_ = msg.value[2];
      prev_yaw_time_ = time;
    }    
    else {
      r_filter_.input_buffer_.push_back(msg.value[5]);        
      r_filter_.input_time_buffer_.push_back(msg.header.stamp.toSec());  
    }
    

  } 
  else if (msg.header.frame_id.find("dvl") != std::string::npos && msg.value.size() >= 2) 
  {
    if(!h_filter_.initialized_) // keep only last message if not initialized
    {  
      dvl_outlier_window_.clear();  
      h_filter_.input_buffer_.clear();        
      h_filter_.input_time_buffer_.clear();  
    } 
    Eigen::Vector2d dvl(msg.value[0], msg.value[1]);
    
    // DVL Outlier rejection using median absolute deviation (Morgado et al)
    if(dvl_outlier_rejection_){
      dvl_outlier_window_.push_back(dvl);
      if (dvl_outlier_window_.size() > WINDOW_SIZE)
          dvl_outlier_window_.erase(dvl_outlier_window_.begin());
      if (isOutlier(dvl)){
          ROS_WARN_STREAM("DVL measurement rejected as outlier: (" << dvl(0) << ", " << dvl(1) << ")");
          return;
      }
    }

    h_filter_.input_buffer_.push_back(dvl);
    h_filter_.input_time_buffer_.push_back(msg.header.stamp.toSec()); 
  } 
  else if (msg.header.frame_id.find("depth") != std::string::npos && msg.value.size() >= 1) 
  {
    if(!v_filter_.initialized_) // keep only last message if not initialized
    {   
      v_filter_.input_buffer_.clear();        
      v_filter_.input_time_buffer_.clear();  
    }    
    // compute depth rate by derivating depth cell measurements:
    double time = ros::Time::now().toSec();
    double Dt = time - prev_depth_time_;
    //TODO: maybe add here the low pass filter
    // ROS_WARN_STREAM("Depth_dot: "<<((msg.value[0]- prev_depth_)/Dt) <<" | "<<msg.value[0]<<" | "<<prev_depth_<<" | "<<Dt);
    v_filter_.input_buffer_.push_back((msg.value[0]- prev_depth_)/Dt);        
    v_filter_.input_time_buffer_.push_back(time);  
    prev_depth_ = msg.value[0];
    prev_depth_time_ = time;
  }  
}

void DockingFilterNode::usbl_callback(const farol_msgs::mUSBLFix &msg){
  if(msg.header.frame_id.find("usbl") != std::string::npos){
    if(msg.type == 0){
      usbl_auv_.segment<1>(0) << msg.range;
      usbl_state_.set(0, true);
    }
    else{
      usbl_auv_.segment<2>(1) << msg.bearing, msg.elevation;
      usbl_state_.set(1, true);
    }
  }else{
    usbl_dock_ << usbl_auv_(0), msg.bearing, msg.elevation;
    usbl_state_.set(2, true);
  }
}

// ( . Y . ) obtain the relative position of AUV in dock D frame
Sophus::SE3d DockingFilterNode::extract_observations(){  

  // Make the ranges equal to the average of dock an
  usbl_dock_(0) = (usbl_dock_(0)+usbl_auv_(0))/2; 
  usbl_auv_(0) = usbl_dock_(0);

  // Convert (range,bearing,elevation) -> (x,y,z)
  Eigen::Vector3d xyz_auv = rbe_to_xyz(usbl_auv_);
  Eigen::Vector3d xyz_dock = rbe_to_xyz(usbl_dock_);

  // Compute translation (AUV in Dock frame) 
  Eigen::Vector3d t = xyz_dock;

  // 2. Normalize both vectors
  Eigen::Vector3d v1 = xyz_dock.normalized();    // in Dock frame
  Eigen::Vector3d v2 = -xyz_auv.normalized();    // negate to align orientation

  // 3. Compute rotation that aligns v1 to v2
  double cos_theta = v1.dot(v2);
  Eigen::Vector3d axis = v1.cross(v2);

  Sophus::SO3d R;
  // Handle the case that the vectors are almost aligned
  if (axis.norm() < 1e-6) {
    if (cos_theta > 0.9999) {
      R = Sophus::SO3d();  // identity
    } else {
      Eigen::Vector3d ortho = v1.unitOrthogonal();
      R = Sophus::SO3d(Eigen::AngleAxisd(M_PI, ortho).toRotationMatrix());
    }
  } else {
    axis.normalize();
    double theta = std::acos(std::clamp(cos_theta, -1.0, 1.0));
    Eigen::Vector3d so3_vec = theta * axis;
    R = Sophus::SO3d::exp(so3_vec);
  }
  
  return Sophus::SE3d(R, t);
}


Eigen::Vector4d computeMedianUSBL(const std::vector<Eigen::Vector4d>& fixes) {
    auto median = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        size_t n = v.size();
        return n % 2 == 0 ? (v[n / 2 - 1] + v[n / 2]) / 2.0 : v[n / 2];
    };
    std::vector<double> x, y, z, yaw;
    for (const auto& fix : fixes) {x.push_back(fix(0)); y.push_back(fix(1)); z.push_back(fix(2)); yaw.push_back(fix(3)); }
    return {median(x), median(y), median(z), median(yaw)};
}

double computeMedian(std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    size_t n = values.size();
    return n % 2 == 0 ? (values[n / 2 - 1] + values[n / 2]) / 2.0 : values[n / 2];
}

Eigen::Vector2d computeMedian(const std::vector<Eigen::Vector2d>& buffer)
{
    std::vector<double> surge, sway;
    for (const auto& vec : buffer) 
    {
        surge.push_back(vec(0));
        sway.push_back(vec(1));
    }
    return {computeMedian(surge), computeMedian(sway)};
}

bool DockingFilterNode::isOutlier(const Eigen::Vector2d& measurement)
{
    if (dvl_outlier_window_.size() < WINDOW_SIZE) return false; // Not enough data to decide

    // Compute the median of the current sliding window
    Eigen::Vector2d median = computeMedian(dvl_outlier_window_);

    // Compute the deviations from the median
    std::vector<double> surge_deviations, sway_deviations;
    for (const auto& vec : dvl_outlier_window_) 
    {
        surge_deviations.push_back(std::abs(vec(0) - median(0)));
        sway_deviations.push_back(std::abs(vec(1) - median(1)));
    }

    // Compute the MAD for surge and sway
    double mad_surge = computeMedian(surge_deviations);
    double mad_sway = computeMedian(sway_deviations);

    // Scale MAD by a constant factor (1.4826 for Gaussian distributions)
    Eigen::Vector2d scaled_mad(1.4826 * mad_surge, 1.4826 * mad_sway);

    // Compute the absolute deviation of the measurement
    Eigen::Vector2d deviation = (measurement - median).cwiseAbs();

    // Threshold based on scaled MAD
    return (deviation.array() > (3.0 * scaled_mad.array())).any(); // 3-sigma rule
}


void DockingFilterNode::timerIterCallback(const ros::TimerEvent &event) {
  // Proper initialization of the filter using the median of the first 5 measurements
  if(!h_filter_.initialized_ || !r_filter_.initialized_ || !v_filter_.initialized_){
    // complete set of fixes has been received
    if(usbl_state_.all()){
      // save measurement into initializer buffer
      initializer_measurements_.push_back(observations_);
      // if buffer has already enough measurements for initalization
      if(initializer_measurements_.size()>4){
        // TODO change this to make the median of the USBL measurements themselves
        observations_ = computeMedianUSBL(initializer_measurements_);

        // v_filter will go de cona
        // initialize the filters
        h_filter_.initialize(observations_.head<2>());
        r_filter_.initialize(observations_(3));
        v_filter_.initialize(observations_(2));
      }
      usbl_state_.reset();
    }
    return;
  }

  // compute state prediction from received ahrs and dvl message
  // v_filter will go de cona
  r_filter_.predict();
  h_filter_.predict(r_filter_.state_vec_);
  v_filter_.predict();

  // update if new set of usbl messages has been received
  if(usbl_state_.all()){
    // compute SE3 transform from the set of usbl measurements
    observations_ = extract_observations();
    // probaly split into translation and rotation 
    // v_filter will go de cona
    h_filter_.update(observations_.head<2>());
    r_filter_.update(observations_(3));
    v_filter_.update(observations_(2));
    
    usbl_state_.reset();
  }

  // publish the estimated state
  state_msg_.header.stamp = ros::Time::now();
  state_msg_.header.frame_id = "docking_station";
  state_msg_.x = h_filter_.state_vec_(0);
  state_msg_.y = h_filter_.state_vec_(1);
  state_msg_.xy_cov[0] = h_filter_.state_cov_(0);
  state_msg_.xy_cov[1] = h_filter_.state_cov_(1);
  state_msg_.xy_cov[2] = h_filter_.state_cov_(2);
  state_msg_.xy_cov[3] = h_filter_.state_cov_(3);
  state_msg_.z = v_filter_.state_vec_;
  state_msg_.z_cov = v_filter_.state_cov_;
  state_msg_.yaw = r_filter_.state_vec_;
  state_msg_.yaw_cov = r_filter_.state_cov_;
  state_pub_.publish(state_msg_);

  // publish state in the inertial NED reference frame 
  console_state_msg_.header.stamp = ros::Time::now();
  inertial_pos_ = Rot2D(-dock_pose_(3)) * h_filter_.state_vec_.head(2) + dock_pose_.head(2);
  console_state_msg_.X = inertial_pos_(1);    // East
  console_state_msg_.Y = inertial_pos_(0);    // North
  console_state_msg_.Yaw = radiansToDegrees360(wrapToPi(-dock_pose_(3) + r_filter_.state_vec_));
  console_state_msg_.Z = v_filter_.state_vec_ + dock_pose_(2);
  console_state_pub_.publish(console_state_msg_);

  // publish debug msg
  if(debug_){
    debug_msg_.header.stamp = ros::Time::now();

    // horizontal filter debug info
    inertial_pos_ = Rot2D(-dock_pose_(3)) * observations_.head<2>() + dock_pose_.head(2);
    debug_msg_.h_meas_x = observations_(0);
    debug_msg_.h_meas_y = observations_(1);
    debug_msg_.h_meas_x_NED = inertial_pos_(0);
    debug_msg_.h_meas_y_NED = inertial_pos_(1);
    debug_msg_.h_mahalanobis = h_filter_.mahalanobis_distance_;
    debug_msg_.h_outlier_rejected = h_filter_.outlier_rejected_;
    debug_msg_.h_innovation_vec[0] = h_filter_.innovation_vector_(0);
    debug_msg_.h_innovation_vec[1] = h_filter_.innovation_vector_(1);
    debug_msg_.h_innovation_mat[0] = h_filter_.innovation_matrix_(0);
    debug_msg_.h_innovation_mat[1] = h_filter_.innovation_matrix_(1);
    debug_msg_.h_innovation_mat[2] = h_filter_.innovation_matrix_(2);
    debug_msg_.h_innovation_mat[3] = h_filter_.innovation_matrix_(3);
    debug_msg_.h_k[0] = h_filter_.K_(0);
    debug_msg_.h_k[1] = h_filter_.K_(1);
    debug_msg_.h_k[2] = h_filter_.K_(2);
    debug_msg_.h_k[3] = h_filter_.K_(3);
    debug_msg_.h_k[4] = h_filter_.K_(4);
    debug_msg_.h_k[5] = h_filter_.K_(5);
    debug_msg_.h_k[6] = h_filter_.K_(6);
    debug_msg_.h_k[7] = h_filter_.K_(7);

    // vertical filter debug info
    debug_msg_.v_meas_z = observations_(2);
    debug_msg_.v_meas_z_NED = prev_depth_;
    debug_msg_.v_mahalanobis = v_filter_.mahalanobis_distance_;
    debug_msg_.v_outlier_rejected = v_filter_.outlier_rejected_;
    debug_msg_.v_innovation_vec = v_filter_.innovation_vector_;
    debug_msg_.v_innovation_mat = v_filter_.innovation_matrix_;
    debug_msg_.v_k = v_filter_.K_;

    // rotational filter debug info
    debug_msg_.r_meas_yaw = observations_(3);
    debug_msg_.r_meas_yaw_NED = radiansToDegrees360(wrapToPi(-dock_pose_(3) + observations_(3)));
    debug_msg_.r_mahalanobis = r_filter_.mahalanobis_distance_;
    debug_msg_.r_outlier_rejected = r_filter_.outlier_rejected_;
    debug_msg_.r_innovation_vec = r_filter_.innovation_vector_;
    debug_msg_.r_innovation_mat = r_filter_.innovation_matrix_;
    debug_msg_.r_k = r_filter_.K_;
    
    debug_pub_.publish(debug_msg_);
  }
  return;
}


// ( . Y . ) Main  
int main(int argc, char** argv)
{
  // node set up:
  ros::init(argc, argv, "docking_filter_node"); //node name

  // node handle
  ros::NodeHandle nh;

  // private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type DockingFilterNode");

  // instantiate an DockingFilterNode class object and pass in pointers to nodehandle public and private for constructor to use
  DockingFilterNode df(&nh,&nh_private);

  // Going into spin: let the callbacks do all the work
  ros::spin();

  return 0;
}


