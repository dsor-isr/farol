/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "UsblFix2Pos"
#include "UsblFix2Pos.h"

/*
   @.@ CONSTRUCTOR
   */
UsblFix2Pos::UsblFix2Pos(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
  ROS_INFO("in class constructor of UsblFix2Pos");
  loadParams();
  initializeSubscribers();
  initializePublishers();
}

/*
   @.@ Destructor
   */
UsblFix2Pos::~UsblFix2Pos()
{
  // +.+ shutdown publishers
  pub_pose_fix.shutdown();
  if (p_fix_type == false){
    pub_usbl_est_state.shutdown();
    pub_usbl_est_console.shutdown();
    pub_usbl_est_console_auv0_.shutdown();
    pub_usbl_est_console_auv1_.shutdown();
  }

  // +.+ shutdown subscribers
  sub_state.shutdown();
  sub_usbl.shutdown();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

/*
   @.@ Member Helper function to set up subscribers;
   */
void UsblFix2Pos::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers for UsblFix2Pos");
  sub_usbl  = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/usbl_fix", "usbl_fix"), 1, &UsblFix2Pos::usblFixBroadcasterCallback, this);
  if (p_fix_type){
    sub_state = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/acomms_state", "acomms_state"), 1, &UsblFix2Pos::stateAcommsCallback, this);
  }
  else{
    sub_state = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state", "state"), 1, &UsblFix2Pos::stateCallback, this);
  }
}

/*
   @.@ Member helper function to set up publishers;
   */
void UsblFix2Pos::initializePublishers()
{
  ROS_INFO("Initializing Publishers for UsblFix2Pos"); 
  pub_pose_fix = nh_private_.advertise<dsor_msgs::Measurement>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/position", "position"), 1, true);
  pub_usbl_est_console = nh_private_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/console_state_usbl_estimation", "state_usbl_est"), 1, true);
  pub_usbl_est_console_auv0_ = nh_private_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/console_state_usbl_estimation_auv0", "state_usbl_est_auv0"), 1, true);
  pub_usbl_est_console_auv1_ = nh_private_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/console_state_usbl_estimation_auv1", "state_usbl_est_auv1"), 1, true);

  if (p_fix_type == false) {
    pub_usbl_est_state = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state_usbl_estimation", "usbl_est"), 1, true);
  }
}

/*
   @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
   */
void UsblFix2Pos::initializeTimers() {
  list_cleaner_timer = nh_private_.createTimer(ros::Duration(p_t_sync), &UsblFix2Pos::listTimerCallback, this);
}

/*
   @.@ Load the parameters
   */
void UsblFix2Pos::loadParams() {
  ROS_INFO("Load the UsblFix2Pos parameters");
  p_t_sync   = FarolGimmicks::getParameters<int>(nh_private_, "t_sync", 2);
  p_fix_type = FarolGimmicks::getParameters<bool>(nh_private_, "fix_type", true);
  p_meas_noise = FarolGimmicks::getParameters<double>(nh_private_, "meas_noise", true);
  name_vehicle_id_ = FarolGimmicks::getParameters<std::string>(nh_private_, "name_vehicle_id");

  auv0_source_id_ = FarolGimmicks::getParameters<int>(nh_private_, "auv0_source_id", "7");
  auv1_source_id_ = FarolGimmicks::getParameters<int>(nh_private_, "auv1_source_id", "2");
}

void UsblFix2Pos::listTimerCallback(const ros::TimerEvent &event) {
  // Clear old measurements from the usblfix list
  std::list<farol_msgs::mUSBLFix>::iterator it;
  for (it = usblfix_list.end(); it != usblfix_list.begin(); --it) {
    if((ros::Time::now() - (*it).header.stamp).toSec() < p_t_sync) {
      usblfix_list.erase(usblfix_list.begin(), it);
      break;
    }
  }
  // Clear old measurements from the stateAcomms list
  std::list<farol_msgs::stateAcomms>::iterator iter;
  for (iter = stateAcomms_list.end(); iter != stateAcomms_list.begin(); --iter) {
    if((ros::Time::now() - (*iter).header.stamp).toSec() < p_t_sync) {
      stateAcomms_list.erase(stateAcomms_list.begin(), iter);
      break;
    }
  }
  if((ros::Time::now().toSec() - state_stamp) > p_t_sync){
    initialized = false;
  }
}

/*
   @.@ Callback gps -> convert lat lon to utm and publish in a pose message
   */
// spherical = [range, bearing, elevation], cartesian = [x, y, z]
void UsblFix2Pos::transformPosition(double spherical[3], double cartesian[3]){
  // https://rbrundritt.wordpress.com/2008/10/14/conversion-between-spherical-and-cartesian-coordinates-systems/
  cartesian[0] = std::fabs(spherical[0] * std::cos(spherical[2])) * std::cos(spherical[1]);
  cartesian[1] = std::fabs(spherical[0] * std::cos(spherical[2])) * std::sin(spherical[1]);
  cartesian[2] = spherical[0] * std::sin(spherical[2]);
}

void UsblFix2Pos::getEstLatLon(double spherical[3], double latLon[2]){

  // https://stackoverflow.com/a/7835325
  double latLon_temp[2];
  latLon_temp[0] = latLon[0];
  latLon_temp[1] = latLon[1];

  latLon_temp[0] = (latLon_temp[0] * FarolGimmicks::PI) / 180.0;
  latLon_temp[1] = (latLon_temp[1] * FarolGimmicks::PI) / 180.0;

  latLon[0] = std::asin(std::sin(latLon_temp[0]) * std::cos((spherical[0]/1000.0)/R) + std::cos(latLon_temp[0])*std::sin((spherical[0]/1000.0)/R)*std::cos(spherical[1]));

  latLon[1] = latLon_temp[1] + std::atan2(std::sin(spherical[1])*std::sin((spherical[0]/1000.0)/R)*std::cos(latLon_temp[0]), std::cos((spherical[0]/1000.0)/R)- std::sin(latLon_temp[0])*std::sin(latLon[1]));

  latLon[0] = (latLon[0] * 180.0) / FarolGimmicks::PI; 
  latLon[1] = (latLon[1] * 180.0) / FarolGimmicks::PI; 

}

void UsblFix2Pos::usblFixBroadcasterCallback(const farol_msgs::mUSBLFix &msg) {

  if (!initialized) return;

  // Calculate position of the vehicle relative to the anchor state
  farol_msgs::mUSBLFix usbl = msg;

  // Dumb variables
  double cartesian[3];
  double latlon[2];
  std::copy(std::begin(state_lat_lon), std::end(state_lat_lon), std::begin(latlon));
  double spherical[3];

  // Compute Position only if Full_Fix
  if(usbl.type == usbl.FULL_FIX) {
    spherical[0] = usbl.range;
    spherical[1] =	usbl.bearing;
    spherical[2] = usbl.elevation;

  } else {

    // Add fix to a list
    usblfix_list.push_back(usbl);

    // Find if previous ranges and or angles are in sync (t_sync seconds)
    std::list<farol_msgs::mUSBLFix>::reverse_iterator it;
    for (it = usblfix_list.rbegin(); it != usblfix_list.rend(); ++it) {
      if((*it).source_name == usbl.source_name && (*it).type != usbl.type && fabs((usbl.header.stamp-(*it).header.stamp).toSec()) < p_t_sync)
        break;
    }
    // Return if no nearby pointer is found
    if(it == usblfix_list.rend()){
      return;
    }

    // Calculate position fix
    if(usbl.type == usbl.AZIMUTH_ONLY) {
      spherical[0] = (*it).range;
      spherical[1] = usbl.bearing;
      spherical[2] = usbl.elevation;
    } else {
      return;
    } 
  }

  transformPosition(spherical, cartesian);

  // Pub if usbl is an anchor
  if (p_fix_type == false){

    farol_msgs::mState console_state;
    console_state.header.stamp = usbl.header.stamp;
    console_state.X = state[1] + cartesian[1];
    console_state.Y = state[0] + cartesian[0];

    if (usbl.source_id == auv0_source_id_) { // if measured state is from auv0's source id
      pub_usbl_est_console_auv0_.publish(console_state);
    } else if (usbl.source_id == auv1_source_id_) { // if measured state is from auv1's source id
      pub_usbl_est_console_auv1_.publish(console_state);
    } else { // default
      pub_usbl_est_console.publish(console_state);
    }
    
    getEstLatLon(spherical, latlon);

    auv_msgs::NavigationStatus usbl_est;

    usbl_est.header.stamp = usbl.header.stamp;
    usbl_est.position.north = state[0] + cartesian[0];
    usbl_est.position.east = state[1] + cartesian[1];

    usbl_est.global_position.latitude = latlon[0];
    usbl_est.global_position.longitude = latlon[1];

    pub_usbl_est_state.publish(usbl_est);
  }
  else{
    
    // Find in buffer for a correspondent state that correspond to the same vehicle of the angles and if are in sync (t_sync seconds)
    std::list<farol_msgs::stateAcomms>::reverse_iterator iter;
    for (iter = stateAcomms_list.rbegin(); iter != stateAcomms_list.rend(); ++iter) {
      if((*iter).source_id == usbl.source_id && fabs((usbl.header.stamp-(*iter).header.stamp).toSec()) < p_t_sync)
        break;
    }
    // Return if no nearby pointer is found
    if(iter == stateAcomms_list.rend()){
      return;
    }
    
    farol_msgs::mState console_state;
    console_state.header.stamp = usbl.header.stamp;
    console_state.X = (*iter).position.east + cartesian[1];
    console_state.Y = (*iter).position.north + cartesian[0];
    pub_usbl_est_console.publish(console_state);

    // Publish final pose
    dsor_msgs::Measurement pose_fix;
    pose_fix.header.stamp = usbl.header.stamp;
    pose_fix.header.frame_id = name_vehicle_id_ + '_' + usbl.header.frame_id;
    // set position
    pose_fix.value.push_back((*iter).position.north + cartesian[0]);
    pose_fix.value.push_back((*iter).position.east + cartesian[1]);
    // set noise covariance
    pose_fix.noise.push_back((*iter).position_variance.north + usbl.position_covariance[0] + p_meas_noise);
    pose_fix.noise.push_back((*iter).position_variance.east + usbl.position_covariance[4] + p_meas_noise);
    pub_pose_fix.publish(pose_fix);

  }
  return; 
}

void UsblFix2Pos::stateCallback(const auv_msgs::NavigationStatus &msg){

  initialized = true;
  state_stamp = msg.header.stamp.toSec();

  state[0] = msg.position.north;
  state[1] = msg.position.east;
  state_var[0] = msg.position_variance.north;
  state_var[1] = msg.position_variance.east;

  //if (p_fix_type == false){
  state_lat_lon[0] = msg.global_position.latitude;
  state_lat_lon[1] = msg.global_position.longitude;
  //}
}

void UsblFix2Pos::stateAcommsCallback(const farol_msgs::stateAcomms &msg){

  initialized = true;
  state_stamp = msg.header.stamp.toSec();
  
  // Add fix to a list
  stateAcomms_list.push_back(msg);

}

/*
   @.@ Main
   */
int main(int argc, char **argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "acoustic_converters_node"); //node name
  // +.+ create a node handle; need to pass this to the class constructor
  ros::NodeHandle nh, nh_p("~");

  ROS_INFO("main: instantiating an object of type UsblFix2Pos");

  // +.+ instantiate an UsblFix2Pos class object and pass in pointer to nodehandle for constructor to use
  UsblFix2Pos usblfix2pos(&nh, &nh_p);

  // +.+ Added to work with timer -> going into spin; let the callbacks do all the work
  ros::spin();

  return 0;
}
