/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "Gnss2UtmOutlier"
#include "Gnss2UtmOutlier.h"

/*
@.@ CONSTRUCTOR
*/
Gnss2UtmOutlier::Gnss2UtmOutlier(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
  ROS_INFO("in class constructor of Gnss2UtmOutlier");
  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimers();
}

void Gnss2UtmOutlier::initializeTimers() {
    timer_gps_outlier_ = nh_.createTimer(ros::Duration(1.0), &Gnss2UtmOutlier::timerGPSOutlierCallback, this);
    timer_usbl_back_ = nh_.createTimer(ros::Duration(3.0), &Gnss2UtmOutlier::timerUSBLDelayCallback, this);
    timer_gps_outlier_.stop();
    timer_usbl_back_.stop();
}

/*
 @.@ Iteration via timer callback - Notice we have two timers here
*/
void Gnss2UtmOutlier::timerGPSOutlierCallback(const ros::TimerEvent &event){
  
  state_pub_.publish(utm_);

}

void Gnss2UtmOutlier::timerUSBLDelayCallback(const ros::TimerEvent &event){

  dsor_msgs::Measurement usbl;
  usbl.header.stamp = ros::Time::now() - ros::Duration(3.0);
  usbl.header.frame_id = "usbl";
  usbl.noise.emplace_back(1.0);
  usbl.noise.emplace_back(1.0);
  
  usbl.value.emplace_back(utm_.value[0] - 100);
  usbl.value.emplace_back(utm_.value[1] - 100);

  state_pub_.publish(usbl);

}

/*
@.@ Destructor
*/
Gnss2UtmOutlier::~Gnss2UtmOutlier()
{

  // +.+ shutdown publishers
  state_pub_.shutdown();

  // +.+ shutdown subscribers
  gnss_sub_.shutdown();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

/*
@.@ Member Helper function to set up subscribers;
*/
void Gnss2UtmOutlier::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers for Gnss2UtmOutlier");

  gnss_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/gnss", "/sensors/gnss"), 10, &Gnss2UtmOutlier::gnssBroadcasterCallback, this);
}

/*
@.@ Member helper function to set up publishers;
*/
void Gnss2UtmOutlier::initializePublishers()
{
  ROS_INFO("Initializing Publishers for Gnss2UtmOutlier"); 
  state_pub_ = nh_.advertise<dsor_msgs::Measurement>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/position", "/measurement/position"), 1, true);
}

void Gnss2UtmOutlier::initializeServices(){

  enable_gps_outlier_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, "services/enable_gps_outlier", "/sensor/fake/enable_gps_outlier"), &Gnss2UtmOutlier::enableGPSService, this);
  
  enable_usbl_delay_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, "services/enable_usbl_delay", "/sensor/fake/enable_usbl_delay"), &Gnss2UtmOutlier::enableUSBLService, this);
}

/*
@.@ Load the parameters
*/
void Gnss2UtmOutlier::loadParams()
{
  ROS_INFO("Load the Gnss2UtmOutlier parameters");
  // Program Parameters
  p_default_depth_ = FarolGimmicks::getParameters<double>(nh_private_, "default_depth", NAN);
}

/*
@.@ Callback gps -> convert lat lon to utm and publish in a state message
*/

void Gnss2UtmOutlier::gnssBroadcasterCallback(const sensor_msgs::NavSatFix &msg)
{
  // Check if navigation status is good, otherwise don't publish
  if (msg.status.status == -1)
    return;

  bool northp;
  int zone;
  double northing, easting, gamma, k;

  dsor_msgs::Measurement utm;
  // TODO: zone not being used rn, in future we should make sure that all inputs belong in the same zone
  // TODO: The northing y jumps by UTMUPS::UTMShift() when crossing the equator in the southerly direction. Sometimes it is useful to remove this discontinuity in y by extending the "northern" hemisphere using UTMUPS::Transfer:
  // Forward conversion from GPS to UTM
  try {
    GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude, zone, northp, easting, northing, gamma, k);
  }
  catch (const GeographicLib::GeographicErr::exception &ex)
  {
    ROS_WARN("Gnss2UtmOutlier caught exception: %s", ex.what());
    return;
  }

  utm.header.stamp = msg.header.stamp;
  utm.header.frame_id = msg.header.frame_id;

  utm.value.push_back(northing + 100);
  utm.value.push_back(easting + 100);

  utm.noise.push_back(msg.position_covariance[0]);
  utm.noise.push_back(msg.position_covariance[4]);

  // publish vehicle state
  utm_ = utm;
  //state_pub_.publish(utm);
}

bool Gnss2UtmOutlier::enableGPSService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

  if(req.data){
    timer_gps_outlier_.start();
    res.success = true;
    res.message = "Gps outliers activated";
  }
  else{
    timer_gps_outlier_.stop();
    res.success = false;
    res.message = "Gps outliers deactivated";
  }

  return true;
}

bool Gnss2UtmOutlier::enableUSBLService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

  if(req.data){
    timer_usbl_back_.start();
    res.success = true;
    res.message = "USBL delayed activated";
  }
  else{
    timer_usbl_back_.stop();
    res.success = false;
    res.message = "USBL delayed deactivated";
  }

  return true;
}


/*
@.@ Main
*/
int main(int argc, char **argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "utm_converter_outlier_node"); //node name
  // +.+ create a node handle; need to pass this to the class constructor
  ros::NodeHandle nh, nh_p("~");

  ROS_INFO("main: instantiating an object of type Gnss2UtmOutlier");

  // +.+ instantiate an Gnss2UtmOutlier class object and pass in pointer to nodehandle for constructor to use
  Gnss2UtmOutlier gnss2utm(&nh, &nh_p);

  // +.+ Added to work with timer -> going into spin; let the callbacks do all the work
  ros::spin();

  return 0;
}
