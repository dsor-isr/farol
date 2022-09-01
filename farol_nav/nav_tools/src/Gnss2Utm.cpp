/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "Gnss2Utm"
#include "Gnss2Utm.h"

/*
@.@ CONSTRUCTOR
*/
Gnss2Utm::Gnss2Utm(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
  ROS_INFO("in class constructor of Gnss2Utm");
  loadParams();
  initializeTimers();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
}

/*
@.@ Destructor
*/
Gnss2Utm::~Gnss2Utm()
{

  // +.+ shutdown publishers
  gnss_position_pub_.shutdown();
	state_gt_pub_.shutdown();

  // +.+ shutdown subscribers
  gnss_sub_.shutdown();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

void Gnss2Utm::initializeTimers() {
    timer_gps_ = nh_.createTimer(ros::Duration(0.1), &Gnss2Utm::timerGPSCallback, this);
		timer_gps_gt_ = nh_.createTimer(ros::Duration(0.1), &Gnss2Utm::timerGPSGtCallback, this);
}

/*
@.@ Member Helper function to set up subscribers;
*/
void Gnss2Utm::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers for Gnss2Utm");
  gnss_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/gnss", "gnss"), 1, &Gnss2Utm::gnssBroadcasterCallback, this);
}

/*
@.@ Member helper function to set up publishers;
*/
void Gnss2Utm::initializePublishers()
{
  ROS_INFO("Initializing Publishers for Gnss2Utm"); 
  gnss_position_pub_ = nh_.advertise<dsor_msgs::Measurement>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/position", "position"), 1, true);
	state_gt_pub_ = nh_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/ground_truth", "/State_gt"), 1);
}
void Gnss2Utm::initializeServices(){
  enable_gps_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, "services/enable_gps", "enable_gps"), &Gnss2Utm::enableGPSService, this);
}

/*
@.@ Load the parameters
*/
void Gnss2Utm::loadParams()
{
  ROS_INFO("Load the Gnss2Utm parameters");
  // Program Parameters
  p_default_depth_ = FarolGimmicks::getParameters<double>(nh_private_, "default_depth", NAN);
}

/*
@.@ Callback gps -> convert lat lon to utm and publish in a state message
*/

void Gnss2Utm::gnssBroadcasterCallback(const sensor_msgs::NavSatFix &msg)
{

  // Check if navigation status is good, otherwise don't publish
  if (msg.status.status == -1){
    gps_good_ = false;
    return;
  } 

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
    ROS_WARN("Gnss2Utm caught exception: %s", ex.what());
    return;
  }

  utm.header.stamp = msg.header.stamp;
  utm.header.frame_id = msg.header.frame_id;

  utm.value.push_back(northing);
  utm.value.push_back(easting);

  utm.noise.push_back(msg.position_covariance[0]);
  utm.noise.push_back(msg.position_covariance[4]);

  // publish vehicle state
  utm_ = utm;
  //state_pub_.publish(utm);
  
  gps_good_ = true;
}

void Gnss2Utm::timerGPSCallback(const ros::TimerEvent &event){
  if(gps_good_ && !utm_.value.empty()){
    gnss_position_pub_.publish(utm_);
    utm_.value.clear();
  }
}

void Gnss2Utm::timerGPSGtCallback(const ros::TimerEvent &event){

	if (utm_.value.empty())
		return;

 	state_gt_.header.stamp = ros::Time::now();
	// Set Position
  state_gt_.X = utm_.value[1];
  state_gt_.Y = utm_.value[0];
	
	state_gt_pub_.publish(state_gt_);
}

bool Gnss2Utm::enableGPSService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

  if(req.data){
    timer_gps_.start();
    res.success = true;
    res.message = "Gps activated";
  }
  else{
    timer_gps_.stop();
    res.success = false;
    res.message = "Gps deactivated";
  }

  return true;
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

  ROS_INFO("main: instantiating an object of type Gnss2Utm");

  // +.+ instantiate an Gnss2Utm class object and pass in pointer to nodehandle for constructor to use
  Gnss2Utm gnss2utm(&nh, &nh_p);

  // +.+ Added to work with timer -> going into spin; let the callbacks do all the work
  ros::spin();

  return 0;
}
