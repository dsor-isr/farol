/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "Gnss2Utm"
#include "AuvState2mState.h"

/*
@.@ CONSTRUCTOR
*/
AuvState2mState::AuvState2mState(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
	ROS_INFO("in class constructor of AuvState2mState");
	loadParams();
	initializeSubscribers();
	initializePublishers();
	initializeTimers();

}

/*
@.@ Destructor
*/
AuvState2mState::~AuvState2mState()
{
	// +.+ shutdown publishers
	mstate_pub_.shutdown();

	// +.+ shutdown subscribers
	sub_auv_state_.shutdown();
	sub_inside_pressure_filtered_.shutdown();
  sub_inside_pressure_rate_.shutdown();

	// +.+ shutdown node
	nh_.shutdown();
	nh_private_.shutdown();
}

/*
@.@ Member Helper function to set up subscribers;
*/
void AuvState2mState::initializeSubscribers()
{
	ROS_INFO("Initializing Subscribers for AuvState2mState");
    sub_auv_state_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state", "state"), 1, &AuvState2mState::mStateBroadcasterCallback, this);
    sub_inside_pressure_filtered_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/inside_pressure_filter", "inside_pressure_filter/data"), 1, &AuvState2mState::insidePressureFilteredCallback, this);
    sub_inside_pressure_rate_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/inside_pressure_rate", "inside_pressure_rate/data_dot"), 1, &AuvState2mState::insidePressureRateCallback, this);
	sub_gps_status_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/gnss", "sensors/gnss"), 0, &AuvState2mState::mGPSStatusCallback, this);
}

/*
@.@ Member helper function to set up publishers;
*/
void AuvState2mState::initializePublishers()
{
	ROS_INFO("Initializing Publishers for AuvState2mState"); 
    mstate_pub_ = nh_.advertise<farol_msgs::mState>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/console_state", "State"), 1, true);
}

void AuvState2mState::initializeTimers()
{

}

/*
@.@ Load the parameters
*/
void AuvState2mState::loadParams()
{
	ROS_INFO("No AuvState2mState parameters to load");
}

/*
 @.@ Callback inside_pressure from a low pass filter 
 */
void AuvState2mState::insidePressureFilteredCallback(const std_msgs::Float32 &msg){
  in_press_ = msg.data;
}

/*
 * @.@ Callback inside_pressure rate from a low pass filter
 */
void AuvState2mState::insidePressureRateCallback(const std_msgs::Float32 &msg){
  in_press_dot_ = msg.data;
}

void AuvState2mState::mStateBroadcasterCallback(const auv_msgs::NavigationStatus &msg)
{
	farol_msgs::mState mstate;
	// Set Header Information
	mstate.status = farol_msgs::mState::STATUS_ALL_OK;
	mstate.GPS_Good = gps_status_;
	mstate.IMU_Good = 1;
	mstate.header = msg.header;

	// Set Position
	mstate.X = msg.position.east;
	mstate.Y = msg.position.north;
	mstate.Z = msg.position.depth;

	mstate.Depth = msg.position.depth;
	mstate.altitude = msg.altitude;

	// Set Orientation
	mstate.Roll = msg.orientation.x;
	mstate.Pitch = msg.orientation.y;
	mstate.Yaw = msg.orientation.z;

	// Set Linear Velocity
	mstate.Vx = msg.seafloor_velocity.x;
	mstate.Vy = msg.seafloor_velocity.y;
	mstate.Vz = msg.seafloor_velocity.z;

	// Set Body Velocity
	mstate.u = msg.body_velocity.x;

	// Set Angular Rate
	mstate.Roll_rate = msg.orientation_rate.x;
	mstate.Pitch_rate = msg.orientation_rate.y;
	mstate.Yaw_rate = msg.orientation_rate.z;

	// Set Inside Pressure
	mstate.In_Press = in_press_;
	mstate.In_Press_dot = in_press_dot_;

	// publish vehicle state
    mstate_pub_.publish(mstate);
}

/*
 * @.@ Callback to update GPS status 
 */
void AuvState2mState::mGPSStatusCallback(const sensor_msgs::NavSatFix &msg){
	gps_status_ = msg.status.status;
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

	ROS_INFO("main: instantiating an object of type AuvState2mState");

	// +.+ instantiate an AuvState2mState class object and pass in pointer to nodehandle for constructor to use
	AuvState2mState auvState2mState(&nh, &nh_p);

	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	ros::spin();

	return 0;
}
