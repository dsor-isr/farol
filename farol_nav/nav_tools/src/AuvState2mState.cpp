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

	insidePressure = new LowPassFilter();
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
	sub_inside_pressure_.shutdown();

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
    sub_inside_pressure_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/inside_pressure", "inside_pressure/data"), 1, &AuvState2mState::insidePressureCallback, this);
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
	timer_in_pressure_ = nh_.createTimer(ros::Duration(0.1), &AuvState2mState::insidePressureTimer, this);
}

/*
@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
*/

void AuvState2mState::insidePressureTimer(const ros::TimerEvent& event){
	std::pair<double, double> ipressure = insidePressure->predict();
	in_press = ipressure.first;
  	in_press_dot = ipressure.second;
}

/*
@.@ Load the parameters
*/
void AuvState2mState::loadParams()
{
	ROS_INFO("No AuvState2mState parameters to load");
}

/*
 @.@ Callback gps -> convert lat lon to utm and publish in a state message
*/
void AuvState2mState::insidePressureCallback(const farol_msgs::Pressure &msg){
	// Hack due to the pressure cell repeating old values
	static double old_pressure = 0;
	if(msg.pressure != old_pressure){
		insidePressure->update(msg.pressure);
		old_pressure = msg.pressure;
	}
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
	mstate.In_Press = in_press;
	mstate.In_Press_dot = in_press_dot;

	// publish vehicle state
    mstate_pub_.publish(mstate);
}

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
