/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "Gnss2State"
#include "Gnss2State.h"

/*
#######################################################################################################################
@.@ CONSTRUCTOR: put all dirty work of initializations here
Note the odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
#######################################################################################################################
*/
Gnss2State::Gnss2State(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
	ROS_INFO("in class constructor of Gnss2State");
	loadParams();
	initializeSubscribers();
	initializePublishers();
}

/*
#######################################################################################################################
@.@ Destructor
#######################################################################################################################
*/
Gnss2State::~Gnss2State()
{

	// +.+ shutdown publishers
	pub_state.shutdown();
    pub_zone.shutdown();

	// +.+ shutdown subscribers
	sub_gnss.shutdown();

	// +.+ shutdown node
	nh_.shutdown();
	nh_private_.shutdown();
}

/*
#######################################################################################################################
@.@ Member Helper function to set up subscribers;
note odd syntax: &Gnss2State::subscriberCallback is a pointer to a member function of Gnss2State
"this" keyword is required, to refer to the current instance of Gnss2State
#######################################################################################################################
*/
void Gnss2State::initializeSubscribers()
{
	ROS_INFO("Initializing Subscribers for Gnss2State");
  sub_gnss = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/gnss", "/acomms/nav/filter/state"), 10, &Gnss2State::gnssBroadcasterCallback, this);
}

/*
#######################################################################################################################
@.@ Member helper function to set up publishers;
#######################################################################################################################
*/
void Gnss2State::initializePublishers()
{
	ROS_INFO("Initializing Publishers for Gnss2State"); // ---> add publishers here
  pub_state = nh_private_.advertise<auv_msgs::NavigationStatus>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/state", "/acomms/nav/filer/state"), 1, true);
  pub_zone = nh_private_.advertise<std_msgs::String>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/utmzone", "/state/utmzone"), 1, true);
}

/*
#######################################################################################################################
@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
#######################################################################################################################
*/

/*
#######################################################################################################################
@.@ Load the parameters
#######################################################################################################################
*/
void Gnss2State::loadParams()
{
	ROS_INFO("Load the Gnss2State parameters");
    // Program Parameters
	p_default_depth  = FarolGimmicks::getParameters<double>(nh_private_, "default_depth", NAN);
}

/*
#######################################################################################################################
 @.@ Callbacks Section / Methods
#######################################################################################################################
*/

/*
#######################################################################################################################
 @.@ Callback gps -> convert lat lon to utm and publish in a state message
#######################################################################################################################
*/

void Gnss2State::gnssBroadcasterCallback(const auv_msgs::NavigationStatus &msg)
{
	auv_msgs::NavigationStatus state_fix;
	state_fix = msg;

    bool northp;
    int zone;
    double northing, easting, gamma, k;
    
    // TODO: zone not being used rn, in future we should make sure that all inputs belong in the same zone
    // TODO: The northing y jumps by UTMUPS::UTMShift() when crossing the equator in the southerly direction. Sometimes it is useful to remove this discontinuity in y by extending the "northern" hemisphere using UTMUPS::Transfer:
    // Forward conversion from GPS to UTM
    try {
        GeographicLib::UTMUPS::Forward(msg.global_position.latitude, msg.global_position.longitude, zone, northp, easting, northing, gamma, k);
    }
    catch (const GeographicLib::GeographicErr::exception &ex)
    {
        ROS_WARN("Gnss2State caught exception: %s", ex.what());
        return;
    }

    state_fix.header = msg.header;

    state_fix.position.north = northing;
    state_fix.position.east  = easting;
    state_fix.orientation.z = msg.orientation.z;
    state_fix.position.depth = msg.position.depth;
	// state_fix.position.depth = p_default_depth;
	// If merging, fill in other fields too
	// state_fix.orientation 		= state_merge.orientation;
	// state_fix.body_velocity 	= state_merge.body_velocity;
	// state_fix.seafloor_velocity = state_merge.seafloor_velocity;
	// publish vehicle state
    pub_state.publish(state_fix);

	// publish utm zone
    std_msgs::String msg_zone;
	msg_zone.data = GeographicLib::UTMUPS::EncodeZone(zone, northp);
    pub_zone.publish(msg_zone);
}

/*
#######################################################################################################################
 @.@ Main
 #######################################################################################################################
 */
int main(int argc, char **argv)
{
	// +.+ ROS set-ups:
	ros::init(argc, argv, "acoustic_converters_node"); //node name
	// +.+ create a node handle; need to pass this to the class constructor
	ros::NodeHandle nh, nh_p("~");

	ROS_INFO("main: instantiating an object of type Gnss2State");

	// +.+ instantiate an Gnss2State class object and pass in pointer to nodehandle for constructor to use
	Gnss2State Gnss2State(&nh, &nh_p);

	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	ros::spin();
	return 0;
}
