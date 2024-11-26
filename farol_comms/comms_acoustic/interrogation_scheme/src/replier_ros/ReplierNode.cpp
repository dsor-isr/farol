/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
 // this header incorporates all the necessary #include files and defines the class "ReplierNode"
 #include "ReplierNode.h"

 /*
 @.@ CONSTRUCTOR:
 */
 ReplierNode::ReplierNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private)
 {
 	ROS_INFO("in class constructor of ReplierNode");
 	loadParams();

	 // Create timeouts queue
    for (int i=0; i<timeout_list.size(); ++i){
      timeouts.push(timeout_list[i]);
    }

	initializeSubscribers();
 	initializePublishers();
 
	srand(static_cast <unsigned int> (time(0))); 
 	initializeTimer();

 }

 /*
 @.@ Destructor
 */
 ReplierNode::~ReplierNode() {

 	// +.+ shutdown publishers
 	range_pub_.shutdown();
	im_pub_.shutdown();
	trigger_serialization_pub_.shutdown();
	deserialization_pub_.shutdown();


 	// +.+ shutdown subscribers
 	ack_sub_.shutdown();
	serializer_sub_.shutdown();

 	// +.+ stop timer
 	timer_.stop();

 	// +.+ shutdown node
 	nh_.shutdown();
 }

 /*
 @.@ Member Helper function to set up subscribers;
 */
 void ReplierNode::initializeSubscribers() {
 	ROS_INFO("Initializing Subscribers for ReplierNode");

	ack_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/modem_recv", "modem/recv"), 1, &ReplierNode::replyCallback, this);
  	serializer_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/payload", "payload_to_transmit"), 1, &ReplierNode::serializerCallback, this);
}

 /*
 @.@ Member helper function to set up publishers;
 */
 void ReplierNode::initializePublishers() {
 	ROS_INFO("Initializing Publishers for ReplierNode"); 	// ---> add publishers here
	range_pub_ = nh_.advertise<farol_msgs::mUSBLFix>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/meas_usbl_fix", "measurement/usbl_fix"), 1);
	im_pub_ = nh_.advertise<dmac::DMACPayload>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/modem_send", "modem/send"), 1);
  	trigger_serialization_pub_ = nh_.advertise<std_msgs::Empty>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/trigger_serialization", "trigger_serialization"), 1);
  	deserialization_pub_ = nh_.advertise<std_msgs::String>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/deserialize", "payload_to_deserialize"), 1);
 }

 /*
 @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
 */
 void ReplierNode::initializeTimer() {
 	timer_ =nh_.createTimer(ros::Duration(p_tslack_/2.0), &ReplierNode::timerIterCallback, this);
 	timer_.stop();
 }

 /*
 @.@ Load the parameters
 */
 void ReplierNode::loadParams() {
 	ROS_INFO("Load the ReplierNode parameters");

	p_modem_id_ = FarolGimmicks::getParameters<int>(nh_private_, "modem_id");
	p_tslack_ = FarolGimmicks::getParameters<double>(nh_private_,"tslack");
	timeout_list = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "timeout_list");

}

 /*
 @.@ 
 */

void ReplierNode::triggerSerialization(){
	std_msgs::Empty aux;
	trigger_serialization_pub_.publish(aux);
}

 /*
 @.@ Callbacks Section / Methods
 */

 /*
 @.@ Iteration via timer callback
 */
 void ReplierNode::timerIterCallback(const ros::TimerEvent &event) {

 	// @.@ Do your algorithm part and publish things here, like while loop with ros::Rate rate and rate.sleep(duration)
 	// in classical ros
	ROS_ERROR("compressing timeout replying anyway");
	reply_ims_.payload = "N";
	modem_id_last_send_ims_ = 0;
	pinger_ack = false;
	
 }
 /*
 @.@ Callback for serializing new message to be transmitted via acoustic modem -> sends the message to acoustic modem 
	 where dmac will do its magic.
 */
 void ReplierNode::serializerCallback(const std_msgs::String &msg){

	if ((ros::Time::now() - reply_ims_.header.stamp).toSec() > p_tslack_){
		ROS_ERROR("Serializer didn't reply in useful time. Missing reply ping");
		return;
	}

	// +.+ Stop actual timer
	timer_.stop();

	reply_ims_.header.stamp = ros::Time::now();
	reply_ims_.payload = msg.data;

	im_pub_.publish(reply_ims_);
}

void ReplierNode::replyCallback(const dmac::DMACPayload &msg){

	// +.+ checks if the dmac message has the modem address we are expecting
	if (msg.source_address == modem_id_last_send_ims_){
		range_ = ((msg.timestamp - t_last_send_ims_) - p_tslack_ * SLACK_TIME) / (2 * SLACK_TIME) * SOUND_SPEED;

		timeouts.push(timeouts.front());
    	timeouts.pop();
	

		// +.+ Discard very strange measurements

		// update next maximum range because it depends on the timeout

    	range_max = (timeouts.front() - p_tslack_)/2.0*SOUND_SPEED;	

		if (fabs(range_) < range_max){
			// +.+ Declaration of dmac::mUSBLFix message
			farol_msgs::mUSBLFix range_msg;

			// +.+ Specify that this is a range only message
			range_msg.type = range_msg.RANGE_ONLY;
			 
			// +.+ time stamp of half-way, update timestamp 
			range_msg.header.stamp = msg.header.stamp - ros::Duration(fabs(range_) / SOUND_SPEED + p_tslack_ / 2.0);
			 
			// +.+ range value to be published
			range_msg.range = range_;

			// +.+ address of the modem that sent the message
			range_msg.source_id = msg.source_address;

			// +.+ name of the modem that sent the message
			range_msg.source_name = msg.source_name;

			// Acknowledge received from pinger
			pinger_ack = true;

			// +.+ publish dmac::mUSBLFix message, range only measurement in this case
			range_pub_.publish(range_msg);
        	// ROS_WARN("From modem %d to %d the range is %.1fm",p_modem_id_, msg.source_address, range_);

			reply_ims_.header.stamp = ros::Time(0);
		}
	}

	if (msg.destination_address == p_modem_id_){
		// deserialize data
      	std_msgs::String payload_data;
        payload_data.data = msg.payload + ":vehicle" + std::to_string(msg.source_address) ; 
      	deserialization_pub_.publish(payload_data);

      	// populate message
     	reply_ims_.header.stamp = ros::Time::now();
     	reply_ims_.type = reply_ims_.DMAC_IMS;
     	reply_ims_.ack = pinger_ack;
      	reply_ims_.destination_address = msg.source_address;
      	reply_ims_.timestamp = msg.timestamp + round(p_tslack_ * SLACK_TIME);
      	reply_ims_.timestamp_undefined =false;
      	

      	// Save times and address for computing range
      	t_last_send_ims_ = reply_ims_.timestamp;
      	modem_id_last_send_ims_ = msg.source_address;

      	timer_.stop();
      	timer_.start();

      	triggerSerialization();
    }
    else{
      	reply_ims_.header.stamp = ros::Time(0);
      	t_last_send_ims_ = 0;
      	modem_id_last_send_ims_ = 0;
    }
}


 /*
 @.@ Main
 */
 int main(int argc, char** argv)
 {
 	// +.+ ROS set-ups:
 	ros::init(argc, argv, "atomic_clocks_acoustic_replier_node"); //node name
 	// +.+ create a node handle; need to pass this to the class constructor
 	ros::NodeHandle nh, nh_p("~");

 	ROS_INFO("main: instantiating an object of type ReplierNode");

 	// +.+ instantiate an ReplierNode class object and pass in pointer to nodehandle for constructor to use
 	ReplierNode atomicClocksAcousticReplier(&nh, &nh_p);

 	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
 	ros::spin();

 	return 0;
 }