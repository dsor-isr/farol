//=================================================================================================
// Copyright (c) 2015, Farol Team, Instituto Superior Tecnico
// All rights reserved.
//=================================================================================================

#include <PingerNode.h>

  AcousticPinger::AcousticPinger(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private){

    // Parameters
    loadParams();

    // Create modems and timeouts queue
    for (int i=0; i<modems_list.size(); ++i){
      modems.push(atof(modems_list[i].c_str()));
      timeouts.push(timeout_list[i]);
    }
    // Subscribers
    initializeSubscribers();

    // Publishers
    initializePublishers();


    // Timer
    initializeTimer();


    // Start cycle
    tlastRECVIMS = 0;
    tlastSENDIMS = 0;
    lastRECVTime_modem = 0;
    lastRECVTime_ros = ros::Time(0);

    // update next maximum range because it depends on the timeout
    range_max = (timeouts.front() - tslack)/2.0*SOUND_SPEED;

    waiting_for_serializer = false;

    // it starts the modem in disable mode
    ENABLE_ = false;

  }
  
  AcousticPinger::~AcousticPinger() {

 	// +.+ shutdown publishers
	sub_enable.shutdown();
	sub_in_modem.shutdown();
	sub_serializer.shutdown();

 	// +.+ shutdown subscribers
	pub_im.shutdown();
	pub_range.shutdown();
	pub_triggerserialization.shutdown();
	pub_deserialization.shutdown();
 	
	 // +.+ stop timer
 	timer.stop();

 	// +.+ shutdown node
 	nh_.shutdown();
 }
  
   /*
 @.@ Member Helper function to set up subscribers;
 */
void AcousticPinger::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for PingerNode");

  sub_enable = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/enable", "enable"), 1, &AcousticPinger::EnableCallback, this);
  sub_in_modem = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/modem_recv", "modem/recv"), 1, &AcousticPinger::RECVIMSCallback, this);
  sub_serializer = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/payload", "payload_to_transmit"), 1, &AcousticPinger::serializerCallback, this);
}

 void AcousticPinger::initializePublishers() {
 	ROS_INFO("Initializing Publishers for AcousticPinger"); 

  pub_im = nh_.advertise<dmac::DMACPayload>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/modem_send", "modem/send"), 1);
  pub_range = nh_.advertise<farol_msgs::mUSBLFix>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/meas_usbl_fix", "measurement/usbl_fix"), 1);
  pub_triggerserialization = nh_.advertise<std_msgs::Empty>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/trigger_serialization", "trigger_serialization"), 1);
  pub_deserialization = nh_.advertise<std_msgs::String>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/deserialize", "payload_to_deserialize"), 1);
}  

  /*
 @.@ Load the parameters
 */
 void AcousticPinger::loadParams() {
 	ROS_INFO("Load the AcousticPinger parameters");

  modems_list = FarolGimmicks::getParameters<std::vector<std::string>>(nh_private_, "modems_list");
	timeout_list = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "timeout_list");
	tslack = FarolGimmicks::getParameters<double>(nh_private_,"tslack");
 }

 /*
 @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
*/
void AcousticPinger::initializeTimer() {
  timer = nh_.createTimer(ros::Duration(timeouts.front()), &AcousticPinger::Timer, this, true);
  timer.stop();
}

  void AcousticPinger::EnableCallback(const std_msgs::Bool& msg)
  {
    if(ENABLE_ == false && msg.data == true)
    {
      ENABLE_ = msg.data;
      pingNextNode();
    }
    ENABLE_ = msg.data;
  }

  void AcousticPinger::RECVIMSCallback(const dmac::DMACPayload& msg)
  {
    // Update Clock with the last known instant
    updateClock(msg.timestamp+msg.duration, msg.header.stamp);

    // Check if the ID corresponds to the modem we are expecting
    if(msg.source_address == modems.front() && msg.type == msg.DMAC_IMS)
    {
      // deserialize data
      std_msgs::String payload_data;
      payload_data.data = msg.payload + ":vehicle" + std::to_string(msg.source_address) ; 
      pub_deserialization.publish(payload_data);

      // compute range
      tlastRECVIMS = msg.timestamp;
      double range = ((tlastRECVIMS-tlastSENDIMS)-tslack*1000000)/2000000.0*SOUND_SPEED;

      // Discard very strange measurements
      if(fabs(range)<range_max)
      {
        // Building the message
        farol_msgs::mUSBLFix range_msg;
        range_msg.type=range_msg.RANGE_ONLY;
        range_msg.header.stamp=msg.header.stamp
                              -ros::Duration(fabs(range)/SOUND_SPEED+tslack/2.0); // time stamp of half-way
        range_msg.header.frame_id = "usbl";
        range_msg.range = range;
        range_msg.source_id = msg.source_address;
        range_msg.source_name = msg.source_name;
        pub_range.publish(range_msg);
        ROS_WARN("Range to node %d is %.3f of max value %.3f with frame %s", msg.source_address, range, range_max, msg.header.frame_id.c_str());
      }
      // Schedule next ping
      pingNextNode();
    }
  }

  // TODO: Ask for Clock to the Modem
  void AcousticPinger::updateClock(unsigned long int tmodem, ros::Time tros){
    unsigned long int old_clock=getModemClockNow();
    lastRECVTime_modem = tmodem;
    lastRECVTime_ros = tros;
  }

  // Get the last modem estimate
  unsigned long int AcousticPinger::getModemClockNow(){
    ros::Time tnow = ros::Time::now();
    return lastRECVTime_modem + (tnow-lastRECVTime_ros).toSec()*1000000+100000; // 100ms diff detected between estimated and real (maybe processing and receiving messages)
  }

void AcousticPinger::triggerSerialization()
  {
    std_msgs::Empty aux;
    pub_triggerserialization.publish(aux);
    waiting_for_serializer = true;
  }

  void AcousticPinger::serializerCallback(const std_msgs::String& msg)
  {
    if(!waiting_for_serializer)
    {
      ROS_ERROR("Received payload to transmit, in the wrong cycle");
      return;
    }
    waiting_for_serializer = false;

    // Send Message to Modem after a slack time
    unsigned long int tping = tlastRECVIMS + round(tslack*1000000);

    // Check if we can continue after a timeout
    if(tlastRECVIMS==0 && lastRECVTime_modem!=0 && (ros::Time::now()-lastRECVTime_ros).toSec()<50.0 )
    {
      tping=getModemClockNow() + round(tslack*1000000);
      // Transmit immediately
    }
    else if(tlastRECVIMS==0)
      tping=0;

    dmac::DMACPayload im;
    im.header.stamp = ros::Time::now();
    
    im.destination_address = modems.front();
    im.type = im.DMAC_IMS;
    im.timestamp = tping;
    if(tping==0)
      im.timestamp_undefined =true;
    else
      im.timestamp_undefined = false;
    im.ack  = false;
    im.payload =  msg.data;
    pub_im.publish(im);

    // Storing times
    tlastRECVIMS = 0;
    tlastSENDIMS = tping;

  }

  void AcousticPinger::pingNextNode()
  {
    ROS_INFO_STREAM("Pinger node started");
    if(!ENABLE_)
      return;

    // Send first element to the end of the queue
    modems.push(modems.front());
    modems.pop();
    timeouts.push(timeouts.front());
    timeouts.pop();

    // request the serializer to serialize the data to be sent
    triggerSerialization();
    ROS_INFO_STREAM("Trigger OK");

    // Send Message to Modem after a slack time
    unsigned long int tping = tlastRECVIMS + round(tslack*1000000);

    // update next maximum range because it depends on the timeout
    range_max = (timeouts.front() - tslack)/2.0*SOUND_SPEED;
    ROS_INFO("Timeout front: %.3f", timeouts.front());

    // Set the next timeout
    // include gap to next ping on the timeout
    double time_gap_to_ping = (tping-getModemClockNow())/1000000.0;
    ROS_INFO("Time gap to ping: %.3f", time_gap_to_ping);
    if(time_gap_to_ping>0 && time_gap_to_ping<2.0){
      timer.setPeriod(ros::Duration(timeouts.front()+time_gap_to_ping),true);
    }else{
      timer.setPeriod(ros::Duration(timeouts.front()),true);
    }

    timer.stop();
    timer.start();
  }


  void AcousticPinger::Timer(const ros::TimerEvent& e){
    ROS_INFO("OK");
    if(waiting_for_serializer)
      ROS_WARN("No reply from serializer in %.3fs", timeouts.front());
    else
      ROS_WARN("No reply from destination modem [%d] in %.3fs", modems.front(), timeouts.front());
    waiting_for_serializer = false;
    tlastRECVIMS = 0;
    pingNextNode();

  }

/*
 * MAIN
 */
int main(int argc, char **argv){
  ros::init(argc, argv, "pinger_acomms");
  ros::NodeHandle nh, nh_private("~");

  AcousticPinger pinger(&nh, &nh_private);

  pinger.pingNextNode();
  ros::spin();

  return 0;
}