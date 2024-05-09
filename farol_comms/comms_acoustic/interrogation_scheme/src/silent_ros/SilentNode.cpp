//=================================================================================================
// Copyright (c) 2015, Farol Team, Instituto Superior Tecnico
// All rights reserved.
//=================================================================================================

#include <SilentNode.h>

SilentPinger::SilentPinger(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private){
  // Parameters
  loadParams();

  // Create modems and slots queue
  for (int i=0; i<modems_list.size(); ++i){
    modems.push(atof(modems_list[i].c_str()));
    slots.push(slot_list[i]);
  }
  // Organize modem and slot queue to start with anchor
  while(modems.front() != anchor){
    modems.push(modems.front());
    modems.pop();
    slots.push(slots.front());
    slots.pop();
  }

  // Subscribers
  initializeSubscribers();

  // Publishers
  initializePublishers();


  // Timer
  initializeTimer();

  // waiting for non-zero time now values (else it won't be possible to publish in the constructor)
  ros::Duration(5.0).sleep();

  // trigger ECLK message for computation of time offset between Modem's System Clock and UTC Time
  triggerECLKmessage();

  // Start cycle
  lastRECVTime_modem = 0;
  lastRECVTime_ros = ros::Time(0);

  // update next maximum range because it depends on the timeout
  range_max = (slots.front() - tslack)/2.0*SOUND_SPEED;

  waiting_for_serializer = false;
  init = true;
  ok_check = 0;
  
  if(modem_id == anchor) first_time = true;
  else first_time = false;

  // it starts the modem in disable mode
  ENABLE_ = false;
}


SilentPinger::~SilentPinger(){
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
 * @.@ Member Helper function to set up subscribers;
 */
void SilentPinger::initializeSubscribers(){
  ROS_INFO("Initializing Subscribers for SilentPinger");

  sub_enable = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/enable", "enable"), 1, &SilentPinger::EnableCallback, this);
  sub_in_modem = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/modem_recv", "modem/recv"), 1, &SilentPinger::RECVIMSCallback, this);
  sub_serializer = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/payload", "payload_to_transmit"), 1, &SilentPinger::serializerCallback, this);
  if(modem_id != anchor) sub_tinit = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/tinit", "tinit_sub"), 1, &SilentPinger::timeFromAnchorCallback, this);
  else sub_tinit = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/send_time", "send_time"), 1, &SilentPinger::startCallback, this);
  sub_modem_clock = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/modem_clock", "clock"), 1, &SilentPinger::modemClockCallback, this);
}


void SilentPinger::initializePublishers(){
 	ROS_INFO("Initializing Publishers for SilentPinger");

  pub_im = nh_.advertise<dmac::DMACPayload>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/modem_send", "modem/send"), 1);
  pub_range = nh_.advertise<farol_msgs::mUSBLFix>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/meas_usbl_fix", "measurement/usbl_fix"), 1);
  pub_triggerserialization = nh_.advertise<std_msgs::Empty>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/trigger_serialization", "trigger_serialization"), 1);
  pub_deserialization = nh_.advertise<std_msgs::String>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/deserialize", "payload_to_deserialize"), 1);
  if(modem_id == anchor) pub_tinit = nh_.advertise<std_msgs::UInt32>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/tinit", "tinit_pub"), 1);
}  


/*
 * @.@ Load the parameters
 */
void SilentPinger::loadParams(){
 	ROS_INFO("Load the SilentPinger parameters");

  modems_list = FarolGimmicks::getParameters<std::vector<std::string>>(nh_private_, "modems_list");
	slot_list = FarolGimmicks::getParameters<std::vector<double>>(nh_private_, "slot_list");
	tslack = FarolGimmicks::getParameters<double>(nh_private_, "tslack");
  modem_id = FarolGimmicks::getParameters<int>(nh_private_, "modem_id");
  anchor = FarolGimmicks::getParameters<int>(nh_private_, "anchor");
  tnew_start = FarolGimmicks::getParameters<int>(nh_private_, "tnew_start");
}


/*
 * @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
 */
void SilentPinger::initializeTimer(){
  timer = nh_.createTimer(ros::Duration(slots.front()), &SilentPinger::Timer, this, true);
  timer.stop();

  timer_sys_clock_overflow = nh_.createTimer(ros::Duration(0.5), &SilentPinger::SysClockOverflowHandler, this, true);
}

void SilentPinger::triggerECLKmessage() {
  dmac::DMACPayload im;
  im.header.stamp = ros::Time::now(); // message time stamp
  im.destination_address = 255;   // Broadcast address
  im.type = im.DMAC_ECLK; // media access type: BURST/IM/IMS/PBM/ECLK

  // Force undefined time since the message should be sent to the modem asap
  im.timestamp_undefined = true;
  
  // message to ask for External Clock from modem
  im.payload = "AT?ECLK";
  pub_im.publish(im);
  ROS_INFO("Message was sent!");
}

void SilentPinger::EnableCallback(const std_msgs::Bool& msg){
  if(ENABLE_ == false && msg.data == true)
  {
    ENABLE_ = msg.data;
    ROS_INFO("Enabled silent acoustic scheme");
  }
  ENABLE_ = msg.data;

  // Reinitialzie modes
  ok_check = 0;
  timer.stop();
  init = true;

  // Organize modem and slot queue to start with anchor
  while(modems.front() != anchor){
    modems.push(modems.front());
    modems.pop();
    slots.push(slots.front());
    slots.pop();
  }

  // IDEA TO MAKE TINIT ZERO
  // tinit = 0;
}

void SilentPinger::modemClockCallback(const dmac::DMACClock& msg) {
  // compute offset between physical clock and utc time
  computeModemTimeOffset(msg.phy_clock, msg.utc_time.sec, msg.utc_time.nsec);

  // compute when modem system clock will overflow
  computeModemSysClockOverflowTime(msg.phy_clock, modem_time_offset);
}

void SilentPinger::computeModemTimeOffset(unsigned int system_clock, unsigned int utc_time_sec, unsigned int utc_time_nsec) {
  // offset in microseconds (UTC Time - Modem System Clock)
  modem_time_offset = (boost::lexical_cast<unsigned long int>(utc_time_sec) * 1000000 + 
                       boost::lexical_cast<unsigned long int>(utc_time_nsec) / 1000) 
                       - boost::lexical_cast<unsigned long int>(system_clock);
  ROS_INFO("Modem Offset (UTC - SysClock): %lu microseconds.", modem_time_offset);
}

void SilentPinger::computeModemSysClockOverflowTime(unsigned int system_clock, unsigned long int modem_time_offset) {
  // compute how many microseconds until overflow
  unsigned int time_until_overflow = UNSIGNED_INT_MAX - system_clock;

  // utc time corresponding to sys clock overflow instant (seconds)
  sys_clock_overflow_instant = (boost::lexical_cast<double>(boost::lexical_cast<unsigned long int>(system_clock) 
                                                            + modem_time_offset
                                                            + boost::lexical_cast<unsigned long int>(time_until_overflow)))/1000000;

  ROS_INFO("Next Sys Clock Overflow: %f seconds.", sys_clock_overflow_instant);
}

void SilentPinger::SysClockOverflowHandler(const ros::TimerEvent& e) {
  // system clock overflow instant hasn't been set yet
  if (sys_clock_overflow_instant == -1) {
    return;
  }

  // if the overflow instant has passed, we need to update the modem_time_offset
  if (ros::Time::now().toSec() > sys_clock_overflow_instant) {
    modem_time_offset += UNSIGNED_INT_MAX;

    ROS_INFO("NEW: Modem Offset (UTC - SysClock): %lu microseconds.", modem_time_offset);

    // the new overflow instant should be calculated as well
    sys_clock_overflow_instant += boost::lexical_cast<double>(UNSIGNED_INT_MAX/1000000);
    
    ROS_INFO("NEW: Next Sys Clock Overflow: %f seconds.", sys_clock_overflow_instant);

    // now that the modem_time_offset has been updated as soon as possible, we should
    // properly update it using the ECLK message from the modem (which might be slower
    // due to communication issues), but the new values for the offset and overflow instant
    // will be 100% correct
    triggerECLKmessage();
  
  }
}

void SilentPinger::startCallback(const interrogation_scheme::StartSilent& msg){
  if(!ENABLE_){
    ROS_ERROR("Tried to start interrogation scheme without enabling first. Enable, then retry");
    return;
  }else{
    ROS_INFO("Sending init time to other vehicles...");

    // Transform to seconds
    tinit = msg.hours * 3600 + msg.minutes * 60 + msg.seconds + ros::Time::now().toSec();

    // Publish time to be read by Data Serializer
    std_msgs::UInt32 tmsg;
    tmsg.data = tinit;
    pub_tinit.publish(tmsg);

    ROS_INFO("Set time %lu", (unsigned long)(tmsg.data));

    double time_before_start = getTimeBeforeStart();

    // Start init mode
    init = true; // redundancy
    ok_check = 0;

    // Organize modem and slot queue to start with anchor
    while(modems.front() != anchor){
      modems.push(modems.front());
      modems.pop();
      slots.push(slots.front());
      slots.pop();
    }

    pingNextNode(tinit - time_before_start);  // Ping all vehicles to start at certain time
  }
}


void SilentPinger::timeFromAnchorCallback(const std_msgs::UInt32& msg){
  // Receive time from anchor
  if(msg.data == 0){
    init = false;   // End init mode for non-anchors
  }else{
    init = true;  // Redundancy
    tinit = msg.data; // Give starting time for non-anchors

    // Organize modem and slot queue to start with anchor
    while(modems.front() != anchor){
      modems.push(modems.front());
      modems.pop();
      slots.push(slots.front());
      slots.pop();
    }

    double time_before_start = getTimeBeforeStart();

    ROS_INFO("Time From all slots combined %3f, tinit = %lu, now = %3f", time_before_start, tinit, ros::Time::now().toSec());
    ROS_INFO("Conta = %3f", tinit - ros::Time::now().toSec() - time_before_start);
    tslot_end = tinit - time_before_start;   // Set end time to pick up start for non-anchor
    timer.stop();
    timer.setPeriod(ros::Duration(tinit - ros::Time::now().toSec() - time_before_start), true);
    timer.start();
  }
}


double SilentPinger::getTimeBeforeStart(){
  // Find right time to wake up modems
  int front_modem = modems.front();

  // Organize modem and slot queue to start with anchor
  while(modems.front() != anchor){
    modems.push(modems.front());
    modems.pop();
    slots.push(slots.front());
    slots.pop();
  }
  double time_before_start = 0.0;
  
  do{
    // Find next modem and add slot time until reaching anchor again
    modems.push(modems.front());
    modems.pop();
    slots.push(slots.front());
    slots.pop();
    time_before_start += slots.front();
  }while(modems.front() != anchor);

  time_before_start -= slots.front();

  // Organize modem and slot queue to start where we left
  while(modems.front() != front_modem){
    modems.push(modems.front());
    modems.pop();
    slots.push(slots.front());
    slots.pop();
  }

  ROS_INFO("Time before start = %3f", time_before_start);
  
  return time_before_start;
}


void SilentPinger::RECVIMSCallback(const dmac::DMACPayload& msg){
  ROS_INFO("Received message!!!");
  
  // If the scheme is not enabled, do nothing
  if(!ENABLE_)
    return;

  // Update Clock with the last known instant
  updateClock(msg.timestamp+msg.duration, msg.header.stamp); // deprecated?

  ROS_INFO("Received message. source = %d, modems front = %d", msg.source_address, modems.front());
  
  if(init){
    if(modem_id == anchor){
      ok_check++;
    }//else{
      // // deserialize data
      // std_msgs::String payload_data;
      // payload_data.data = msg.payload + ":vehicle" + std::to_string(msg.source_address);
      // pub_deserialization.publish(payload_data);

      // // compute range
      // tlastRECVIMS = msg.timestamp;
      // // double range = ((tlastRECVIMS-tlastSENDIMS)-tslack*1000000)/2000000.0*SOUND_SPEED;
      // double range = (tlastRECVIMS - (tslot_start + tslack) * 1000000)/1000000.0*SOUND_SPEED;
      
      // slot_ack = true;

      // // Discard very strange measurements
      // if(fabs(range)<range_max){
      //   // Building the message
      //   farol_msgs::mUSBLFix range_msg;
      //   range_msg.type=range_msg.RANGE_ONLY;
      //   range_msg.header.stamp=msg.header.stamp;
      //                         // -ros::Duration(fabs(range)/SOUND_SPEED+tslack/2.0);  time stamp of half-way
      //   range_msg.header.frame_id = "usbl";
      //   range_msg.range = range;
      //   range_msg.source_id = msg.source_address;
      //   range_msg.source_name = msg.source_name;
      //   pub_range.publish(range_msg);
        
      //   ROS_WARN("Range to node %d is %.3f of max value %.3f with frame %s", msg.source_address, range, range_max, msg.header.frame_id.c_str());
      //   return;
      // }
    // }
  }
  if(msg.source_address == modems.front() && msg.type == msg.DMAC_IMS){
    // Check if the ID corresponds to the modem we are expecting
    std::string str;
    str = "Serialization " + msg.payload;
    ROS_INFO("%s", const_cast<char*>(str.c_str()));
    // deserialize data
    std_msgs::String payload_data;
    payload_data.data = msg.payload + ":vehicle" + std::to_string(msg.source_address);
    pub_deserialization.publish(payload_data);

    // compute range
    tlastRECVIMS = msg.timestamp;
    // double range = ((tlastRECVIMS-tlastSENDIMS)-tslack*1000000)/2000000.0*SOUND_SPEED;
    unsigned long int travel_time = (boost::lexical_cast<unsigned long int>(msg.timestamp) + modem_time_offset) 
                                    - (tslot_start + tslack)*1000000; // microseconds
    double range = boost::lexical_cast<double>(travel_time) / 1000000 * SOUND_SPEED;
    
    slot_ack = true;

    // Discard very strange measurements
    if(fabs(range)<range_max){
      // Building the message
      farol_msgs::mUSBLFix range_msg;
      range_msg.type=range_msg.RANGE_ONLY;
      range_msg.header.stamp=msg.header.stamp;
                            // -ros::Duration(fabs(range)/SOUND_SPEED+tslack/2.0);  time stamp of half-way
      range_msg.header.frame_id = "usbl";
      range_msg.range = range;
      range_msg.source_id = msg.source_address;
      range_msg.source_name = msg.source_name;
      pub_range.publish(range_msg);
      
      ROS_WARN("Range to node %d is %.3f of max value %.3f with frame %s", msg.source_address, range, range_max, msg.header.frame_id.c_str());
    }
  }else{
    // If the information is received in the wrong cycle
    ROS_WARN("Received in the wrong cycle: destination modem [%d] instead of [%d]", modems.front(), msg.source_address);
  }
}


// TODO: Ask for Clock to the Modem
void SilentPinger::updateClock(unsigned long int tmodem, ros::Time tros){
  // unsigned long int old_clock=getModemClockNow();
  lastRECVTime_modem = tmodem;
  lastRECVTime_ros = tros;
}


// Get the last modem estimate
unsigned long int SilentPinger::getModemClockNow(){
  ros::Time tnow = ros::Time::now();
  return lastRECVTime_modem + (tnow-lastRECVTime_ros).toSec()*1000000+100000; // 100ms diff detected between estimated and real (maybe processing and receiving messages)
}

void SilentPinger::triggerSerialization(){
  std_msgs::Empty aux;
  pub_triggerserialization.publish(aux);
  waiting_for_serializer = true;
}


void SilentPinger::serializerCallback(const std_msgs::String& msg){
  if(!waiting_for_serializer)
  {
    ROS_ERROR("Received payload to transmit, in the wrong cycle");
    return;
  }
  waiting_for_serializer = false;

  // Send Message to Modem after a slack time
  // unsigned long int tping = round((tslot_start + tslack)*1000000);
  unsigned long int tping = tslot_start + tslack; // this is in UTC time, in seconds

  dmac::DMACPayload im;
  im.header.stamp = ros::Time::now();
  
  im.destination_address = 255;   // Broadcast address

  im.type = im.DMAC_IMS;
  im.timestamp = boost::lexical_cast<unsigned int>(tping*1000000 - modem_time_offset); // timestamp in Modem System Clock (microseconds)
  ROS_INFO("destination address %d, utc timestamp %lu, sys clock timestamp = %lu", 
            im.destination_address, (unsigned long)(tping), (unsigned long)(im.timestamp));
  
  while(ros::Time::now().toSec() < tping);

  // Force undefined time
  tping = 0;
  if(tping==0) im.timestamp_undefined =true;
  else im.timestamp_undefined = false;
  
  // im.ack  = replier_ack;
  im.payload =  msg.data;
  pub_im.publish(im);
  ROS_INFO("Message was sent!");
}


void SilentPinger::pingNextNode(double tinit_aux){
  if(!ENABLE_)
    return;
  
  // Normal pinging behaviour
  if(tinit_aux == 0){
    tslot_start = tslot_end;
    tslot_end = tslot_start + slots.front();

    // update next maximum range because it depends on the timeout
    range_max = (slots.front() - tslack)/2.0*SOUND_SPEED;
  }
  else{
    tslot_start = ros::Time::now().toSec();
    tslot_end = tinit_aux;  // Wait until init time to wake up

    // update next maximum range because it depends on the timeout
    range_max = (tslot_end - tslot_start - tslack)/2.0*SOUND_SPEED;
  }

  if(modem_id == modems.front()){
    // request the serializer to serialize the data to be sent
    ROS_INFO("Triggered serialization");
    triggerSerialization();
  }
  ROS_INFO("duration troubleshooot %3f", tslot_end - ros::Time::now().toSec());
  
  timer.stop();
  timer.setPeriod(ros::Duration(tslot_end - ros::Time::now().toSec()), true);
  timer.start();
}


void SilentPinger::Timer(const ros::TimerEvent& e){
  // Check current mode
  if(init){
    // Send first element to the end of the queue
    modems.push(modems.front());
    modems.pop();
    slots.push(slots.front());
    slots.pop();

    // Check if anchor
    if(modem_id == anchor && modems.front() == modem_id){

      ROS_INFO("Checking if got ok from everyone %d", ok_check);
      if(ok_check == modems.size() - 1){
        // Got ok checks from everyone, anchor moves to normal pinging
        init = false;

        // Reset time input for anchor
        if(modem_id == anchor){
          std_msgs::UInt32 tmsg;
          tmsg.data = 0;
          pub_tinit.publish(tmsg);
        }
      }else{
        // This means it failed the check and is now going to postpone the start
        int hours, minutes, seconds;
        seconds = tnew_start % 60;
        minutes = tnew_start / 60;
        hours = minutes / 60;
        minutes = minutes % 60;

        interrogation_scheme::StartSilent msg;
        msg.hours = hours;
        msg.minutes = minutes;
        msg.seconds = seconds;
        startCallback(msg);
      }
    }
    ROS_INFO("Vai pingar de volta");
    pingNextNode();
  }else{
    if(waiting_for_serializer){
      ROS_WARN("No reply from serializer in %.3fs", slots.front());
    }else if(slot_ack){
      ROS_INFO("Received reply from modem [%d] with slot time %.3fs", modems.front(), slots.front());
      slot_ack = false;
    }else if(modem_id != modems.front()){
      ROS_WARN("No reply from modem [%d] in the corresponding slot %.3fs", modems.front(), slots.front());
    }
    
    waiting_for_serializer = false;

    // Send first element to the end of the queue
    modems.push(modems.front());
    modems.pop();
    slots.push(slots.front());
    slots.pop();

    pingNextNode();
  }
}


/*
 * MAIN
 */
int main(int argc, char **argv){
  ros::init(argc, argv, "silent_acomms");
  ros::NodeHandle nh, nh_private("~");

  SilentPinger pinger(&nh, &nh_private);

  // pinger.pingNextNode();
  ros::spin();

  return 0;
}