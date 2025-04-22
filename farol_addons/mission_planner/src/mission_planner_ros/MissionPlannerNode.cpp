/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "MissionPlannerNode.h"
#include "MissionPlannerAlgorithm.h"

// @.@ Constructor
MissionPlannerNode::MissionPlannerNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

  mission_planner_alg_ = std::make_unique<MissionPlannerAlgorithm>();

}

// @.@ Destructor
MissionPlannerNode::~MissionPlannerNode() {

  // +.+ shutdown publishers
  mission_string_pub_.shutdown();
  new_iz_mission_pub_.shutdown();
  mission_started_ack_pub_.shutdown();
  ready_for_mission_pub_.shutdown();

  // +.+ shutdown subscribers
  state_sub_.shutdown();
  interest_zone_sub_.shutdown();
  new_iz_mission_acomms_sub_.shutdown();
  being_scanned_acomms_sub_.shutdown();
  mission_started_ack_acomms_sub_.shutdown();
  stop_pf_sub_.shutdown();
  glider0_state_usbl_meas_sub_.shutdown();  
  glider1_state_usbl_meas_sub_.shutdown();  

  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void MissionPlannerNode::loadParams() {
  ROS_INFO("Load the MissionPlannerNode parameters");

  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);
  waypoints_update_interval_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/waypoints_update_interval", 360);
  path_post_rotation_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/path_post_rotation", 0.0);
  path_orientation_ = FarolGimmicks::getParameters<int>(nh_private_, "mission_planner/path_orientation", 1);
  path_type_ = FarolGimmicks::getParameters<std::string>(nh_private_, "mission_planner/path_type", "lawnmower_normal");
  min_turning_radius_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/min_turning_radius", 50.0);
  path_speed_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/path_speed", 0.7);
  resolution_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/resolution", 10.0);
  dist_inter_vehicles_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/dist_inter_vehicles", 15.0);
  vehicle_id_ = FarolGimmicks::getParameters<int>(nh_private_, "mission_planner/vehicle_id", 2);
  timeout_ack_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/timeout_ack", 120.0);
  send_waypoints_auvs_following_ = FarolGimmicks::getParameters<bool>(nh_private_, "mission_planner/send_waypoints_auvs_following", false);
  wp_distance_along_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/wp_distance_along", 40.0);
  wp_distance_cross_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/wp_distance_cross", 15.0);
  wp_offset_along_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/wp_offset_along", 0.0);
  wp_offset_cross_ = FarolGimmicks::getParameters<double>(nh_private_, "mission_planner/wp_offset_cross", 10.0);
}

bool MissionPlannerNode::interestZoneService(mission_planner::InterestZone::Request &req,
                                             mission_planner::InterestZone::Response &res) {
  // check if max min values are correct
  if (req.northing_min > req.northing_max || req.easting_min > req.easting_max) {
    res.success = false;
    res.message = "Invalid interest zone.";
    return false;
  }

  std::vector<int> ids = {vehicle_id_};

  // start new mission according to zone of interest published
  // uses path post rotation specified in the config file (addons.yaml)
  mission_string_ = mission_planner_alg_->startNewMission(req.northing_min, req.northing_max, req.easting_min, req.easting_max,
                                                          ids[0], -1, -1, -1, dist_inter_vehicles_,
                                                          path_orientation_, veh_pos_, min_turning_radius_, resolution_,
                                                          path_type_, path_speed_, mission_string_pub_, path_post_rotation_,
                                                          req.start_mission);
  res.success = true;
  res.message = req.start_mission ? "Started new PF Mission on interest zone."
                                  : "Wrote mission file to home dir without actually starting a PF Mission.";
  return true;
}

void MissionPlannerNode::stateCallback(const auv_msgs::NavigationStatus &msg) {
  // update vehicle position
  veh_pos_[0] = msg.position.east;
  veh_pos_[1] = msg.position.north;
}

void MissionPlannerNode::interestZoneCallback(const mission_planner::mInterestZone &msg) {
  // populate new interest zone mission message with IZ and array of IDs corresponding
  // to the participating vehicles in the mission and publish

  // CHECK IF PARTICIPATING VEHICLES HAVE BEEN SCANNED
  // (otherwise, don't send interest zone, we need the array of IDs)
  if (participating_veh_.empty()) {
    ROS_WARN_STREAM("Interest Zone not sent to other vehicles. Need to scan for available vehicles first.");
    return;
  }

  // check if max min values are correct
  if (msg.northing_min > msg.northing_max || 
      msg.easting_min > msg.easting_max) {
    ROS_WARN_STREAM("Interest Zone not sent to other vehicles. Incoherent min/max values.");
    return;
  }

  // check utm zone
  if (msg.utm_zone < 1 || msg.utm_zone > 60) {
    ROS_WARN_STREAM("Interest Zone not sent to other vehicles. Invalid UTM Zone.");
    return;
  }

  // check target_depth
  if (msg.target_depth < 0) {
    ROS_WARN_STREAM("Interest Zone not sent to other vehicles. Invalid target depth.");
    return;
  }

  // convert set of participating vehicles' ids to vector
  std::vector<int> ids(participating_veh_.begin(), participating_veh_.end());

  // msg to be sent
  mission_planner::mNewIZMission new_mission_msg;

  new_mission_msg.interest_zone = msg;
  // make sure angle sent acoustically to gliders is in [0, 360] deg
  new_mission_msg.interest_zone.rotation_angle = FarolGimmicks::wrap2pi(new_mission_msg.interest_zone.rotation_angle/180*M_PI, 0)/M_PI*180;

  new_mission_msg.ID0 = ids[0];
  new_mission_msg.ID1 = (ids.size() < 2) ? -1 : ids[1];
  new_mission_msg.ID2 = (ids.size() < 3) ? -1 : ids[2];
  new_mission_msg.ID3 = (ids.size() < 4) ? -1 : ids[3];

  // publish message
  ROS_WARN_STREAM("Interest Zone sent to other vehicles.");
  new_iz_mission_pub_.publish(new_mission_msg);

  // update flag and save new instant of time
  waiting_for_mission_started_ack_ = true;
  waiting_time_start_ = ros::Time::now().toSec();

  // update last received mission message
  last_IZ_mission_.interest_zone.northing_min = new_mission_msg.interest_zone.northing_min;
  last_IZ_mission_.interest_zone.northing_max = new_mission_msg.interest_zone.northing_max;
  last_IZ_mission_.interest_zone.easting_min = new_mission_msg.interest_zone.easting_min;
  last_IZ_mission_.interest_zone.easting_max = new_mission_msg.interest_zone.easting_max;
  last_IZ_mission_.interest_zone.utm_zone = new_mission_msg.interest_zone.utm_zone;
  last_IZ_mission_.interest_zone.rotation_angle = new_mission_msg.interest_zone.rotation_angle;
  last_IZ_mission_.interest_zone.target_depth = new_mission_msg.interest_zone.target_depth;
  last_IZ_mission_.ID0 = new_mission_msg.ID0;
  last_IZ_mission_.ID1 = new_mission_msg.ID1;
  last_IZ_mission_.ID2 = new_mission_msg.ID2;
  last_IZ_mission_.ID3 = new_mission_msg.ID3;
}

// runs on glider
void MissionPlannerNode::newIZMissionZoneAcommsCallback(const mission_planner::mNewIZMission &msg) { 
  // ack msg
  mission_planner::mMissionStartedAck ack_msg;
  
  // check if max min values are correct
  if (msg.interest_zone.northing_min > msg.interest_zone.northing_max || 
      msg.interest_zone.easting_min > msg.interest_zone.easting_max || 
      msg.ID0 == -1) { // not good
    // send acoustic message back saying PF HAS NOT started
    ack_msg.started_ack = false;

  } else { // everything ok, let's start PF
    // start new mission according to zone of interest published
    // uses path post rotation (rotation_angle) specified in the interest zone message!
    mission_string_ = mission_planner_alg_->startNewMission(msg.interest_zone.northing_min, msg.interest_zone.northing_max, 
                                                            msg.interest_zone.easting_min, msg.interest_zone.easting_max,
                                                            msg.ID0, msg.ID1, msg.ID2, msg.ID3, dist_inter_vehicles_,
                                                            path_orientation_, veh_pos_, min_turning_radius_, resolution_,
                                                            path_type_, path_speed_, mission_string_pub_,
                                                            msg.interest_zone.rotation_angle,
                                                            true);
    
    // send acoustic message back saying PF HAS started
    ack_msg.started_ack = true;

    // reset set of vehicle IDs which have acknowledged their mission has started
    mission_started_veh_.clear();

    // set depth reference to be read my glider interface, which will set target_depth via BSD
    std_msgs::Float64 new_msg;
    new_msg.data = msg.interest_zone.target_depth;
    depth_ref_pub_.publish(new_msg);
  }

  ack_msg.vehicle_ID = vehicle_id_;
  mission_started_ack_pub_.publish(ack_msg);
}

void MissionPlannerNode::beingScannedAcommsCallback(const std_msgs::Bool &msg) {
  // if we are being scanned, send back a message saying we want to be a part of the mission
  // with our vehicle ID

  std_msgs::Int8 new_msg;
  new_msg.data = vehicle_id_;

  ready_for_mission_pub_.publish(new_msg);
}

void MissionPlannerNode::vehicleReadyAcommsCallback(const std_msgs::Int8 &msg) {
  // update set of participating vehicle IDs
  participating_veh_.insert(msg.data);
}

void MissionPlannerNode::stopParticipatingVehiclesPF() {
  // make all participating vehicles publish Flag 0
  std_msgs::Bool bool_msg;
  bool_msg.data = true;
  stop_all_pf_pub_.publish(bool_msg);
}

void MissionPlannerNode::stopPFAcomms(const std_msgs::Bool &msg) {
  // publish Flag 0 to stop PF if it is running
  std_msgs::Int8 new_msg;
  new_msg.data = 0;
  
  status_flag_pub_.publish(new_msg);
}

double MissionPlannerNode::getPathMainOrientationFromMissionString(const std::string &mission_string) {
  std::istringstream stream(mission_string);  // Create a stream from the string
  std::string line;

  // regex pattern for LINE line
  static const boost::regex pattern_line("^LINE ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*)");

  boost::smatch matches;

  // read each line from mission and create rotated mission
  while (std::getline(stream, line)) {
    if (line.rfind("LINE", 0) == 0) { // if line starts with "LINE"
      // # LINE xInit yInit xEnd yEnd velocity <nVehicle> <gamma> <user data>

      // get matches according to pattern
      boost::regex_search(line, matches, pattern_line);

      std::string xInit = matches[1].str();
      std::string yInit = matches[2].str();
      std::string xEnd = matches[3].str();
      std::string yEnd = matches[4].str();

      // get line orientation
      return atan2(std::stod(yEnd) - std::stod(yInit), std::stod(xEnd) - std::stod(xInit)) / M_PI * 180;
    }
  }

  // if the mission has no LINE, return 0.0
  return 0.0;
}

// runs on sailboat/ASV
void MissionPlannerNode::missionStartedAckAcommsCallback(const mission_planner::mMissionStartedAck &msg) {
  // check if mission started
  if (!msg.started_ack) {
    ROS_WARN_STREAM("PF Mission NOT STARTED for vehicle ID: " + std::to_string(msg.vehicle_ID));

    // stop all PF and we're not waiting for acks anymore
    ROS_WARN_STREAM("Stopping PF for all participating vehicles (FLAG 0).");
    stopParticipatingVehiclesPF();
    waiting_for_mission_started_ack_ = false;
    return;
  }

  // if mission has started, add ID to set of vehicles for which the mission has started
  mission_started_veh_.insert(msg.vehicle_ID);
  ROS_WARN_STREAM("PF Mission started normally for vehicle ID: " + std::to_string(msg.vehicle_ID));

  // if the mission has started for all of them, declare SUCCESS and create mission file
  if (mission_started_veh_ == participating_veh_) {
    ROS_WARN_STREAM("All participating vehicles acknowledged mission start.");
    // actually doesn't "START" a new mission, just creates the mission string and writes to file, because of FALSE flag
    // uses path post rotation (rotation_angle) specified in the interest zone message!
    mission_string_ = mission_planner_alg_->startNewMission(last_IZ_mission_.interest_zone.northing_min, last_IZ_mission_.interest_zone.northing_max, 
                                                            last_IZ_mission_.interest_zone.easting_min, last_IZ_mission_.interest_zone.easting_max,
                                                            last_IZ_mission_.ID0, last_IZ_mission_.ID1, last_IZ_mission_.ID2, last_IZ_mission_.ID3, dist_inter_vehicles_,
                                                            path_orientation_, veh_pos_, min_turning_radius_, resolution_,
                                                            path_type_, path_speed_, mission_string_pub_, last_IZ_mission_.interest_zone.rotation_angle,
                                                            false);

    // no longer waiting for acks
    waiting_for_mission_started_ack_ = false;

    // set flag to true to signify PF started for all participating vehicles
    mission_started_ = true;

    // compute path main orientation from new mission string
    path_main_orientation_ = getPathMainOrientationFromMissionString(mission_string_);
  }
}

bool MissionPlannerNode::changeConfigsService(mission_planner::Configs::Request &req,
                                                    mission_planner::Configs::Response &res) {
  // check if new configs are OK (within acceptable values)
  if ((req.path_orientation != 0 && req.path_orientation != 1) ||
      (req.path_type != "lawnmower_normal" && req.path_type != "lawnmower_encircling") ||
      (req.min_turning_radius < 0) ||
      (req.path_speed <= 0) ||
      (req.resolution < 0) ||
      (req.dist_inter_vehicles <= 0) ||
      (req.timeout_ack <= 0)) {
    ROS_WARN_STREAM("Incorrect configurations for Mission Planner.");
    res.success = false;
    res.message = "Incorrect configurations for Mission Planner.";
    return false;
  }
  
  // update configs
  path_post_rotation_ = req.path_post_rotation;
  path_orientation_ = req.path_orientation;
  path_type_ = req.path_type;
  min_turning_radius_ = req.min_turning_radius;
  path_speed_ = req.path_speed;
  resolution_ = req.resolution;
  dist_inter_vehicles_ = req.dist_inter_vehicles;
  timeout_ack_ = req.timeout_ack;
  ROS_INFO("Updated configurations for Mission Planner.");
  res.success = true;
  res.message = "Updated configurations for Mission Planner.";
  return true;
}

void MissionPlannerNode::Glider0StateUsblMeas(const farol_msgs::mState &msg) {
  glider0_State_ = {msg.X, msg.Y};
}

void MissionPlannerNode::Glider1StateUsblMeas(const farol_msgs::mState &msg) {
  glider1_State_ = {msg.X, msg.Y};
}

// @.@ Member helper function to set up subscribers
void MissionPlannerNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for MissionPlannerNode");

  // subscribe to the vehicle state to update veh_pos_
  state_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/state", "dummy"),
    10, &MissionPlannerNode::stateCallback, this);

  // subscribe to the interest zone that is published on the master vehicle, the one who is going to send it via acomms
  interest_zone_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/interest_zone", "dummy"),
    10, &MissionPlannerNode::interestZoneCallback, this);

  // subscribe to the interest zone that comes via acoustic comms
  new_iz_mission_acomms_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/new_iz_mission_acomms", "dummy"),
    10, &MissionPlannerNode::newIZMissionZoneAcommsCallback, this);

  // subscribe to the topic that asks if we are being scanned for a future IZ mission
  being_scanned_acomms_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/being_scanned_acomms", "dummy"),
    10, &MissionPlannerNode::beingScannedAcommsCallback, this);

  // subscribe to the topic that receives the ID of participating vehicles in the IZ mission
  vehicle_ready_acomms_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/vehicle_ready_acomms", "dummy"),
    10, &MissionPlannerNode::vehicleReadyAcommsCallback, this);

  // subscribe to the acknowledgement received from the participation vehicles in the IZ mission
  mission_started_ack_acomms_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/mission_started_ack_acomms", "dummy"),
    10, &MissionPlannerNode::missionStartedAckAcommsCallback, this);

  stop_pf_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/stop_pf", "dummy"),
    10, &MissionPlannerNode::stopPFAcomms, this);

  // if sending waypoints using CANARIAS ist_ros package is enabled
  if (send_waypoints_auvs_following_) {
    glider0_state_usbl_meas_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
      "topics/subscribers/glider0_state_usbl_meas", "dummy"),
      10, &MissionPlannerNode::Glider0StateUsblMeas, this);

    glider1_state_usbl_meas_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
      "topics/subscribers/glider0_state_usbl_meas", "dummy"),
      10, &MissionPlannerNode::Glider1StateUsblMeas, this);
  }
}


// @.@ Member helper function to set up publishers
void MissionPlannerNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for MissionPlannerNode");

  // publisher for the new mission string
  mission_string_pub_ = nh_private_.advertise<std_msgs::String>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/Mission_String", "dummy"), 1);

  new_iz_mission_pub_ = nh_private_.advertise<mission_planner::mNewIZMission>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/new_iz_mission", "dummy"), 1);

  mission_started_ack_pub_ = nh_private_.advertise<mission_planner::mMissionStartedAck>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/mission_started_ack", "dummy"), 1);

  ready_for_mission_pub_ = nh_private_.advertise<std_msgs::Int8>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/ready_for_mission", "dummy"), 1);

  stop_all_pf_pub_ = nh_private_.advertise<std_msgs::Bool>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/stop_all_pf", "dummy"), 1);

  status_flag_pub_ = nh_private_.advertise<std_msgs::Int8>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/status_flag", "dummy"), 1);

  waypoints_pub_ = nh_private_.advertise<std_msgs::Float64MultiArray>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/waypoints", "dummy"), 1);

  depth_ref_pub_ = nh_private_.advertise<std_msgs::Float64>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/depth_ref", "dummy"), 1);
}


// @.@ Member helper function to set up services
void MissionPlannerNode::initializeServices() {
  ROS_INFO("Initializing Services for MissionPlannerNode");

  // servers
  interest_zone_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/services/interest_zone", "dummy"), &MissionPlannerNode::interestZoneService, this);
  
  change_configs_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/services/change_configs", "dummy"), &MissionPlannerNode::changeConfigsService, this);

}


// @.@ Member helper function to set up the timer
void MissionPlannerNode::initializeTimer() {
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &MissionPlannerNode::timerIterCallback, this);
  timer_waypoints_ =nh_.createTimer(ros::Duration(waypoints_update_interval_), &MissionPlannerNode::timerWaypointsCallback, this);
}


// @.@ Where the magic should happen.
void MissionPlannerNode::timerIterCallback(const ros::TimerEvent &event) {
  
  // if waiting for ack from participating vehicles
  if (waiting_for_mission_started_ack_) {
    time_ = ros::Time::now().toSec();

    // if we have been waiting for more than timeout seconds, abort mission and send flag 0 to all vehicles
    if (time_ - waiting_time_start_ > timeout_ack_) {
      // stop all PF and we're not waiting for acks anymore
      ROS_WARN_STREAM("Stopping PF for all participating vehicles (FLAG 0).");
      stopParticipatingVehiclesPF();
      mission_started_ = false;

      // if sending waypoints using CANARIAS ist_ros package is enabled
      if (send_waypoints_auvs_following_) {
        // reset saved positions for participating vehicles
        glider0_State_ = max_coords_;
        glider1_State_ = max_coords_;
      }
      
      waiting_for_mission_started_ack_ = false;
    }
  }
}

void MissionPlannerNode::timerWaypointsCallback(const ros::TimerEvent &event) {
  // if sending waypoints using CANARIAS ist_ros package is enabled AND
  // if path following has successfully started for all participating vehicles
  if (send_waypoints_auvs_following_ && mission_started_) {
    ROS_WARN("Trying to send waypoints to Sailboat.");

    if (participating_veh_.size() >= 2 && glider0_State_ != max_coords_ && glider1_State_ != max_coords_) {
      // if we have position from both participating vehicles (we are considering only 2 AUVs/Gliders)
      gliders_avg_ = {(glider0_State_[0] + glider1_State_[0])/2,
                      (glider0_State_[1] + glider1_State_[1])/2};

      ROS_WARN("Computed gliders_avg based on 2 gliders.");

    } else if (participating_veh_.size() == 1 && 
               (glider0_State_ != max_coords_ || glider1_State_ != max_coords_)) {
      // if there is only one participating vehicle and we have one of the gliders' State
      gliders_avg_ = (glider0_State_ != max_coords_) ? std::vector<double>{glider0_State_[0], 
                                                                           glider0_State_[1]} 
                                                     : std::vector<double>{glider1_State_[0], 
                                                                           glider1_State_[1]};

      ROS_WARN("Computed gliders_avg based on 1 glider.");

    } else if (glider0_State_ == max_coords_ && glider1_State_ == max_coords_) {
      ROS_WARN("Mission started but had no USBL measurements of other gliders.");
      return;
    }

    ROS_WARN("Sending waypoints to Sailboat based on %ld different usbl measurements.", participating_veh_.size());
    mission_planner_alg_->sendWaypointsToSailboat(gliders_avg_, path_main_orientation_,
                                                  waypoints_pub_,
                                                  wp_distance_along_, wp_distance_cross_,
                                                  wp_offset_along_, wp_offset_cross_,
                                                  last_IZ_mission_.interest_zone.utm_zone);

    ROS_WARN("Sent waypoints!");
  }
  // ROS_WARN("timerwaypoints");
}

/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "mission_planner_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type MissionPlannerNode");

  // +.+ instantiate an MissionPlannerNode class object and pass in pointers to nodehandle public and private for constructor to use
  MissionPlannerNode mission_planner(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
