/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.

Don't you miss the danger ...
*/
#include "ConsolePathParserNode.h"

/**
 * @brief  Console Path Parser node constructor.
 *
 * @param nodehandle  the public ros nodehandle
 * @param nodehandle_private  the private ros nodehandle
 */
ConsolePathParserNode::ConsolePathParserNode(
    ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private)
  : nh_(*nodehandle), nh_private_(*nodehandle_private) {
    ROS_INFO("in class constructor of ConsolePathParserNode");
    loadParams();
    initializeSubscribers();
    initializePublishers();
    initializeServices();
    initializeTimer();

    createPathFolder();
  }

/**
 * @brief  Console Path Parser node destructor
 */
ConsolePathParserNode::~ConsolePathParserNode() {

  /* shutdown publishers */
  fullpath_pub_.shutdown();
  section_pub_.shutdown();
  wpref_pub_.shutdown();
  formation_pub_.shutdown();
  biased_formation_pub_.shutdown();
  altitude_pub_.shutdown();
  depth_pub_.shutdown();

  /* shutdown subscribers */
  missionstring_sub_.shutdown();
  state_sub_.shutdown();
  flag_sub_.shutdown();

  /* Shutdown all the service clients */
  reset_path_client_.shutdown();
  spawn_arc_client_.shutdown();
  spawn_line_client_.shutdown();
  start_pf_client_.shutdown();
  stop_pf_client_.shutdown();
  set_path_speed_client_.shutdown();

  /* stop timer */
  timer_.stop();

  /* shutdown node */
  nh_.shutdown();
  nh_private_.shutdown();
}

/**
 * @brief  Method to initialize all the subscribers
 */
void ConsolePathParserNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for ConsolePathParserNode");
  missionstring_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/Mission_String", "/Mission_String"), 10, &ConsolePathParserNode::missionStringCallback, this);
  state_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state", "/State"), 10, &ConsolePathParserNode::updateCallback, this);
  flag_sub_  = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/flag", "/Flag"), 10, &ConsolePathParserNode::flagCallback, this);
}


/**
 * @brief  Method to initialize all the publishers 
 */
void ConsolePathParserNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for ConsolePathParserNode"); 
  section_pub_    = nh_.advertise<farol_msgs::Section>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/Path_Section", "/Path_Section"), 10);
  formation_pub_  = nh_.advertise<farol_msgs::Formation>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/Formation", "/Formation"), 10, true);
  biased_formation_pub_ = nh_.advertise<farol_msgs::Formation>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/biased_formation", "/biased_formation"), 10, true);
  wpref_pub_      = nh_.advertise<geometry_msgs::PointStamped>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/WPRef", "/WPRef"), 10);
  altitude_pub_   = nh_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/DepthRef", "/ref/depth"), 10);
  depth_pub_      = nh_.advertise<std_msgs::Float64>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/AltRef", "/ref/altitude"), 10);
  fullpath_pub_   = nh_.advertise<farol_msgs::MultiSection>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/FullMission", "/FullMission"), 1, true);
  flag_pub_  = nh_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/flag", "/Flag"), 1, true);
}

/**
 * @brief  Method to initialized all the services
 */
void ConsolePathParserNode::initializeServices() {

  /* Read the service names from the configuration files */
  std::string reset_path_name = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/reset_path");
  std::string arc_section_name = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/arc2d_path");
  std::string line_section_name = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/line_path");
  std::string pf_start_name = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/pf_start");
  std::string pf_stop_name = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/pf_stop");
  std::string speed_name = FarolGimmicks::getParameters<std::string>(nh_private_, "topics/services/set_speed");

  /* Initiate all the service clients */
  reset_path_client_ = nh_.serviceClient<dsor_paths::ResetPath>(reset_path_name);
  spawn_arc_client_ = nh_.serviceClient<dsor_paths::SpawnArc2D>(arc_section_name);
  spawn_line_client_ = nh_.serviceClient<dsor_paths::SpawnLine>(line_section_name);
  start_pf_client_ = nh_.serviceClient<path_following::StartPF>(pf_start_name);
  stop_pf_client_ = nh_.serviceClient<path_following::StopPF>(pf_stop_name);
  set_path_speed_client_ = nh_.serviceClient<dsor_paths::SetConstSpeed>(speed_name);
}

/**
 * @brief  Method to load all the parameters 
 */
void ConsolePathParserNode::loadParams() {
  path_folder = FarolGimmicks::getParameters<std::string>(nh_private_, "path_folder","~/paths_from_console");
  // TODO: probably not being used, legacy
  own_id = FarolGimmicks::getParameters<int>(nh_private_,"vehicle_id");
  p_console_new_ = FarolGimmicks::getParameters<bool>(nh_private_,"console_new", false);
}

/**
 * @brief  Method to initialize the timer
 */
void ConsolePathParserNode::initializeTimer() {
  timer_ = nh_.createTimer(ros::Duration(1.0 / ConsolePathParserNode::nodeFrequency()), &ConsolePathParserNode::depthCallback, this);
  timer_.stop();
}

/**
 * @brief  Method to setup the frequency of the node
 */
double ConsolePathParserNode::nodeFrequency() {
  double node_frequency;
  nh_private_.param("node_frequency", node_frequency, 5.0);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

void ConsolePathParserNode::createPathFolder() {
  namespace fs = std::filesystem;
  fs::create_directories(path_folder);
}

/* 
 * @brief  Mission String Callback. Accepts string and tries to parse mission from it
 *
 * @param msg  A reference to a string message that contains a mission
 */
void ConsolePathParserNode::missionStringCallback(const std_msgs::String &msg) {
  ROS_INFO("New Mission String");
  std::istringstream is(msg.data);
  parseMission(is);

  /* Call the method to request the service to generate the path */
  requestPath();

  /* get pretty name for the mission */
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[BUF_SIZE_TIME];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, BUF_SIZE_TIME, "/%Y-%m-%d_%I-%M-%S.txt", timeinfo);
  ROS_INFO_STREAM("PATH: " << path_folder);
  std::string str(path_folder + buffer);
  
  try {
    ROS_INFO("Saving mission in: [%s]", str.c_str());
    std::ofstream out(str.c_str());
    out << msg.data;
    out.close();
  } catch (int e) {
    ROS_ERROR("An exception occurred while saving path from console to file. Exception Nr. %d", e);
  }
}


/**
 * @brief  Method used to request the path stored in the mission to the path service
 */
void ConsolePathParserNode::requestPath() {

  std::list<Section>::iterator it;
  
  /* Auxiliary variable to check wether there was at least 1 valid section to follow */
  int run = 0;

  /* Call the service to reset the path */
  dsor_paths::ResetPath srv;
  srv.request.reset_path = true;
  reset_path_client_.call(srv);

  if (p_console_new_){
    xrefpoint = 0;
    yrefpoint =0;
  }

  /* Interate over the entire mission that was requested */
  for(it = mission.begin(); it != mission.end(); ++it) {
    /* Check which type of mission we have: 2=Line, 3=Arc */
    if(it->type == 2) {

      /* Make sure the line received is valid */
      if(!(it->xi == it->xe && it->yi == it->ye)) {
        /* Call the service to spawn a line in the path */
        dsor_paths::SpawnLine srv;
        srv.request.start_point[1] = xrefpoint + it->xi;
        srv.request.start_point[0] = yrefpoint + it->yi;
        srv.request.start_point[2] = 0.0;

        srv.request.end_point[1] = xrefpoint + it->xe;
        srv.request.end_point[0] = yrefpoint + it->ye;
        srv.request.end_point[2] = 0.0;
      
        srv.request.ref_point = {0.0, 0.0, 0.0}; 
        spawn_line_client_.call(srv);

        /* Call the service to specify the section desired speed for this section */
        dsor_paths::SetConstSpeed speed_srv;
        speed_srv.request.speed = it->velocity;
        speed_srv.request.default_speed = it->velocity;
        set_path_speed_client_.call(speed_srv);
      
        run++;  /* Increment the number of valid sent sections */
      }
    
    } else if(it->type == 3) {

      /* Make sure the arc received is valid */
      if(!((it->xi == it->xe && it->yi == it->ye) || 
           (it->xi == it->xc && it->yi == it->yc) ||
           (it->xc == it->xe && it->yc == it->ye))) {

        /* Call the service to spawn an arc in the path */
        dsor_paths::SpawnArc2D srv;
        srv.request.start_point[1] = xrefpoint + it->xi;
        srv.request.start_point[0] = yrefpoint + it->yi;
      
        srv.request.end_point[1] = xrefpoint + it->xe;
        srv.request.end_point[0] = yrefpoint + it->ye;

        srv.request.center_point[1] = xrefpoint + it->xc;
        srv.request.center_point[0] = yrefpoint + it->yc;

        srv.request.direction = it->adirection;
        srv.request.z = 0.0;
        spawn_arc_client_.call(srv);
      
        /* Call the service to specify the section desired speed for this section */
        dsor_paths::SetConstSpeed speed_srv;
        speed_srv.request.speed = it->velocity;
        speed_srv.request.default_speed = it->velocity;
        set_path_speed_client_.call(speed_srv);

        /* Increment the number of valid sent sections */
        run++;
      }
    }
  }

  /* If there was at least one valid section to follow, invoke the path following algorithm */
  if(run >= 1) {
      /* Call the path following service */
      path_following::StartPF srv;
      start_pf_client_.call(srv);
  }
}

/*
 * @brief  Function used in MissionString_Callback. The goal is to parse the 
 * string mission to the vehicle format of sections
 *
 * @param is  A reference to a stringstream
 */
void ConsolePathParserNode::parseMission(std::istream &is) {
  int version = 0;
  int lines = 0;
  double Gamma0 = 0;
  // +.+. Which vehicles will cooperate -> LEGACY probably
  std::list<int> IDs_present; 

  // +.+ Initialize formation mode to false
  formation_mode = false; 
  biased_formation_mode = false;
  std::string line;

  farol_msgs::MultiSection FullSection;

  // +.+ Delete the previous mission
  mission.clear();   
  
  // +.+ Delete the previous formation
  formation.clear(); 

  // +.+ Read all the lines from the string
  while (getline(is, line)) {
    ROS_INFO("parseMission new line: [%s]", line.c_str());
    Section newSection;
    int res = 0;
    if (line[0] == '#') // Discard comments
      continue;
    lines++;          // Number of useful lines
    if (lines == 1) { // Version
      sscanf(line.c_str(), "%d", &version);
      if (version != 3) {
        ROS_ERROR("Mission version [%d] not supported. Supported versions: 3",
            version);
        return;
      }
      continue;
    }

    // +.+ Reference Points
    if (lines == 2) { 
      sscanf(line.c_str(), "%lf %lf %*s", &xrefpoint, &yrefpoint);
      if (p_console_new_){
        FullSection.RefPoint.x = 0;
        FullSection.RefPoint.y = 0;
      }
      FullSection.RefPoint.x = xrefpoint;
      FullSection.RefPoint.y = yrefpoint;
    }

    // +.+ Formation info
    if (line.compare(0, 9, "FORMATION") == 0) {
      std::vector<std::string> formation_str;
      boost::split(formation_str, line, boost::is_any_of("\t "));

      std::vector<std::string>::iterator fstrs;
      Formation aux;
      for (fstrs = formation_str.begin() + 1; fstrs != formation_str.end();
           fstrs = fstrs + 3) {
        if (strlen((*fstrs).c_str()) == 0)
          break;
        // +.+ Build the object
        aux.id = atoi((*fstrs).c_str());
        aux.x = atof((*(fstrs + 1)).c_str());
        aux.y = atof((*(fstrs + 2)).c_str());
        formation.push_back(aux); // add a new vehicle

        // +.+ For changing the path and see if this vehicle will act in this mission
        if (atoi((*fstrs).c_str()) == own_id) {
          formation_mode = true;
          x_forma = atof((*(fstrs + 1)).c_str());
          y_forma = atof((*(fstrs + 2)).c_str());
          ROS_INFO("FORMATION x=%lf y=%lf", x_forma, y_forma);
        }
      }

      if (!formation_mode) { 
        ROS_WARN("This mission is not for this vehicle, no id=%d in the "
                 "FORMATION line",
                 own_id);
        return;
      }
      // +.+ Don't add anything to the mission
      continue;
    } else if (line.compare(0, 4, "BIAS") == 0) {
      std::vector<std::string> formation_str;
      boost::split(formation_str, line, boost::is_any_of("\t "));

      std::vector<std::string>::iterator fstrs;
      Formation aux;
      for (fstrs = formation_str.begin() + 1; fstrs != formation_str.end();
           fstrs = fstrs + 3) {
        if (strlen((*fstrs).c_str()) == 0)
          break;
        // +.+ Build the object
        aux.id = atoi((*fstrs).c_str());
        aux.x = atof((*(fstrs + 1)).c_str());
        aux.y = atof((*(fstrs + 2)).c_str());
        formation.push_back(aux); // add a new vehicle

        // +.+ For changing the path and see if this vehicle will act in this mission
        if (atoi((*fstrs).c_str()) == own_id) {
          biased_formation_mode = true;
          x_forma = atof((*(fstrs + 1)).c_str());
          y_forma = atof((*(fstrs + 2)).c_str());
          ROS_INFO("BIASED FORMATION x=%lf y=%lf", x_forma, y_forma);
        }
      }

      if (!biased_formation_mode) { 
        ROS_WARN("This mission is not for this vehicle, no id=%d in the "
                 "FORMATION line",
                 own_id);
        return;
      }
      // Don't add anything to the mission
      continue; 
    } 
    // +.+ Point
    else if (line.compare(0, 5, "POINT") == 0) {
      ROS_INFO("POINT %s", line.c_str());
      newSection.type = 1;
      if ((res = sscanf(line.c_str(), "POINT %lf %lf %f %f %f %f %d",
              &newSection.xe, &newSection.ye, &newSection.radius,
              &newSection.velocity, &newSection.heading,
              &newSection.time, &newSection.nVehicle)) ==
          6) { // Optional value
        newSection.nVehicle = -1;
        Gamma0 = 0;
        newSection.gamma_s = 0;
        newSection.gamma_e = 0;
      } else if (res >= 7) {
        Gamma0 = 0;
        newSection.gamma_s = 0;
        newSection.gamma_e = 0;
      } else {
        continue;
      }
    } 
    // +.+ Line
    else if (line.compare(0, 4, "LINE") == 0) {
      newSection.type = 2;
      if ((res = sscanf(line.c_str(), "LINE %lf %lf %lf %lf %f %d %lf",
                        &newSection.xi, &newSection.yi, &newSection.xe,
                        &newSection.ye, &newSection.velocity,
                        &newSection.nVehicle, &newSection.gamma_e)) == 5) { 
        newSection.nVehicle = -1;
      }
      if (res >= 5 && res <= 6) {
        Gamma0 = (*--mission.end()).gamma_e;
        newSection.gamma_s = Gamma0;
        newSection.gamma_e = Gamma0 + sqrt(pow(newSection.xe - newSection.xi, 2) + pow(newSection.ye - newSection.yi, 2));
      } else {
        continue;
      }
    } 
    // +.+ Arc
    else if (line.compare(0, 3, "ARC") == 0) {
      newSection.type = 3;
      newSection.gamma_e = -1;
      
      if ((res = sscanf(
               line.c_str(), "ARC %lf %lf %lf %lf %lf %lf %f %d %f %d %lf",
               &newSection.xi, &newSection.yi, &newSection.xc, &newSection.yc,
               &newSection.xe, &newSection.ye, &newSection.velocity,
               &newSection.adirection, &newSection.radius, &newSection.nVehicle,
               &newSection.gamma_e)) == 9) { // Optional value
        newSection.nVehicle = -1;
      }

      if (res >= 9 && res <= 10) {
        Gamma0 = (*--mission.end()).gamma_e;
        double psis = atan2(newSection.yi - newSection.yc, newSection.xi - newSection.xc);
        double psie = atan2(newSection.ye - newSection.yc, newSection.xe - newSection.xc);
        newSection.gamma_s = Gamma0;
        
        // +.+  Check direction
        // +.+ Clockwise
        if (newSection.adirection == -1)
          newSection.gamma_e = Gamma0 + ANG_P(psis - psie) * newSection.radius;
        // +.+ Counter clockwise
        else
          newSection.gamma_e = Gamma0 + (2 * PI - ANG_P(psis - psie)) * newSection.radius;
      } else {
        continue;
      }
    } 
    // +.+ Depth
    else if (line.compare(0, 5, "DEPTH") == 0) {
      if ((res = sscanf(line.c_str(), "DEPTH %f %f %d", &newSection.depth, &newSection.time, &newSection.nVehicle)) == 3) {
        newSection.type = 4;
      } else
        ROS_ERROR("Erroneous construction of DEPTH Primitive.");
    } 
    // +.+ Alt
    else if (line.compare(0, 3, "ALT") == 0) {
      if ((res = sscanf(line.c_str(), "ALT %f %f %d", &newSection.depth, &newSection.time, &newSection.nVehicle)) == 3) {
        newSection.type = 5;
      } else
        ROS_ERROR("Erroneous construction of Altitude Primitive.");
    } 
    // +.+ Unknown
    else
      continue;

    // +.+ Create variables for RViz Visualization -> LEGACY
    geometry_msgs::Point StartPoint;
    geometry_msgs::Point CenterPoint;
    geometry_msgs::Point EndPoint;
    StartPoint.x = newSection.xi;
    StartPoint.y = newSection.yi;
    StartPoint.z = 0.0;
    CenterPoint.x = newSection.xc;
    CenterPoint.y = newSection.yc;
    CenterPoint.z = 0.0;
    EndPoint.x = newSection.xe;
    EndPoint.y = newSection.ye;
    EndPoint.z = 0.0;

    FullSection.type.push_back(newSection.type);
    FullSection.adirection.push_back(newSection.adirection);
    FullSection.StartPoint.push_back(StartPoint);
    FullSection.CenterPoint.push_back(CenterPoint);
    FullSection.EndPoint.push_back(EndPoint);
    FullSection.Gamma_s.push_back(newSection.gamma_s);
    FullSection.Gamma_e.push_back(newSection.gamma_e);

    mission.push_back(newSection);
  }

  FullSection.header.stamp = ros::Time::now();
  fullpath_pub_.publish(FullSection);

  farol_msgs::Formation Form_Topic; 
  
  // +.+ Send the formation to the Cooperative_PF

  // +.+ If we are in the presence of a Formation definition
  if (formation_mode || biased_formation_mode) {
    Form_Topic.ID.resize(formation.size());
    Form_Topic.x.resize(formation.size());
    Form_Topic.y.resize(formation.size());
    std::list<Formation>::iterator j;
    int i = 0;
    for (j = formation.begin(); j != formation.end(); ++j, ++i) {
      Form_Topic.ID[i] = (*j).id;
      Form_Topic.x[i] = (*j).x;
      Form_Topic.y[i] = (*j).y;
    }
    if (formation_mode)
      // +.+ Change the mission accordingly
      missionFormation();            
  } 
  // +.+ If in the fille there is only gammas and specific missions for each vehicle - LEGACY
  else if (IDs_present.size() > 0) { 
    Form_Topic.ID.resize(IDs_present.size());
    Form_Topic.x.resize(IDs_present.size());
    Form_Topic.y.resize(IDs_present.size());
    std::list<int>::iterator j;
    int i = 0;
    for (j = IDs_present.begin(); j != IDs_present.end(); ++j, ++i) {
      Form_Topic.ID[i] = (*j);
      Form_Topic.x[i] = 0;
      Form_Topic.y[i] = 0;
    }
  } 
  // +.+ Normal Path-Following without Cooperation (constant speed)
  else {
    Form_Topic.ID.resize(1);
    Form_Topic.x.resize(1);
    Form_Topic.y.resize(1);
    Form_Topic.ID[0] = own_id;
    Form_Topic.x[0] = 0;
    Form_Topic.y[0] = 0;
  }

  // +.+ Print ref point 
  printf("---------------------------------------------------------\n");
  printf("Mission: xrefpoint=%lf yrefpoint=%lf\n\n", xrefpoint, yrefpoint);
  
  std::list<Section>::iterator i;
  
  for (i = mission.begin(); i != mission.end(); ++i) {
    // +.+ Gammas and Section Length for all the sections of this vehicle
    if (((*i).type == 2 || (*i).type == 3) && ((*i).nVehicle == -1 || (*i).nVehicle == own_id)) {
      
      Form_Topic.Gamma_e.push_back((*i).gamma_e);

      // +.+ Start point, end point, center point
      Eigen::VectorXf ps(2), pe(2), pc(2); 
      ps[0] = (*i).xi;
      ps[1] = (*i).yi;
      pe[0] = (*i).xe;
      pe[1] = (*i).ye;
      pc[0] = (*i).xc;
      pc[1] = (*i).yc;

      // +.+ Lines
      if ((*i).type == 2) { // Lines
        Form_Topic.Length.push_back((pe - ps).norm());
      } 
      // +.+ Arcs
      else if ((*i).type == 3) {
        // +.+ Initial angle                         
        double phi0 = atan2(ps(1) - pc(1), ps(0) - pc(0)); 
        
        // +.+ Final angle
        double phie = atan2(pe(1) - pc(1), pe(0) - pc(0)); 
        
        // +.+ Arc radius
        double R = (pc - ps).norm();                      
        double arc_len = 0;
        
        // +.+. Check direction
        // +.+ Clockwise
        if ((*i).adirection == -1) {
          arc_len = ANG_P(phi0 - phie) * R;
        }
        // +.+ Counter-clockwise
        else{
          arc_len = (2 * PI - ANG_P(phi0 - phie)) * R;
        }
        Form_Topic.Length.push_back(arc_len);
      }
    }

    printf("SECTION type=%d Xi=%lf Yi=%lf Xc=%lf Yc=%lf Xe=%lf Ye=%lf Vl=%f "
           "Dir=%d R0=%lf NV=%d Gs=%lf Ge=%lf\n\n",
           (*i).type, (*i).xi, (*i).yi, (*i).xc, (*i).yc, (*i).xe, (*i).ye,
           (*i).velocity, (*i).adirection, (*i).radius, (*i).nVehicle,
           (*i).gamma_s, (*i).gamma_e);
  }
  printf("---------------------------------------------------------\n\n");

  // TODO: Legacy here, need to review
  // +.+ Send formation to the Cooperative_PF but it is only activated by the section
  if (biased_formation_mode)
    biased_formation_pub_.publish(Form_Topic);
  else
    formation_pub_.publish(Form_Topic);
  
  // +.+ Detected Mission to do
  if (mission.size() > 0) {
    // +.+ Initialize variables
    act_section = mission.begin();
    gamma_s = 0;
    gamma_e = 0;
    ENABLE = true;

    // +.+ Start the Mission
    startNewSection();
  }
}

// @.@ Function used in MissionString_Callback -> for cooperative pf
void ConsolePathParserNode::missionFormation() {
  std::list<Section>::iterator i;
  for (i = mission.begin(); i != mission.end(); ++i) {
    // +.+ Can only change the mission if it doesn't have any vehicle associatead
    if ((*i).nVehicle != -1) 
      continue;

    // +.+ Waypoint
    if ((*i).type == 1) { // WayPoint
      continue;
    // +.+ Straight Line
    } else if ((*i).type == 2) { // Straight Line
      double psi = atan2((*i).ye - (*i).yi, (*i).xe - (*i).xi);
      (*i).xi = (*i).xi + x_forma * cos(psi) - y_forma * sin(psi);
      (*i).yi = (*i).yi + x_forma * sin(psi) + y_forma * cos(psi);
      (*i).xe = (*i).xe + x_forma * cos(psi) - y_forma * sin(psi);
      (*i).ye = (*i).ye + x_forma * sin(psi) + y_forma * cos(psi);
    // +.+ Arc
    } else if ((*i).type == 3) { 
      double psis = atan2((*i).yi - (*i).yc, (*i).xi - (*i).xc) + (*i).adirection * PI / 2;
      double psie = atan2((*i).ye - (*i).yc, (*i).xe - (*i).xc) + (*i).adirection * PI / 2;
      (*i).xi = (*i).xi + x_forma * cos(psis) - y_forma * sin(psis);
      (*i).yi = (*i).yi + x_forma * sin(psis) + y_forma * cos(psis);
      (*i).xe = (*i).xe + x_forma * cos(psie) - y_forma * sin(psie);
      (*i).ye = (*i).ye + x_forma * sin(psie) + y_forma * cos(psie);
    }
  }
}

// @.@ Function to manage switch between sections of the path from console, and what to do in the end
void ConsolePathParserNode::startNewSection() {
  // +.+ Verifify if the section is for this vehicle
  while (act_section != mission.end() &&
         !((*act_section).nVehicle == -1 || (*act_section).nVehicle == own_id)) {
    ROS_WARN("This section is not for this vehicle [ID=%d]",
             (*act_section).nVehicle);
    ROS_INFO("SECTION T=%d Xi=%lf Yi=%lf Xc=%lf Yc=%lf Xe=%lf Ye=%lf Vl=%f "
             "Dir=%d R0=%lf NV=%d Gs=%lf Ge=%lf",
             (*act_section).type, (*act_section).xi, (*act_section).yi,
             (*act_section).xc, (*act_section).yc, (*act_section).xe,
             (*act_section).ye, (*act_section).velocity, (*act_section).adirection,
             (*act_section).radius, (*act_section).nVehicle,
             (*act_section).gamma_s, (*act_section).gamma_e);
    ++act_section;
  }

  // +.+ Mission End
  if (act_section == mission.end()) {
    ROS_INFO("Mission Finished");

    geometry_msgs::PointStamped aux;
    aux.point.x = -3;
    aux.point.y = -3;
    gamma_s = 0;
    gamma_e = 0;
    // +.+  Hold Position in the same point
    wpref_pub_.publish(aux);

    // +.+ Publish, during 2 sec, 0 depth and kill timer
    DesiredDepth = 0.0;
    timer_.stop();

    ENABLE = false;
    return;
  }

  ROS_INFO("SECTION T=%d Xi=%lf Yi=%lf Xc=%lf Yc=%lf Xe=%lf Ye=%lf Vl=%f "
           "Dir=%d R0=%lf NV=%d Gs=%lf Ge=%lf",
           (*act_section).type, (*act_section).xi, (*act_section).yi,
           (*act_section).xc, (*act_section).yc, (*act_section).xe,
           (*act_section).ye, (*act_section).velocity, (*act_section).adirection,
           (*act_section).radius, (*act_section).nVehicle, (*act_section).gamma_s,
           (*act_section).gamma_e);
  
  if (p_console_new_){
      xrefpoint = 0;
      yrefpoint = 0;
    }
  // +.+ Waypoint
  if ((*act_section).type == 1) {

    ROS_INFO("Starting a Waypoint");
    geometry_msgs::PointStamped aux;

    // +.+ Controller needs the Global position
    aux.point.x = (*act_section).xe + xrefpoint;
    aux.point.y = (*act_section).ye + yrefpoint;
    gamma_s = 0;
    // +.+ As long as a waypoint is required even in the middle of the mission gamma starts counting from 0
    gamma_e = 0; 

    wpref_pub_.publish(aux);
  // +.+ Line or Arc  
  } else if ((*act_section).type == 2 || (*act_section).type == 3) { 

    ROS_INFO("Starting a Path Following Mission");
    farol_msgs::Section aux;
    aux.xrefpoint = xrefpoint;
    aux.yrefpoint = yrefpoint;
    aux.direction = (*act_section).adirection;
    aux.xs = (*act_section).xi;
    aux.ys = (*act_section).yi;
    aux.xc = (*act_section).xc;
    aux.yc = (*act_section).yc;
    aux.xe = (*act_section).xe;
    aux.ye = (*act_section).ye;
    aux.Vl = (*act_section).velocity;
    aux.direction = (*act_section).adirection;
    aux.R0 = (*act_section).radius;
    aux.Gamma_s = (*act_section).gamma_s;
    aux.Gamma_e = (*act_section).gamma_e; 
    section_pub_.publish(aux);

    // +.+ New gamma is the previous one
    gamma_s = gamma_e;               
    // +.+ Updated gamma end
    gamma_e = (*act_section).gamma_e; 
    ROS_WARN("Gamma: %f", gamma_e);
  // +.+ Depth and Altitude References
  } else if ((*act_section).type == 4 || (*act_section).type == 5) {
    ROS_INFO("STARTING DEPTH PUBLISHING");
    if ((*act_section).type == 5)
      DesiredDepth = -(*act_section).depth;
    else
      DesiredDepth = (*act_section).depth;

    // +.+ Activate the end of references for depth
    if ((*act_section).time > 0) {
      depth_end = ros::Time::now() + ros::Duration((*act_section).time);
    } else {
      gamma_s = 0;
      gamma_e = 0;
      ++act_section;
      startNewSection();
      depth_end = ros::Time(0);
    }
    timer_.start();
  }
}


/*
#######################################################################################################################
 @.@ Callbacks Section / Methods
#######################################################################################################################
*/

 // @.@ Depth Callback -> Iteration via timer callback
void ConsolePathParserNode::depthCallback(const ros::TimerEvent &event) {
  // +.+ Depth with schedule end
  if (depth_end != ros::Time(0)) {
    if ((depth_end - ros::Time::now()).toSec() <= 0) {
      DesiredDepth = 0.0;
      timer_.stop();
      gamma_s = 0;
      gamma_e = 0;
      ++act_section;
      startNewSection();
      return;
    }
  }

  std_msgs::Float64 depth;
  // +.+ Depth Reference
  if (DesiredDepth >= 0) {
    depth.data = DesiredDepth;
    depth_pub_.publish(depth);
    ROS_INFO_THROTTLE(10.0, "Desired Depth: %f", depth.data);
  } else {
    depth.data = -DesiredDepth;
    altitude_pub_.publish(depth);
    ROS_INFO_THROTTLE(10.0, "Desired Altitude: %f", depth.data);
  }
}

// @.@ Flag Callback 
void ConsolePathParserNode::flagCallback(const std_msgs::Int8 &msg) {
  // +.+ Check the Flag if there is a mission running
  if (!ENABLE)
    return;

  // +.+ Abort everything if received flag == 0
  if (msg.data == 0) { // IDLE
    ENABLE = false;
    timer_.stop();
  }
  // +.+ Path-Following Mission - Flag should be 6
  if ((*act_section).type == 2 || (*act_section).type == 3) {
    if (msg.data != 6) {
      ENABLE = false;
      timer_.stop();
      ROS_ERROR("Mission Ended due to changes in the Flag. Got %d instead of 6",
                msg.data);
    }
  }
}

// @.@ Callback for updating 2D position (X,Y) values in the State topic
void ConsolePathParserNode::updateCallback(const auv_msgs::NavigationStatus &msg) {

	// waypoint works in ENU but state from filter is in NED
  x_act = msg.position.east;
  y_act = msg.position.north;
  
  if (p_console_new_){
    xrefpoint = 0;
    yrefpoint = 0;
  }
  if (ENABLE && (*act_section).type == 1) { 
    // +.+ Position and end point
    Eigen::VectorXf Pos(2), pe(2);
    pe[0] = (*act_section).xe;
    pe[1] = (*act_section).ye;
    Pos[0] = x_act - xrefpoint;
    Pos[1] = y_act - yrefpoint;
    if ((Pos - pe).norm() < (*act_section).radius * 1.2) { // On the Point
      if ((*act_section).time != -1) {
        ros::Duration((*act_section).time).sleep();
        ++act_section;
        // +.+ Start Next Section
        startNewSection();
      }
    }
  }
}

// @.@ Callback for comparing the calculated gamma from a pf algorithm with the
void ConsolePathParserNode::missionCallback(const std_msgs::String &msg) {
  ROS_INFO("Mission [%s]", msg.data.c_str());
}

/**
 * @brief  The main function. The entry point to the program
 *
 * @param argc  The number of arguments
 * @param argv  An array with arguments
 *
 * @return  An int with success or failure status
 */
int main(int argc, char **argv) {

  // +.+ ROS set-ups:
  ros::init(argc, argv, "console_path_parser_node"); // node name

  // +.+ create a node handler
  ros::NodeHandle nh, nh_p("~");

  ROS_INFO("main: instantiating an object of type ConsolePathParserNode");

  // +.+ instantiate an ConsolePathParserNode class object and pass in pointers to nodehandlers for constructor to use
  ConsolePathParserNode consolePathParser(&nh, &nh_p);

  // +.+ Working with timer -> going into spin; let the callbacks do all the work
  ros::spin();

  return 0;
}
