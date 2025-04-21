#include "MissionPlannerAlgorithm.h"

// @.@ Constructor
MissionPlannerAlgorithm::MissionPlannerAlgorithm() {
  mission_start_ = "3\n";

}

// @.@ Destructor
MissionPlannerAlgorithm::~MissionPlannerAlgorithm() {

}

bool MissionPlannerAlgorithm::insideInterestZone(std::vector<double> path_pos, double north_min, double north_max, 
                                                       double east_min, double east_max) {
  if (path_pos[0] >= east_min && path_pos[0] <= east_max && path_pos[1] >= north_min && path_pos[1] <= north_max) {
    return true;
  }

  return false;
}

int MissionPlannerAlgorithm::getInitialArcDirection(std::vector<double> initial_path_pos, double north_min, 
                                                          double north_max, double east_min, double east_max,
                                                          int path_orientation) {
  // determine what's the arc direction for the first turn

  // if initial position is the bottom left or top right corner
  if ((initial_path_pos[0] == east_min && initial_path_pos[1] == north_min) || 
      (initial_path_pos[0] == east_max && initial_path_pos[1] == north_max)) {
    return (path_orientation == 0) ? 1 : -1;
  }
  // else: top left or bottom right corner
  return (path_orientation == 0) ? -1 : 1;
}

int MissionPlannerAlgorithm::getInitialLineDirection(std::vector<double> initial_path_pos, double north_min, 
                                                           double north_max, double east_min, double east_max,
                                                           int path_orientation) {
  // determine what's the line direction initially
  if ((initial_path_pos[0] == east_min && initial_path_pos[1] == north_min) ||
      ((initial_path_pos[0] == east_min && initial_path_pos[1] == north_max) && path_orientation == 0) ||
      ((initial_path_pos[0] == east_max && initial_path_pos[1] == north_min) && path_orientation == 1)) {
    return 1;
  } else {
    return -1;
  }
}

int MissionPlannerAlgorithm::getProgressionSign(std::vector<double> initial_path_pos, double north_min, 
                                                      double north_max, double east_min, double east_max,
                                                      int path_orientation) {
  // determine the sign of progression along the zone of intereset
  if (path_orientation == 0) { // E-W
    return (initial_path_pos[1] == north_min) ? 1 : -1;
  } else { // N-S
    return (initial_path_pos[0] == east_min) ? 1 : -1;
  }
}

double MissionPlannerAlgorithm::updateCurrentRadius(int initial_line_direction, int line_direction, double normal_radius, 
                                                          double big_radius, std::string path_type) {
  if (path_type == "lawnmower_normal") { // radius is always the min_turn_radius
    return normal_radius;
  } else if (path_type == "lawnmower_encircling") { // big radius in one leg direction, normal radius in reverse direction
    return (line_direction == initial_line_direction) ? big_radius : normal_radius;
  } else { // default
    return normal_radius;
  }
}

double MissionPlannerAlgorithm::updateLineLength(double north_min, double north_max, double east_min, double east_max, 
                                                       int initial_line_direction, int line_direction, 
                                                       double normal_radius, double big_radius,
                                                       bool is_first_line, bool is_last_line, int path_orientation) {
  double line_length = (path_orientation == 0) ? (east_max - east_min) // E-W
                                               : (north_max - north_min); // N-S
  
  if (is_first_line) { // FIRST LINE
    return line_length - big_radius;
  } else if (is_last_line) { // LAST LINE
    return (line_direction == initial_line_direction) ? line_length - normal_radius : line_length - big_radius;
  } else { // all other lines
    return line_length - big_radius - normal_radius;
  }
}

std::string MissionPlannerAlgorithm::getPathSections(double north_min, double north_max, double east_min, double east_max,
                                                           int nr_of_vehicles, double dist_inter_vehicles,
                                                           int path_orientation, std::vector<double> vehicle_pos,
                                                           double min_turn_radius, double resolution,
                                                           std::string path_type, double velocity) {
  // path sections string
  std::string path_sections_string = "";

  // current type of section
  std::string curr_section = "LINE";

  // is first line?
  bool is_first_line = true;

  // is last line?
  bool is_last_line = false;
  
  // current intersection, get closest corner to vehicle
  std::vector<double> path_pos = {0.0, 0.0};
  path_pos[0] = (abs(vehicle_pos[0] - east_min) < abs(vehicle_pos[0] - east_max)) ? east_min : east_max;
  path_pos[1] = (abs(vehicle_pos[1] - north_min) < abs(vehicle_pos[1] - north_max)) ? north_min : north_max;

  // sign of progression along the zone of interest (positive or negative, across the easting or northing axis)
  int progression_sign = getProgressionSign(path_pos, north_min, north_max, east_min, east_max, path_orientation);

  // positive or negative direction for the line
  int initial_line_direction = getInitialLineDirection(path_pos, north_min, north_max, east_min, east_max, path_orientation);
  int line_direction = initial_line_direction;

  // current arc direction (left = 1 or right = -1)
  int current_adirection = getInitialArcDirection(path_pos, north_min, north_max, east_min, east_max, path_orientation);

  // compute normal and big radii
  double normal_radius = min_turn_radius + dist_inter_vehicles * (nr_of_vehicles / 2 - 0.5); // sum to the minimum turn radius a compensation for the number of vehicles cooperating
  double big_radius = (path_type == "lawnmower_encircling") ? normal_radius + resolution/2 : normal_radius;

  // current radius and line_length
  double curr_radius = 0;
  double line_length = 0;

  while (insideInterestZone(path_pos, north_min, north_max, east_min, east_max) || curr_section == "LINE") {
    if (curr_section == "LINE") {
      // add line -> # LINE xInit yInit xEnd yEnd velocity <nVehicle> <gamma> <user data>

      curr_radius = updateCurrentRadius(initial_line_direction, line_direction, normal_radius, big_radius, path_type);

      line_length = updateLineLength(north_min, north_max, east_min, east_max, 
                                     initial_line_direction, line_direction, 
                                     normal_radius, big_radius,
                                     is_first_line, is_last_line, path_orientation);

      if (path_orientation == 0) { // E-W lawnmower
        path_sections_string += "LINE " + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1]) + " " 
                                        + std::to_string(path_pos[0] + line_direction * line_length) + " " + std::to_string(path_pos[1]) + " "
                                        + std::to_string(velocity) + " -1\n";

        // update current path position
        path_pos[0] += line_direction * line_length;

      } else { // N-S lawnmower
        path_sections_string += "LINE " + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1]) + " " 
                                        + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1] + line_direction * line_length) + " "
                                        + std::to_string(velocity) + " -1\n";

        // update current path position
        path_pos[1] += line_direction * line_length;
      }

      // we are no longer in the first line
      is_first_line = false;

      // update line direction
      line_direction = (line_direction == 1) ? -1 : 1;

    } else {
      // add arc -> # ARC xInit yInit xCenter yCenter xEnd yEnd velocity adirection radius <nVehicle> <gamma> <user data>
      if (path_orientation == 0) { // E-W lawnmower
        path_sections_string += "ARC " + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1]) + " "
                                       + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1] + progression_sign * curr_radius) + " "
                                       + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1] + progression_sign * 2 * curr_radius) + " "
                                       + std::to_string(velocity) + " " + std::to_string(current_adirection) + " "
                                       + std::to_string(curr_radius) + " -1\n";

        // update current path position
        path_pos[1] += progression_sign * 2 * curr_radius;

      } else { // N-S lawnmower
        path_sections_string += "ARC " + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1]) + " "
                                       + std::to_string(path_pos[0] + progression_sign * curr_radius) + " " + std::to_string(path_pos[1]) + " "
                                       + std::to_string(path_pos[0] + progression_sign * 2 * curr_radius) + " " + std::to_string(path_pos[1]) + " "
                                       + std::to_string(velocity) + " " + std::to_string(current_adirection) + " "
                                       + std::to_string(curr_radius) + " -1\n";

        // update current path position
        path_pos[0] += progression_sign * 2 * curr_radius;

      }

      // update next arc direction if last section was an arc and path type is lawnmower_normal
      if (path_type == "lawnmower_normal") {
        current_adirection *= -1;
      } else if (path_type == "lawnmower_encircling") { // don't change adirection, but change progression sign
        progression_sign *= -1;
      }

      // if after this arc the curent path position is outside the interest zone, then the next line is the last one
      if (!insideInterestZone(path_pos, north_min, north_max, east_min, east_max)) {
        is_last_line = true;
      }

    }
    
    // update current section for next iteration
    curr_section = (curr_section == "LINE") ? "ARC" : "LINE";
  }

  return path_sections_string;
}

std::string MissionPlannerAlgorithm::getFormationLine(std::vector<int> ids, double dist_inter_vehicles) {
  // FORMAT: FORMATION ID1 x_dist y_dist ID2 x_dist y_dist ID3 x_dist y_dist
  std::string formation_line = "FORMATION";
  int i = 0; // iterator for for loop
  
  // leftmost path offset
  double leftmost_offset = - dist_inter_vehicles * (ids.size() / 2 - 0.5);
  
  // add offset for each vehicle ID
  for (int id : ids) {
    formation_line += " " + std::to_string(id) + " 0 " + std::to_string(leftmost_offset + dist_inter_vehicles * i);
    i++;
  }

  // next line
  formation_line += "\n";
  
  return formation_line;
}

void MissionPlannerAlgorithm::rotatePoint(std::string &x_coord, std::string &y_coord, 
                                          double x_center, double y_center, double x_path_offset, double y_path_offset, 
                                          double x_path_offset_new, double y_path_offset_new, double angle) {
  // bring point to origin of the axes
  double x = std::stod(x_coord) + x_path_offset - x_center;
  double y = std::stod(y_coord) + y_path_offset - y_center;

  // apply rotation matrix
  double x_new = x * cos(angle) - y * sin(angle);
  double y_new = x * sin(angle) + y * cos(angle);

  // bring point back to original center
  x_coord = std::to_string(x_new + x_center - x_path_offset_new);
  y_coord = std::to_string(y_new + y_center - y_path_offset_new);
}

std::string MissionPlannerAlgorithm::rotateMissionPath(const std::string &mission, const double x_center, 
                                                       const double y_center, double path_post_rotation) {
  int counter = 0; // counnter for while function
  
  // position of the path frame's origin on the inertial frame
  double x_path_offset = 0;
  double y_path_offset = 0;

  // position of the path_frame's origin on the inertial frame after rotation around the center of the interest zone
  double x_path_offset_new = 0;
  double y_path_offset_new = 0;

  std::string rotated_mission = "";
  
  std::istringstream stream(mission);  // Create a stream from the string
  std::string line;

  // regex match patterns
  static const boost::regex pattern_path_center("^([^ ]*) ([^ ]*)$");
  static const boost::regex pattern_line("^LINE ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*)");
  static const boost::regex pattern_arc("^ARC ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*)");

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

      rotatePoint(xInit, yInit, x_center, y_center, x_path_offset, y_path_offset, x_path_offset_new, y_path_offset_new, path_post_rotation/180*M_PI);
      rotatePoint(xEnd, yEnd, x_center, y_center, x_path_offset, y_path_offset, x_path_offset_new, y_path_offset_new, path_post_rotation/180*M_PI);
      
      // reconstruct line, with rotated points
      rotated_mission += "LINE " + xInit + " " + yInit + " " + xEnd + " " + yEnd;
      
      for (int i = 5; i < (int)matches.size(); i++) {
        rotated_mission += " " + matches[i].str();
      }

      rotated_mission += "\n";

    } else if (line.rfind("ARC", 0) == 0) { // if line starts with "ARC"
      // # ARC xInit yInit xCenter yCenter xEnd yEnd velocity adirection radius <nVehicle> <gamma> <user data>

      // get matches according to pattern
      boost::regex_search(line, matches, pattern_arc);

      std::string xInit = matches[1].str();
      std::string yInit = matches[2].str();
      std::string xCenter = matches[3].str();
      std::string yCenter = matches[4].str();
      std::string xEnd = matches[5].str();
      std::string yEnd = matches[6].str();

      rotatePoint(xInit, yInit, x_center, y_center, x_path_offset, y_path_offset, x_path_offset_new, y_path_offset_new, path_post_rotation/180*M_PI);
      rotatePoint(xCenter, yCenter, x_center, y_center, x_path_offset, y_path_offset, x_path_offset_new, y_path_offset_new, path_post_rotation/180*M_PI);
      rotatePoint(xEnd, yEnd, x_center, y_center, x_path_offset, y_path_offset, x_path_offset_new, y_path_offset_new, path_post_rotation/180*M_PI);
      
      // reconstruct line, with rotated points
      rotated_mission += "ARC " + xInit + " " + yInit + " " + xCenter + " " + yCenter + " " + xEnd + " " + yEnd;
      
      for (int i = 7; i < (int)matches.size(); i++) {
        rotated_mission += " " + matches[i].str();
      }

      rotated_mission += "\n";

    } else if (counter == 1){ // second line of file which contains offset for path coordinates
      // get matches according to pattern
      boost::regex_search(line, matches, pattern_path_center);

      x_path_offset = std::stod(matches[1].str());
      y_path_offset = std::stod(matches[2].str());

      // bring path offset to origin of the axes, rotate it according to the path_post_rotation angle and bring it back to the original position
      x_path_offset_new = ((x_path_offset - x_center) * cos(path_post_rotation) - (y_path_offset - y_center) * sin(path_post_rotation)) + x_center;
      y_path_offset_new = ((x_path_offset - x_center) * sin(path_post_rotation) + (y_path_offset - y_center) * cos(path_post_rotation)) + y_center;

      rotated_mission += std::to_string(x_path_offset_new) + " " + std::to_string(y_path_offset_new) + "\n";

    } else { // if line does not start with "LINE" or "ARC" and is not the second line
      rotated_mission += line + "\n";
    }

    // increase counter for while function
    counter += 1;
  }

  return rotated_mission;
}

std::string MissionPlannerAlgorithm::getNewMissionString(double north_min, double north_max, double east_min, double east_max,
                                                         std::vector<int> ids, double dist_inter_vehicles,
                                                         int path_orientation, std::vector<double> vehicle_pos,
                                                         double min_turn_radius, double resolution, 
                                                         std::string path_type, double velocity, double path_post_rotation) {
  // create new string
  std::string mission = mission_start_;

  // add mission reference point (bottom left corner of the zone of interest)
  mission += std::to_string(east_min) + " " + std::to_string(north_min) + "\n";

  // add formation line if there is more than 1 vehicle
  if (ids.size() > 1) {
    mission += getFormationLine(ids, dist_inter_vehicles);
  }

  // move vehicle position to zone of interest's frame
  vehicle_pos[0] -= east_min;
  vehicle_pos[1] -= north_min;

  // add path sections
  mission += getPathSections(0, north_max - north_min, 0, east_max - east_min,
                             ids.size(), dist_inter_vehicles,
                             path_orientation, vehicle_pos,
                             min_turn_radius, resolution, path_type, velocity);

  return rotateMissionPath(mission, (east_max + east_min)/2, (north_max + north_min)/2, path_post_rotation);
}

std::string MissionPlannerAlgorithm::stampToString(const ros::Time& stamp, const std::string format="%Y-%m-%d-%H-%M-%S") {
  const int output_size = 100;
  char output[output_size];
  std::time_t raw_time = static_cast<time_t>(stamp.sec);
  struct tm* timeinfo = localtime(&raw_time);
  std::strftime(output, output_size, format.c_str(), timeinfo);
  std::stringstream ss; 
  ss << std::setw(9) << std::setfill('0') << stamp.nsec;
  return std::string(output);
}

std::string MissionPlannerAlgorithm::startNewMission(double north_min, double north_max, double east_min, double east_max,
                                                     int ID0, int ID1, int ID2, int ID3, double dist_inter_vehicles,
                                                     int path_orientation, std::vector<double> vehicle_pos,
                                                     double min_turn_radius, double resolution,
                                                     std::string path_type, double velocity, ros::Publisher mission_string_pub, double path_post_rotation,
                                                     bool publish) {

  // create set with non negative ids
  std::set<int> ids_set;
  ids_set.insert(ID0);

  if (ID1 != -1) {
    ids_set.insert(ID1);
    if (ID2 != -1) {
      ids_set.insert(ID2);
      if (ID3 != -1) {
        ids_set.insert(ID3);
      }
    }
  }

  // create vector from set
  std::vector<int> ids(ids_set.begin(), ids_set.end());
  
  // get mission string
  mission_string_ = getNewMissionString(north_min, north_max, east_min, east_max,
                                        ids, dist_inter_vehicles,
                                        path_orientation, vehicle_pos,
                                        min_turn_radius, resolution, 
                                        path_type, velocity, path_post_rotation);

  ROS_INFO("\nMISSION FILE:\n");
  std::cout << mission_string_;

  if (publish) {
    std_msgs::String new_mission;
    new_mission.data = mission_string_;

    // publish new mission string to start new PF
    mission_string_pub.publish(new_mission);
  } else { // special case for writing mission to file
    ROS_WARN_STREAM("Writing mission file.");

    std::string home = getenv("HOME");
    std::string dir = home + "/ramones_mission_" + stampToString(ros::Time::now()) + ".txt";

    // write to file
    std::ofstream myfile;
    myfile.open(dir);
    myfile << mission_string_;
    myfile.close();
  }

  return mission_string_;
}

std::vector<double> MissionPlannerAlgorithm::computeWaypoint(const std::vector<double> ref_point,
                                                             const std::vector<double> delta,
                                                             const double angle) {                                                              
  std::vector<double> wp{ref_point[0] + delta[0]*cos(angle) - delta[1]*sin(angle),
                         ref_point[1] + delta[0]*sin(angle) + delta[1]*cos(angle)};

  return wp;
}

void MissionPlannerAlgorithm::sendWaypointsToSailboat(std::vector<double> gliders_avg, double path_main_orientation, 
                                                      ros::Publisher sailboat_waypoints_pub, 
                                                      double wp_distance_along, double wp_distance_cross,
                                                      double wp_offset_along, double wp_offset_cross,
                                                      int utm_zone) {
  // path main orientation in rad
  double alpha = path_main_orientation/180*M_PI;

  // distances and offsets
  double a = wp_distance_along;
  double c = wp_distance_cross;
  double a_offset = wp_offset_along;
  double c_offset = wp_offset_cross;
  
  // waypoints w1, w2
  // std::vector<double> w1{gliders_avg[0] + wp_distance*cos(alpha), 
  //                       gliders_avg[1] + wp_distance*sin(alpha)}, 
  //                    w2{gliders_avg[0] - wp_distance*cos(alpha), 
  //                       gliders_avg[1] - wp_distance*sin(alpha)};

  // std::vector<double> wps{w1[0], w1[1], w2[0], w2[1]};

  // waypoints 1,2,3,4, R - 2-dim rotation matrix with angle alpha
  //
  // w1 = p_avg + R . | + a + a_offset |
  //                  | + c + c_offset |
  std::vector<double> w1 = computeWaypoint(gliders_avg, std::vector<double>{a + a_offset, c + c_offset}, alpha);
  //
  // w2 = p_avg + R . | + a + a_offset |
  //                  | - c + c_offset |
  std::vector<double> w2 = computeWaypoint(gliders_avg,  std::vector<double>{a + a_offset, - c + c_offset}, alpha);
  //
  // w3 = p_avg + R . | - a + a_offset |
  //                  | + c + c_offset |
  std::vector<double> w3 = computeWaypoint(gliders_avg,  std::vector<double>{- a + a_offset, c + c_offset}, alpha);
  //
  // w4 = p_avg + R . | - a + a_offset |
  //                  | - c + c_offset |
  std::vector<double> w4 = computeWaypoint(gliders_avg,  std::vector<double>{- a + a_offset, - c + c_offset}, alpha);
  
  // build vector with all waypoints coordinates in order PLUS UTM_ZONE IN THE END
  std::vector<double> wps{w1[0], w1[1], w2[0], w2[1], w3[0], w3[1], w4[0], w4[1], static_cast<double>(utm_zone)};

  ROS_WARN("MISSION PLANNER AFTER WPS COMPUTING: easting/northings:\n%f/%f\n%f/%f\n%f/%f\n%f/%f", 
                                                                  w1[0], w1[1], w2[0], w2[1],
                                                                  w3[0], w3[1], w4[0], w4[1]);

  std_msgs::Float64MultiArray msg;
  msg.data = wps;

  sailboat_waypoints_pub.publish(msg);
}