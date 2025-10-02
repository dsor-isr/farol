/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#ifndef CATKIN_WS_MISSIONPLANNERALGORITHM_H
#define CATKIN_WS_MISSIONPLANNERALGORITHM_H

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <fstream>
#include <boost/regex.hpp>

class MissionPlannerAlgorithm {
  public:
   
    /* -------------------------------------------------------------------------*/
    /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
    /* -------------------------------------------------------------------------*/
    MissionPlannerAlgorithm();

    /* -------------------------------------------------------------------------*/
    /**
    * @brief  Destructor
    */
    /* -------------------------------------------------------------------------*/
    ~MissionPlannerAlgorithm();

    std::string startNewMission(double north_min, double north_max, double east_min, double east_max,
                                int ID0, int ID1, int ID2, int ID3, double dist_inter_vehicles,
                                int path_orientation, std::vector<double> vehicle_pos,
                                double min_turn_radius, double resolution, 
                                std::string path_type, double velocity, ros::Publisher mission_string_pub, double path_post_rotation,
                                bool publish);

    void sendWaypointsToSailboat(std::vector<double> gliders_avg, double path_main_orientation, 
                                 ros::Publisher sailboat_waypoints_pub, 
                                 double wp_distance_along, double wp_distance_cross,
                                 double wp_offset_along, double wp_offset_cross,
                                 int utm_zone);

  private:

    int getInitialArcDirection(std::vector<double> initial_path_pos, double north_min, 
                               double north_max, double east_min, double east_max,
                               int path_orientation);
    
    int getProgressionSign(std::vector<double> initial_path_pos, double north_min, 
                           double north_max, double east_min, double east_max,
                           int path_orientation);

    double updateCurrentRadius(int initial_line_direction, int line_direction, double normal_radius, 
                               double big_radius, std::string path_type);

    int getInitialLineDirection(std::vector<double> initial_path_pos, double north_min, 
                                double north_max, double east_min, double east_max,
                                int path_orientation);

    bool insideInterestZone(std::vector<double> path_pos, double north_min, double north_max, 
                            double east_min, double east_max);

    double updateLineLength(double north_min, double north_max, double east_min, double east_max, 
                            int initial_line_direction, int line_direction, 
                            double normal_radius, double big_radius,
                            bool is_first_line, bool is_last_line, int path_orientation);

    std::string getPathSections(double north_min, double north_max, double east_min, double east_max,
                                int nr_of_vehicles, double dist_inter_vehicles,
                                int path_orientation, std::vector<double> vehicle_pos,
                                double min_turn_radius, double resolution, 
                                std::string path_type, double velocity);

    void rotatePoint(std::string &x_coord, std::string &y_coord, 
                                 double x_center, double y_center, double x_path_offset, double y_path_offset, 
                                 double x_path_offset_new, double y_path_offset_new, double angle);
    
    std::string rotateMissionPath(const std::string &mission, const double x_center,
                                  const double y_center, double path_post_rotation);

    std::string getNewMissionString(double north_min, double north_max, double east_min, double east_max,
                                    std::vector<int> ids, double dist_inter_vehicles,
                                    int path_orientation, std::vector<double> vehicle_pos,
                                    double min_turn_radius, double resolution,
                                    std::string path_type, double velocity, double path_post_rotation);
                          
    std::string getFormationLine(std::vector<int> ids, double dist_inter_vehicles);

    std::string stampToString(const ros::Time& stamp, const std::string);

    std::vector<double> computeWaypoint(const std::vector<double> ref_point,
                                        const std::vector<double> delta,
                                        const double angle);

    std::string mission_start_;
    std::string mission_string_;

};
#endif
