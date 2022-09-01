/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/

#include "waypoint.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoint_node"); // node name
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  ROS_INFO("main: instantiating an object of type WaypointNode");

  WaypointNode waypoint(&nh, &nh_p);

  ros::spin();

  return 0;
}
