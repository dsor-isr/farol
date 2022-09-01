#include <ros/ros.h>
#include "innerloops.h"
#include "safeties.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "innerloops");
  ros::NodeHandle nh("~");

  Innerloops con(nh);
  Safeties sft(nh);

  ros::spin();
}
