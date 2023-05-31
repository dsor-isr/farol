#ifndef WP_HEADING_H
#define WP_HEADING_H

#include <wp_controller.h>

/**
 * @brief  Waypoint controller using surge, surge and yaw rate.
 * Not only can go to waypoint and hold its position but can also maintain
 * heading.
 */
class WpHeading : public WaypointController {
private:
  ros::Publisher surge_pub_;
  ros::Publisher sway_pub_;
  ros::Publisher yaw_rate_pub_;

  void calculateRef(Vehicle_t state, WPref_t wp_ref, bool turn_radius_flag);

  void publish();

public:
  WpHeading(ros::Publisher surge_pub, ros::Publisher sway_pub,
            ros::Publisher yaw_rate_pub);
  virtual ~WpHeading() {}
};

#endif /* WP_HEADING_H */
