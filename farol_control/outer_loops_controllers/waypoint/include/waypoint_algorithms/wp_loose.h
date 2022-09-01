#ifndef WP_LOOSE_H
#define WP_LOOSE_H

#include <wp_controller.h>

/**
 * @brief  Waypoint controller similar to standard, using surge and yaw (nose of
 * the vehicle points to the desired position). The difference is that this one
 * limits the rate of yaw reference.
 */
class WpLoose : public WaypointController {
private:
  ros::Publisher surge_pub_;
  ros::Publisher yaw_pub_;

  void calculateRef(Vehicle_t state, WPref_t wp_ref);

  void publish();

public:
  WpLoose(ros::Publisher surge_pub, ros::Publisher yaw_pub);
  virtual ~WpLoose() {}
};

#endif /* WP_LOOSE_H */
