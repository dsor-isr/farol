#ifndef WP_STANDARD_H
#define WP_STANDARD_H

#include <wp_controller.h>

/**
 * @brief  Waypoint controller using surge and yaw, where the nose of
 * the vehicle points to the desired position
 */
class WpStandard : public WaypointController {
private:
  ros::Publisher surge_pub_;
  ros:: Publisher yaw_pub_;

  void calculateRef(Vehicle_t state, WPref_t wp_ref, bool turn_radius_flag) override;

  void publish() override;

public:
  WpStandard(ros::Publisher surge_pub, ros::Publisher yaw_pub);
  virtual ~WpStandard() {}
};

#endif /* WP_STANDARD_H */
