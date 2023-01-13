#include "wp_standard.h"

WpStandard::WpStandard(ros::Publisher surge_pub, ros::Publisher yaw_pub)
      : surge_pub_(surge_pub), yaw_pub_(yaw_pub) {}

 void WpStandard::calculateRef(Vehicle_t state, WPref_t wp_ref, bool turn_radius_flag) {
    // parameters from config
    double cdist = gains_[0];
    double ku = gains_[1];
    double ks = gains_[2];

    // state input
    double yaw = state.eta2[2];
    double x = state.eta1[0];
    double y = state.eta1[1];

    // waypoint reference input
    double x_ref = wp_ref.eta1[0];
    double y_ref = wp_ref.eta1[1];

    // output
    double yaw_ref{0.0};
    double u_ref{0.0};

    double distance_to_wp = sqrt(pow((x - x_ref), 2) + pow((y - y_ref), 2));

    if (distance_to_wp > cdist) {
      yaw_ref =
          -atan2(-(x - x_ref), -(y - y_ref)) * 180.0 / FarolGimmicks::PI +
          90.0;

      u_ref =
          ku *
          asin((distance_to_wp - cdist) / (fabs(distance_to_wp - cdist) + ks)) *
          2.0 / FarolGimmicks::PI;

      // Computing a gain for surge with respect to the heading error
      double yaw_err = yaw - yaw_ref;

      if (yaw_err > 180)
        yaw_err -= 360;

      if (yaw_err < -180)
        yaw_err += 360;

      double err_0_gain = 60.0;
      double gain = turn_radius_flag ? 0.5 : 0.5 + 
                                              tanh(-fabs(yaw_err) * 2 * FarolGimmicks::PI / err_0_gain + 
                                                   FarolGimmicks::PI) * 0.5;

      u_ref *= gain; 
    } else // waypoint was reached, holding position
    {
      u_ref = 0;
      yaw_ref = getYawOut();
    }
    setSurgeOut(u_ref);
    setYawOut(yaw_ref);

  }

  void WpStandard::publish() {
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(
        surge_pub_, getSurgeOut());
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(yaw_pub_,
                                                                  getYawOut());
  }

