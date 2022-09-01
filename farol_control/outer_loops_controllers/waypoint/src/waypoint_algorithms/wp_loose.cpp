#include "wp_loose.h"

WpLoose::WpLoose(ros::Publisher surge_pub, ros::Publisher yaw_pub)
      : surge_pub_(surge_pub), yaw_pub_(yaw_pub) {}

void WpLoose::calculateRef(Vehicle_t state, WPref_t wp_ref) {
    // parameters from config
    double cdist = gains_[0];
    double ku = gains_[1];
    double ks = gains_[2];
    double max_ref_rate = gains_[3];
    std::cout << max_ref_rate << std::endl;

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

    // previous yaw reference value
    double yaw_ref_old = getYawOut();

    // compute distance to waypoint
    double distance_to_wp = sqrt(pow((y - y_ref), 2) + pow((x - x_ref), 2));

    // waypoint hasn't been reached
    if (distance_to_wp > cdist) {

      // compute yaw and surge references
      yaw_ref =
          -atan2(-(x - x_ref), -(y - y_ref)) * 180.0 / FarolGimmicks::PI +
          90.0;

      u_ref =
          ku *
          asin((distance_to_wp - cdist) / (fabs(distance_to_wp - cdist) + ks)) *
          2.0 / FarolGimmicks::PI;

      // calculate and clip the reference rate
      double yaw_ref_err = yaw_ref - yaw_ref_old;

      if (yaw_ref_err > 180)
        yaw_ref_err -= 360;

      if (yaw_ref_err < -180)
        yaw_ref_err += 360;

      double wref = yaw_ref_err / ts_;

      wref = std::max(std::min(wref, max_ref_rate), -max_ref_rate);

      yaw_ref = yaw_ref_old + wref * ts_;

      //compute yaw error and add a gain to surge based on the error
      double yaw_err = yaw - yaw_ref;

      if (yaw_err > 180)
        yaw_err -= 360;

      if (yaw_err < -180)
        yaw_err += 360;

      double err_0_gain = 60.0;
      double gain = tanh(-fabs(yaw_err) * 2 * FarolGimmicks::PI / err_0_gain +
                         FarolGimmicks::PI) *
                        0.5 +
                    0.5; // normal

      u_ref *= gain;
    } else // waypoint was reached, holding position
    {
      u_ref = 0;
      yaw_ref = getYawOut();
    }
    setSurgeOut(u_ref);
    setYawOut(yaw_ref);
  }

  void WpLoose::publish() {
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(
        surge_pub_, getSurgeOut());
    FarolGimmicks::publishValue<std_msgs::Float64, const double>(yaw_pub_,
                                                                  getYawOut());
  }

  
