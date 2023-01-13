#include "wp_heading.h"

WpHeading::WpHeading(ros::Publisher surge_pub, ros::Publisher sway_pub,
          ros::Publisher yaw_rate_pub)
    : surge_pub_(surge_pub), sway_pub_(sway_pub), yaw_rate_pub_(yaw_rate_pub) {}

void WpHeading::calculateRef(Vehicle_t state, WPref_t wp_ref, bool turn_radius_flag) {
  // parameters from config
  double cdist{gains_[0]};
  double k1{gains_[1]};
  double k2{gains_[2]};
  double k3{gains_[3]};

  // state input
  double yaw{state.eta2[2]};
  double x{state.eta1[0]};
  double y{state.eta1[1]};
  double yaw_rad = yaw * FarolGimmicks::PI / 180;

  // waypoint reference input
  double x_ref{wp_ref.eta1[0]};
  double y_ref{wp_ref.eta1[1]};
  double yaw_ref{wp_ref.eta2[2]};
  double yaw_ref_rad = yaw_ref * FarolGimmicks::PI / 180;

  // output
  double u_ref{0.0};
  double v_ref{0.0};
  double r_ref{0.0};

  double distance_to_wp = sqrt(pow((x - x_ref), 2) + pow((y - y_ref), 2));

  Eigen::Matrix2d rot;
  rot << cos(yaw_rad), -sin(yaw_rad),
         sin(yaw_rad), cos(yaw_rad);
  
  Eigen::Matrix2d gains;
  gains << k1, 0.0,
           0.0, k2;

  Eigen::Vector2d ref, errors, error_body;
  errors << x - x_ref, y - y_ref;

  if (distance_to_wp > cdist) {
    // ref = - gains * tanh( rot * errors )
    error_body = rot * errors;
    error_body = error_body.array().tanh();
    ref = - gains * error_body;  
    
    u_ref = ref(0); v_ref = ref(1);
    // u_ref = -k1 * (x - x_ref) * cos(yaw_rad) - k2 * (y - y_ref) * sin(yaw_rad);
    // v_ref = k1 * (x - x_ref) * sin(yaw_rad) - k2 * (y - y_ref) * cos(yaw_rad);

  } else // waypoint was reached, holding position
  {
    u_ref = 0;
    v_ref = 0;
  }
  r_ref = -k3 * FarolGimmicks::angleDiff(yaw_rad, yaw_ref_rad) * 180.0 / FarolGimmicks::PI;
  setSurgeOut(u_ref);
  setSwayOut(v_ref);
  setYawrateOut(r_ref);
}

void WpHeading::publish() {
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(surge_pub_,
                                                                getSurgeOut());
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(sway_pub_,
                                                                getSwayOut());
  FarolGimmicks::publishValue<std_msgs::Float64, const double>(
      yaw_rate_pub_, getYawrateOut());
}
