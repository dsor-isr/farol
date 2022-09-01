#include "thruster_allocation.h"

ThrustAllocation::ThrustAllocation(ros::NodeHandle &nh) {
  initializeSubscribers(nh);
  initializePublishers(nh);
  loadParams(nh);
}

void ThrustAllocation::initializeSubscribers(ros::NodeHandle &nh) {
  // subscribe to thrust
  ft_sub_ = nh.subscribe(FarolGimmicks::getParameters<std::string>(nh, 
    "topics/subscribers/thrust_body_request", 
    "/thrust_body_request"),
    10, &ThrustAllocation::thrusterAllocation, this);
}

void ThrustAllocation::initializePublishers(ros::NodeHandle &nh) {
  thrusters_pub_ = nh.advertise<dsor_msgs::Thruster>(
    FarolGimmicks::getParameters<std::string>(nh, 
    "topics/publishers/thrusters",
    "/thrusters/RPM_Command"), 1);
}

void ThrustAllocation::loadParams(ros::NodeHandle &nh) {
  max_thrust_norm_ = nh.param("thrusters/max_thrust_norm", 22);
  min_thrust_norm_ = nh.param("thrusters/min_thrust_norm", -22);
  readTAM(nh);
  readCT(nh);
  readRPMGain(nh);
}

void ThrustAllocation::saturateVector(Eigen::VectorXd &thr_thrust) {
  int max_ind, min_ind;
  float maximum = thr_thrust.maxCoeff(&max_ind);
  float minimum = thr_thrust.minCoeff(&min_ind);

  // normalize vector in the case max_value is higher than max_value
  // min and max are independent
  float normalize = 1;

  normalize = std::max(fabs(minimum / min_thrust_norm_), normalize);
  normalize = std::max(fabs(maximum / max_thrust_norm_), normalize);

  for (int i = 0; i < thr_thrust.size(); i++) {
    thr_thrust[i] /= normalize;
  }
}

void ThrustAllocation::readTAM(ros::NodeHandle &nh) {
  XmlRpc::XmlRpcValue allocation_matrix;
  nh.getParam("thrusters/allocation_matrix", allocation_matrix);
  // create Thruster Allocation Matrix (B) with shape [num of forces][num of thrusters]

  Eigen::MatrixXd b(6, allocation_matrix.size() / 6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < allocation_matrix.size() / 6; j++) {
      b(i, j) = allocation_matrix[i + j * 6];
    }
  }
  for (int i = 0; i < allocation_matrix.size() / 6; i++) {
    b.block<3, 1>(3, i) = b.block<3, 1>(3, i).cross(b.block<3, 1>(0, i));
  }
  b_inv_ = b.completeOrthogonalDecomposition().pseudoInverse();
}

void ThrustAllocation::readCT(ros::NodeHandle &nh) {
  XmlRpc::XmlRpcValue ctf;
  XmlRpc::XmlRpcValue ctb;
  nh.getParam("thrusters/ctf", ctf);
  nh.getParam("thrusters/ctb", ctb);
  ctf_ << ctf[0], ctf[1], ctf[2];
  ctb_ << ctb[0], ctb[1], ctb[2];
}

void ThrustAllocation::readRPMGain(ros::NodeHandle &nh) {
  XmlRpc::XmlRpcValue actuators_gain;
  nh.getParam("thrusters/actuators_gain", actuators_gain);

  Eigen::VectorXd aux(actuators_gain.size());
  for (int i = 0; i < actuators_gain.size(); i++) {
    aux(i) = actuators_gain[i];
  }
  rpm_gain_ = aux;
}

void ThrustAllocation::thrusterAllocation(const auv_msgs::BodyForceRequest &msg) {
  Eigen::VectorXd ft_req(6);
  ft_req << float(msg.wrench.force.x), 
            float(msg.wrench.force.y),
            float(msg.wrench.force.z), 
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y), 
            float(msg.wrench.torque.z);

  // Compute the force necessary for each thruster
  Eigen::VectorXd thr_thrust = b_inv_ * ft_req;

  // Saturate thrust
  saturateVector(thr_thrust);

  // Convert from force to % of RPM (because of the drivers - legacy)
  dsor_msgs::Thruster thrust;
  Eigen::VectorXd thr_RPM = thr_thrust;
  for (int i = 0; i < thr_thrust.size(); ++i) {
    if (thr_thrust[i] == 0) {
      thr_RPM[i] = 0;
    } else if (thr_thrust[i] > 0) {
      thr_RPM[i] = (-ctf_[1] + sqrt(ctf_[1] * ctf_[1] - 4 * ctf_[0] * (ctf_[2] - thr_thrust[i]))) / (2 * ctf_[0]);
    } else if (thr_thrust[i] < 0) {
      thr_RPM[i] = (-ctb_[1] + sqrt(ctb_[1] * ctb_[1] - 4 * ctb_[0] * (ctb_[2] - thr_thrust[i]))) / (2 * ctb_[0]);
    }
    thrust.value.push_back(thr_RPM[i] / rpm_gain_[i]);
  }
  thrusters_pub_.publish(thrust);
}

/* Create the Static Thruster Allocation Object */
int main(int argc, char **argv) {
  ros::init(argc, argv, "Static Thruster Allocation");
  ros::NodeHandle nh("~");
  ThrustAllocation thr(nh);
  ros::spin();
}