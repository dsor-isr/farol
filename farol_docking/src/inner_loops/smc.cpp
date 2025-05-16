// smc.cpp
#include <farol_docking/inner_loops/smc.hpp>  

SMC::SMC(ros::NodeHandle* nodehandle, ros::NodeHandle* nodehandle_private) : ControllerBase(nodehandle, nodehandle_private) {
  // Parameters
  gains_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10);

  // Subscribers
  // sub_gains_ = nh_.subscribe(FarolGimmicks::getParameters<double>(nh_private_, "topics/subscribers/gains", "gains"), 10, &SMC::set_gains_callback, this);

  // Publishers
  // debug_pub_ = nh_private_.advertise<farol_docking::ControllerDebug>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/debug", "debug"), 5);

  SMC::configure();
}



// void SMC::set_gains_callback(const Float64){
//   gains_ = gains
// }

void SMC::configure() {
  // set up the parameters value
  // reset the controller
}


// INTEGRAL SLIDING MODE CONTROLLER IN R3
bool SMC::compute_force(double Dt) {
  //TODO: actual SMC controll law
  force_ << 0,0,0;
  return true;
}


// INTEGRAL SLIDING MODE CONTROLLER IN SO3
bool SMC::compute_torque(double Dt) {
  //TODO: actual SMC controll law
  torque_ << 0,0,0;
  return true;
}

