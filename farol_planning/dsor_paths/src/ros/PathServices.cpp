#include "PathNode.h"

/**
 * @brief  A method for initializing all the services. This method is called by the 
 * constructor of the PathNode class upon creation
 */
void PathNode::initializeServices() {
  ROS_INFO("Initializing Services for PathNode");

  /* Advertise the services */
  this->reset_path_srv_ = this->nh_.advertiseService(FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/reset_path", "/reset_path"), &PathNode::ResetPathService, this);
  this->set_mode_srv_ = this->nh_.advertiseService(FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/set_mode", "/set_mode"), &PathNode::SetModeService, this);
  this->arc2d_srv_ = this->nh_.advertiseService(FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/arc2d_path", "/spawn_arc2d_path"), &PathNode::Arc2DService, this);
  this->bernoulli_srv_ = this->nh_.advertiseService(FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/bernoulli_path", "/spawn_bernoulli_path"), &PathNode::BernoulliService, this);
  this->circle2D_srv_ = this->nh_.advertiseService(FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/circle2d_path", "/spawn_circle2d_path"), &PathNode::Circle2DService, this);
  this->line_srv_ = this->nh_.advertiseService(FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/line_path", "/spawn_line_path"), &PathNode::LineService, this);
  this->rabbit_const_speed_srv_ = this->nh_.advertiseService(FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/speed/const_rabbit_speed", "/SetConstVdRabbit"), &PathNode::RabbitConstSpeedService, this);
  this->vehicle_const_speed_srv_ = this->nh_.advertiseService(FarolGimmicks::getParameters<std::string>(this->nh_p_, "topics/services/speed/const_vehicle_speed", "/SetConstVehicleSpeed"), &PathNode::VehicleConstSpeedService, this);
}

/**
 * @brief  A method for terminating all the services. This method is called by the destructor of the PathNode class upon deletion
 */
void PathNode::shutdownServices() {

  /* Shutdown all the services here */
  this->reset_path_srv_.shutdown();
  this->set_mode_srv_.shutdown();
  this->arc2d_srv_.shutdown();
  this->bernoulli_srv_.shutdown();
  this->circle2D_srv_.shutdown();
  this->line_srv_.shutdown();
  this->rabbit_const_speed_srv_.shutdown();
  this->vehicle_const_speed_srv_.shutdown();
}


/**
 * @brief  Auxiliar method to try to load a path section into the path object. If the insertion
 * is not valid, it deletes the object from memory and returns false
 *
 * @param section  A pointer to the path section allocated in the heap
 *
 * @return  A bool indicating whether the section was added successfully or not
 */
bool PathNode::loadSectionIntoPath(PathSection * section) {

  /* Try to add the section to the path */
  bool success = this->path_->addPathSection(section);

  /* If not added successfully, free the memory used for that section */
  if(!success) delete section;

  return success;
}


/**
 * @brief  Auxiliar method to try to load a speed section into the path object. If the insertion
 * is not valid, it deletes the object from memory and returns false
 *
 * @param speed  A pointer to the speed section allocated in the heap
 *
 * @return  A bool indicating whether the section was added successfully or not
 */
bool PathNode::loadSpeedIntoPath(Speed * speed) {

  /* Try to add the speed section to the path */
  bool success = this->path_->addSpeedSection(speed);

  /* If not added successfully, free the memory used for that section */
  if(!success) delete speed;

  return success;
}

bool PathNode::ResetPathService(dsor_paths::ResetPath::Request &req, dsor_paths::ResetPath::Response &res){

  try{

    /* Delete the current path */
    delete this->path_;

    /* Alocate memory for a new path */
    this->path_ = new Path();

    /* Reset the gamma parameter */
    this->gamma_.reset();

    /* Reset the sequence id in the messages */
    this->seq_ = 0;
 
    /* Make sure the operation mode is not to get the closest point from the vehicle but rather a point given the gamma */
    this->closer_point_mode_ = false;

    /* Update the response */
    res.success = true;

  } catch(...) { 
    
    /* Inform the user that there was an error */
    ROS_WARN("Could not reset the Path.");
    res.success = false;
    return false;
  }

  ROS_INFO("PATH RESET SUCCESSFULL");

  return true;
}

bool PathNode::SetModeService(dsor_paths::SetMode::Request &req, dsor_paths::SetMode::Response &res) {

  /* Update the mode of operation (use the closest point on the path to the vehicle) 
   * if closest_point_mode == true, otherwise
   * it expects to listen to a gamma value and give the path data relative to that gamma
   */
  req.closest_point_mode == true ? this->closer_point_mode_ = true : this->closer_point_mode_ = false;
  
  /* Inform the user of the current operation mode */
  if(this->closer_point_mode_) {
    ROS_INFO("Path operation mode: Closest Point");
  } else {
    ROS_INFO("Path operation mode: Gamma");
  }

  res.success = true;
  return true;
}


bool PathNode::Arc2DService(dsor_paths::SpawnArc2D::Request &req, dsor_paths::SpawnArc2D::Response &res){

  // Parse the data from the request
  Eigen::Vector2d start_point;
  Eigen::Vector2d end_point;
  Eigen::Vector2d center_point;

  int direction = req.direction;
  double z = req.z;

  bool success = false;

  for(int i = 0; i < 2; i++) {
    start_point[i] = req.start_point[i];
    end_point[i] = req.end_point[i];
    center_point[i] = req.center_point[i];
  }
  
  /* Validate if the arc is valid, otherwise just return insucess */
  if(((start_point - center_point).norm() < 0.000001) || 
     ((start_point - end_point).norm()    < 0.000001) || 
     ((center_point - end_point).norm()   < 0.000001)) {
    ROS_INFO("ARC2D: Some coordinates are the same! Not added to the path");
    res.success = false;
    return true;
  }

  /* Allocate memory for a new Arc2D Object */
  Arc2D * section = new Arc2D(start_point, end_point, center_point, direction, z);

  /* Try to add the arc section to the path */
  success = this->loadSectionIntoPath(section);

  /* Send the update if the path section was added successfully or not */
  res.success = success;
  ROS_INFO("Adding 2D arc to the path");
  return true;
}

bool PathNode::BernoulliService(dsor_paths::SpawnBernoulli::Request &req, dsor_paths::SpawnBernoulli::Response &res){

  /* Parse the data from the request */
  double radius = req.radius;
  double center_x = req.center_x;
  double center_y = req.center_y;
  double z = req.z;

  bool success = false;

  /* Validate the Bernoulli section */
  if(radius <= 0) {
    ROS_INFO("Bernoulli: Radius <= 0! Not added to the path");
    res.success = false;
    return true;
  }

  /* Allocate memory for a new Bernoulli Object */
  Bernoulli * section = new Bernoulli(radius, center_x, center_y, z);

  /* Try to add the bernoulli to the path */
  success = this->loadSectionIntoPath(section);

  /* Send the update if the path section was added successfully or not */
  res.success = success;
  ROS_INFO("Adding Bernoulli to the path");
  return true;
}

bool PathNode::Circle2DService(dsor_paths::SpawnCircle2D::Request &req, dsor_paths::SpawnCircle2D::Response &res) {

  /* Parse the data from the request */
  double radius = req.radius;
  double center_x = req.center_x;
  double center_y = req.center_y;
  double z = req.z;

  bool success = false;
  
  /* Validate the Circle section */
  if(radius <= 0) {
    ROS_INFO("2DCircle: Radius <= 0! Not added to the path");
    res.success = false;
    return true;
  }

  /* Allocate memory for a new Circle2D Object */
  Circle2D * section = new Circle2D(radius, center_x, center_y, z);

  /* Try to add the Circle2D to the path */
  success = this->loadSectionIntoPath(section);

  /* Send the update if the path section was added successfully or not */
  res.success = success;
  ROS_INFO("Adding Circle2D to the path");
  return true;
}

bool PathNode::LineService(dsor_paths::SpawnLine::Request &req, dsor_paths::SpawnLine::Response &res){

  /* Parse the data from the request */
  Eigen::Vector3d start_point;
  Eigen::Vector3d end_point;
  Eigen::Vector3d ref_point;

  bool success = false;

  for(int i = 0; i < 3; i++) {
    start_point[i] = req.start_point[i];
    end_point[i] = req.end_point[i];
    ref_point[i] = req.ref_point[i];
  }

  /* Validate if the line is valid, otherwise just return insucess */
  if((start_point - end_point).norm() < 0.000001) {
    ROS_INFO("LINE: Start point == End Point. Not added to the path");
    res.success = false;
    return true;
  }

  /* Allocate memory for a new Line Object */
  Line * section = new Line(start_point, end_point, ref_point);

  /* Try to add the Line to the path */
  success = this->loadSectionIntoPath(section);
 
  /* Construct the response back */
  res.success = success;
  ROS_INFO("Adding LINE to the path");
  return true;
}


bool PathNode::RabbitConstSpeedService(dsor_paths::SetConstSpeed::Request &req, dsor_paths::SetConstSpeed::Response &res) {
  
  /* Get the data from the message */
  double speed_val = req.speed;
  double default_val = req.default_speed;
  bool success = false;

  /* Create a new Rabbit Speed object */
  ConstRabbitSpeed * speed = new ConstRabbitSpeed(speed_val, default_val);

  /* Try to add the speed object to the path */
  success = this->loadSpeedIntoPath(speed);

  /* Construct the response back */
  res.success = success;
  if(success == true) ROS_INFO("Load rabbit speed section: %lf", speed_val);

  return true;
}

bool PathNode::VehicleConstSpeedService(dsor_paths::SetConstSpeed::Request &req, dsor_paths::SetConstSpeed::Response &res) {
  
  /* Get the data from the message */
  double speed_val = req.speed;
  double default_val = req.default_speed;
  bool success = false;

  /* Create a new Vehicle Speed object */
  ConstVehicleSpeed * speed = new ConstVehicleSpeed(speed_val, default_val);

  /* Try to add the speed object to the path */
  success = this->loadSpeedIntoPath(speed);

  /* Construct the response back */
  res.success = success;
  if(success == true) ROS_INFO("Load vehicle speed section: %lf m/s", speed_val);

  return true;
}


