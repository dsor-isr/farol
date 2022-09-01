#include "Path.h"
#include <ros/ros.h> // TODO - only used for debug printing

/* Default constructor assumes a simple path */
Path::Path() {}

/**
 * @brief  Destructor for the Path object. Deletes the memory allocated for
 * each path section inside the vector of sections
 */
Path::~Path() {

  /* Free the memory used by path sections inside the vector of sections */
  for(std::vector<PathSection*>::iterator i = this->sections_.begin(); i != this->sections_.end(); ++i) {
    delete *i;
  }

  /* Free the memory used by the path speeds inside the vector of speeds */
  for(std::vector<Speed*>::iterator i = this->speeds_.begin(); i != this->speeds_.end(); ++i) {
    delete *i;
  }

  /* Clear the vector that contains the pointers to de-allocated memory */
  this->sections_.clear();
  this->speeds_.clear();
  
  /* Clear the vector that stores the limits for each path section */
  this->sections_limits_.clear();
}

/* Method to check if a path already has a section or not */
bool Path::isEmpty() {
  return this->sections_.size() == 0 ? true : false;
}

/* Add a new path section to the path */
bool Path::addPathSection(PathSection * path_section) { 

  /* If the vector of sections is empty, always add the path section to the path */
  if(this->isEmpty()) {

    /* Add the section to the vector */
    this->sections_.emplace_back(path_section);

    /* Decide now if the path is going to be a complex of different sections or just
     * a simple section path */
    this->type_of_path_ = path_section->can_be_composed() ? COMPLEX_PATH : SIMPLE_PATH;
    
    /* Add the section bounds to the vector of sections_limits */
    if (this->type_of_path_ == COMPLEX_PATH) {
      /* If we have a complex path, then by the default the first gamma is assumed to start at 0 */
      this->sections_limits_.emplace_back(std::make_pair(0, path_section->getMaxGammaValue()));
    } else {
      /* If we have a simple path, then by default the gamma can be from -inf to inf */
      this->sections_limits_.emplace_back(std::make_pair(path_section->getMinGammaValue(), path_section->getMaxGammaValue()));
    }
    return true;
  }

  /* If the Type of path is complex, we can add multiple segments of paths */
  if(this->type_of_path_ == COMPLEX_PATH && path_section->can_be_composed()) {

    /*Add the new segment to the path */
    this->sections_.emplace_back(path_section);

    /* If is the first element, then the section bounds are [0, max_gamma[ */
    if(this->sections_.size() == 1) {
        
        /* Add the section bounds to the vector of sections_limits */
        this->sections_limits_.emplace_back(std::make_pair(0, path_section->getMaxGammaValue()));
    
    } else {
        
        /* Get the last value of the section limit */
        std::pair<double, double> prev_bounds = this->sections_limits_.back(); 

        /* Add the section bounds to the vector of sections_limits */
        this->sections_limits_.emplace_back(std::make_pair(prev_bounds.second, prev_bounds.second + path_section->getMaxGammaValue()));
    }

    return true;
  }

  /* If the path is simple and the section list is not empty, we cannot add the new section to the path */
  return false;
}

/* Get the corresponding Path section given a gamma, and the gamma normalized
 * for that section
 */
std::tuple<PathSection*, double> Path::getPathSection(double gamma) {

  /* Check if the path has no sections yet */
  if(this->sections_.size() == 0) {

    /* Return nullopt if there is not path section yet */
    return std::make_tuple(nullptr, 0.0);
  }

  /* Check if there is only one path section in the path */
  if(this->type_of_path_ == SIMPLE_PATH || this->sections_.size() == 1) {

    /* Limit the gamma between the minimum and maximum bounds for that segment */
    gamma = this->sections_[0]->limitGamma(gamma);

    /* Return the section and the gamma */
    return std::make_tuple(this->sections_[0], gamma);
  }

  /* Check if the path is composed by multiple segments */
  if(this->type_of_path_ == COMPLEX_PATH) {

    /* Handle the special case where gamma < 0 */
    if (gamma < 0) {
      return std::make_tuple(this->sections_[0], 0.0);
    }

    /* Handle the special case where gamma > max_value_of_last_section*/
    std::pair<double, double> bounds = this->sections_limits_.back();
    
    if (gamma > bounds.second) {
      return std::make_tuple(this->sections_.back(), this->sections_.back()->getMaxGammaValue());
    }

    /* Get the index of the section that corresponds to that gamma using binary search algorithm */
    int idx = getIndexOfSection(gamma, 0, this->sections_.size()-1);
    
    /* Get the gamma bounds for that section */
    bounds = this->sections_limits_[idx];

    /* Return the corresponding section and gamma normalized for that section */
    return std::make_tuple(this->sections_[idx], gamma - bounds.first);
}

  /* Otherwise, some strange behaviour occurred, therefore return NULL and original gamma */
  return std::make_tuple(nullptr, gamma);
}

/* Recursive function to get the index of the corresponding section 
 * Using binary search algorithm. In worst case is O(log(N)) complexity
 */
int Path::getIndexOfSection(double gamma, int left, int right) {
      
  /* Get the middle index */
  int mid = left + (right - left) / 2;

  /* See which gamma limits are stored in that index */
  std::pair<double, double> bounds = this->sections_limits_[mid];

  /* If gamma is within bounds, then we have the correct index */
  if(gamma >= bounds.first && gamma <= bounds.second) {
    return mid;
  }

  /* If gamma is lower than the bound of the left, search on the left side*/
  if(gamma < bounds.first) {
    return this->getIndexOfSection(gamma, left, mid-1); 
  }

  /* gamma > bounds.second */
  return this->getIndexOfSection(gamma, mid+1, right);
  
}

/* Method to add a speed section to the path */
bool Path::addSpeedSection(Speed * speed_section) {
  
  /* Check if there are no sections yet. If not, then do not add the speed */
  /* First add the path section, only after the desired speed for it */
  if(this->sections_.size() == 0) {
    return false;
  }
  
  /* Check if the number of path_sections is higher than the number of speeds_sections*/
  if(this->sections_.size() > this->speeds_.size()) {
      
      /* Then we can add a speed desired to the section */
      this->speeds_.emplace_back(speed_section);
      return true;
  }

  /* If we reach here we have some unplanned behaviour, so do not add to the list of speeds */
  return false;
}

/* Method to get the desired speed for a section given the path parameter */
std::optional<double> Path::eq_vd(double gamma) {
  
  /* Check the case where the path is empty */
  if(this->sections_.size() == 0) return std::nullopt;

  /* Check the case where the path is not empty but the speed profile is */
  if(this->sections_.size() != 0 && this->speeds_.size() == 0) return std::optional<double>{0.0};

  std::pair<double, double> bounds;

  /* Check the case where we have a simple path */
  if(this->type_of_path_ == SIMPLE_PATH && this->speeds_.size() != 0) {

    /* Check if gamma is out of the bounds or not */
    bounds = this->sections_limits_[0];

    /* If gamma is out of the bounds, just return the default velocity profile specified */
    if(gamma < bounds.first) {
      return std::optional<double>{this->speeds_[0]->getDefaultVd(bounds.first, this->sections_[0]->derivative_norm(bounds.first))};
    } else if (gamma > bounds.second) {
      return std::optional<double>{this->speeds_[0]->getDefaultVd(bounds.second, this->sections_[0]->derivative_norm(bounds.second-bounds.first))};
    }

    /* If gamma is within bounds, just return the respective speed profile */
    return std::optional<double>{this->speeds_[0]->getVd(gamma, this->sections_[0]->derivative_norm(gamma))};
  }

  /* Check if we have a complex path */
  if(this->type_of_path_ == COMPLEX_PATH && this->speeds_.size() != 0) {
    
    /* Check if gamma is bellow the minimum bound */
    bounds = this->sections_limits_[0];

    /* Check if gamma < min value */
    if (gamma < bounds.first) {
      return std::optional<double>{this->speeds_[0]->getDefaultVd(bounds.first, this->sections_[0]->derivative_norm(bounds.first))};
    }
      
    /* Get the last index that contains a speed profile */
    int last_idx = this->speeds_.size()-1;

    /* Check if gamma is bigger than the last bound for the last speed profile */
    bounds = this->sections_limits_[last_idx];
    
    if(gamma > bounds.second) {

      /* Since gamma is bigger than what is specified in last speed profile, just return the default speed after that */
      return std::optional<double>{this->speeds_[last_idx]->getDefaultVd(bounds.second - bounds.first, this->sections_[last_idx]->derivative_norm(bounds.second - bounds.first))};
    }

    /* Get the index to where the correct profile is stored for this gamma (since gamma is within bounds) */
    int idx = getIndexOfSection(gamma, 0, last_idx);

    /* Get the gamma bounds for that section */
    bounds = this->sections_limits_[idx];
    
    /* Get the internal gamma corresponding to that particular section/speed profile */
    gamma = gamma - bounds.first;

    /* Return the speed profile for that particular internal gamma */
    return std::optional<double>{this->speeds_[idx]->getVd(gamma, this->sections_[idx]->derivative_norm(gamma))};
  }

  /* If did not go inside the if statements, then some special bug occurred. In that
   * case return the null optional */
  return std::nullopt;
}

/* Method to get the desired acceleration for a section given the path parameter */
std::optional<double> Path::eq_d_vd(double gamma) {
  
  /* Check the case where the path is empty */
  if(this->sections_.size() == 0) return std::nullopt;

  /* Check the case where the path is not empty but the speed profile is */
  if(this->sections_.size() != 0 && this->speeds_.size() == 0) return std::optional<double>{0.0};

  std::pair<double, double> bounds;

  /* Check the case where we have a simple path */
  if(this->type_of_path_ == SIMPLE_PATH && this->speeds_.size() != 0) {

    /* Check if gamma is out of the bounds or not */
    bounds = this->sections_limits_[0];

    /* If gamma is out of the bounds, just return 0.0 */
    if(gamma < bounds.first || gamma > bounds.second) {
      return std::optional<double>{0.0};
    }

    /* If gamma is within bounds, just return the respective acceleration profile */
    return std::optional<double>{this->speeds_[0]->get_d_Vd(gamma, this->sections_[0]->derivative_norm(gamma))};
  }

  /* Check if we have a complex path */
  if(this->type_of_path_ == COMPLEX_PATH && this->speeds_.size() != 0) {
    
    /* Check if gamma is bellow the minimum bound */
    bounds = this->sections_limits_[0];

    /* Check if gamma < min value */
    if (gamma < bounds.first) {
      return std::optional<double>{0.0};
    }
      
    /* Get the last index that contains a speed profile */
    int last_idx = this->speeds_.size()-1;

    /* Check if gamma is bigger than the last bound for the last speed profile */
    bounds = this->sections_limits_[last_idx];
    
    if(gamma > bounds.second) {

      /* Since gamma is bigger than what is specified in last speed profile, just return 0.0  */
      return std::optional<double>{0.0};
    }

    /* Get the index to where the correct profile is stored for this gamma (since gamma is within bounds) */
    int idx = getIndexOfSection(gamma, 0, last_idx);

    /* Get the gamma bounds for that section */
    bounds = this->sections_limits_[idx];
    
    /* Get the internal gamma corresponding to that particular section/speed profile */
    gamma = gamma - bounds.first;

    /* Return the acceleration profile for that particular internal gamma */
    return std::optional<double>{this->speeds_[idx]->get_d_Vd(gamma, this->sections_[idx]->derivative_norm(gamma))};
  }

  /* If did not go inside the if statements, then some special bug occurred. In that
   * case return the null optional */
  return std::nullopt;
}

/* Get the corresponding position to the gamma parameter */
std::optional<Eigen::Vector3d> Path::eq_pd(double gamma) {

  Eigen::Vector3d pd;

  /* Get the path section corresponding to that gamma */
  PathSection* path_section = nullptr;
  double gamma_internal;

  std::tie (path_section, gamma_internal) = this->getPathSection(gamma);
  
  /* Check if there was a section in the path list */
  if(path_section) {

    /* Get the desired position for that section */
    pd = path_section->eq_pd(gamma_internal);

    /* Return the desired position */
    return std::optional<Eigen::Vector3d>{pd};
  }

  /* Return nothing if there is no path section */
  return std::nullopt;
}

std::optional<Eigen::Vector3d> Path::eq_d_pd(double gamma) {

  Eigen::Vector3d d_pd;

  /* Get the path section corresponding to that gamma */
  PathSection* path_section = nullptr;
  double gamma_internal;

  std::tie (path_section, gamma_internal) = this->getPathSection(gamma);

  /* Check if there was a section in the path list */
  if(path_section) {

    /* Get the derivative for that section */
    d_pd = path_section->eq_d_pd(gamma_internal);

    /* Return the desired position */
    return std::optional<Eigen::Vector3d>{d_pd};
  }

  /* Return nothing if there is no path section*/
  return std::nullopt;
}

std::optional<Eigen::Vector3d> Path::eq_dd_pd(double gamma) {

  Eigen::Vector3d dd_pd;

  /* Get the path section corresponding to that gamma */
  PathSection* path_section = nullptr;
  double gamma_internal;

  std::tie (path_section, gamma_internal) = this->getPathSection(gamma);

  /* Check if there was a section in the path list */
  if(path_section) {

    /* Get the second derivative for that section */
    dd_pd = path_section->eq_dd_pd(gamma_internal);

    /* Return the desired position */
    return std::optional<Eigen::Vector3d>{dd_pd};
  }

  /* Return nothing if there is no path section */
  return std::nullopt;
}

std::optional<double> Path::tangent(double gamma) {

  double result;

  /* Get the path section corresponding to the gamma */
  PathSection* path_section = nullptr;
  double gamma_internal;

  std::tie (path_section, gamma_internal) = this->getPathSection(gamma);

  /* Check if there was a section in the path list */
  if(path_section){

    /* Get the tangent for that section */
    result = path_section->tangent(gamma_internal);

    /* Return the result */
    return std::optional<double>{result};
  }

  return std::nullopt;
}

std::optional<double> Path::curvature(double gamma) {

  double result;

  /* Get the path section corresponding to the gamma */
  PathSection* path_section = nullptr;
  double gamma_internal;

  std::tie (path_section, gamma_internal) = this->getPathSection(gamma);

  /* Check if there was a section in the path list */
  if(path_section){

    /* Get the curvature for that section */
    result = path_section->curvature(gamma_internal);

    /* Return the result */
    return std::optional<double>{result};
  }

  return std::nullopt;
}

std::optional<double> Path::derivative_norm(double gamma) {

  double result;

  /* Get the path section corresponding to the gamma */
  PathSection* path_section = nullptr;
  double gamma_internal;

  std::tie (path_section, gamma_internal) = this->getPathSection(gamma);

  /* Check if there was a section in the path list */
  if(path_section){

    /* Get the derivative_norm for that section */
    result = path_section->derivative_norm(gamma_internal);

    /* Return the result */
    return std::optional<double>{result};
  }

  return std::nullopt; 
}

/* Method to get the gamma corresponding to the closest point on the path
 * relative to the coordinate passed as argument 
 */
std::optional<double> Path::getClosestGamma(Eigen::Vector3d &coordinate) {
   
  /* Handle the special case where the path is still empty */
  if(this->isEmpty()) {
    return std::nullopt;
  }

  /* If we only have one section it is easy! We only need to check for the gamma
   * in that section that yields the closest point
   */
  if(this->type_of_path_ == SIMPLE_PATH) { 
    this->gamma_closest_point_ = this->sections_[0]->getClosestPointGamma(coordinate);
    return std::optional<double>{this->gamma_closest_point_};
  }

  /* If the path composed of multiple sections, first we must select the section 
   * and only then get the gamma for that particular section
   */
  if(this->type_of_path_ == COMPLEX_PATH) {
    
    /* Declare a set of auxiliary variables */
    unsigned int idx_section_smallest_error = 0;
    double gamma_of_smallest_error = 0.0;
    double smallest_error_distance = std::numeric_limits<double>::max();
    double error_distance = 0.0;
    
    /* If we are checking the closest point for the first time*/
    if(this->first_check_on_closest_point_) {
      first_check_on_closest_point_ = false;

      double gamma_normalized = 0.0;
      
      /* For each path section, get the gamma_closest */
      for(unsigned int i = 0; i < this->sections_.size(); i++) {
    
        /* Compute the normalized gamma corresponding to that particular section */
        gamma_normalized = this->sections_[i]->getClosestPointGamma(coordinate);

        //ROS_INFO("Index %d - First gamma norm: %lf", i, gamma_normalized);

        /* Compute the error distance for that particular section */
        error_distance = (this->sections_[i]->eq_pd(gamma_normalized) - coordinate).norm();

        /* If the distance error is smaller than in previous sections */
        if (error_distance < smallest_error_distance) {
          idx_section_smallest_error = i;
          gamma_of_smallest_error = gamma_normalized;
          smallest_error_distance = error_distance;
        }
      }
    
    /* If this is not the first iteration, we assume that the vehicle does not jump
     * around and moves continuously. Therefore we do not need to check in all the sections. Only
     * on the current section and the one imediatyly next! */
    } else {
     
      /* By default the index of the section is the same as the last one */
      idx_section_smallest_error = this->index_section_smallest_error_;

      /* Get the new gamma for the closest point in the same section as the previous iteration */
      gamma_of_smallest_error = this->sections_[idx_section_smallest_error]->getClosestPointGamma(coordinate);

      /* Compute the error distance for that particular section */
      error_distance = (this->sections_[idx_section_smallest_error]->eq_pd(gamma_of_smallest_error) - coordinate).norm();

      /* Look at the gamma normalized for the next section (if it exists) */
      if(idx_section_smallest_error < this->sections_.size()-1) {
         double test_next = this->sections_[idx_section_smallest_error + 1]->getClosestPointGamma(coordinate);
         double next_err_dist = (this->sections_[idx_section_smallest_error + 1]->eq_pd(test_next) - coordinate).norm();

        /* If the error is smaller in the next section, just switch */
        if(next_err_dist < error_distance) {
          idx_section_smallest_error = idx_section_smallest_error + 1;
          gamma_of_smallest_error = test_next;
        }
      }
    }

    /* Transform the gamma_normalized into the gamma of the complete path */
    std::pair<double, double> bounds = this->sections_limits_[idx_section_smallest_error];
    this->gamma_closest_point_ = bounds.first + gamma_of_smallest_error; 
    
    /* Save the auxiliary variables for the next iteration */
    this->index_section_smallest_error_ = idx_section_smallest_error; 
    return std::optional<double>{this->gamma_closest_point_};
  }

  /* Otherwise an unexpected case ocurred and we should not return a value */
  return std::nullopt;
 }

/** 
 * @brief  Method to return the minimum and maximum value that gamma can achive in the path 
 *
 * @return  A pair with 2 doubles with the first being the minimum value and the second the maximum value
 */
std::pair<double, double> Path::getMinMaxGamma() {
 
  /* Check if the path is empty, if so the minimum and maximum gamma is 0.0 by default */
  if(this->isEmpty()) return std::make_pair(0.0, 0.0);

  std::pair<double, double> lower_bound = this->sections_limits_.front();
  std::pair<double, double> upper_bound = this->sections_limits_.back();

  return std::make_pair(lower_bound.first, upper_bound.second);
}
