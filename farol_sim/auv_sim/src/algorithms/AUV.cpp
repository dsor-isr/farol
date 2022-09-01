#include "AUV.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <ros/ros.h> //TODO: remove this header - only used for debugging

AUV::AUV(double mass,
          double fluid_density,
          double zg,
          double vehicle_density,
          const Eigen::Vector3d &inertia_tensor,
          const Eigen::Matrix<double, 6, 1> &linear_damping_tensor,
          const Eigen::Matrix<double, 6, 1> &quadratic_damping_tensor,
          const Eigen::Matrix<double, 6, 1> &added_mass_tensor,
          const Eigen::MatrixXd &allocation_matrix,
          const Eigen::Vector3d &lump_param_positive,
          const Eigen::Vector3d &lump_param_negative,
          const Eigen::Vector2d &min_max_thruster_input,
          double thrusters_gain,
          double thrusters_pole,
          double thrusters_delay,
          double sampling_period,
          const Eigen::Vector3d &disturbance_mean,
          const Eigen::Vector3d &disturbance_sigma,
          const Eigen::Vector3d &disturbance_minimum,
          const Eigen::Vector3d &disturbance_maximum): mass_(mass), fluid_density_(fluid_density), zg_(zg),
                lump_param_positive_(lump_param_positive),
                lump_param_negative_(lump_param_negative),
                min_max_thruster_input_(min_max_thruster_input),
                thrusters_gain_(thrusters_gain),
                thrusters_pole_(thrusters_pole),
                thrusters_delay_(thrusters_delay),
                sampling_period_(sampling_period),
                disturbance_minimum_(disturbance_minimum),
                disturbance_maximum_(disturbance_maximum) {

    /* Compute the radius of a sphere with equivalent density to the vehicle */
    this->radius_ = std::pow(mass/((3.0/4.0)*M_PI*vehicle_density), 1.0/3.0);

    /* Create a 6x6 diagonal rigid body mass matrix */
    Eigen::Matrix<double, 6, 6> mass_matrix = Eigen::Matrix<double, 6, 6>::Zero(6, 6);
    for(int i=0; i < 3; i++) mass_matrix(i, i) = mass;
    for(int i=3; i < 6; i++) mass_matrix(i, i) = inertia_tensor(i-3);

    /* Create a 6x6 diagonal added mass matrix */
    Eigen::Matrix<double, 6, 6> added_mass_matrix;
    added_mass_matrix = added_mass_tensor.asDiagonal();

    /* Compute the 6x6 diagonal total mass matrix */
    this->M_ = mass_matrix + added_mass_matrix;
    this->M_inv_ = this->M_.inverse();
    this->M11_ = this->M_.block(0, 0, 3, 3);
    this->M12_ = this->M_.block(0, 3, 3, 3);
    this->M21_ = this->M_.block(3, 0, 3, 3);
    this->M22_ = this->M_.block(3, 3, 3, 3);

    /* Create a 6x6 diagonal linear damping matrix */
    this->Dl_ = linear_damping_tensor.asDiagonal();

    /* Create a 6x6 diagonal quadratic damping matrix */
    this->Dq_ = quadratic_damping_tensor.asDiagonal();

    /* Save the allocation matrix */
    this->allocation_matrix_ = allocation_matrix;

    /* Initialize the thrusters */
    this->number_of_delays_inputs_ = int(std::round(this->thrusters_delay_ / this->sampling_period_));

    /* Initialize the thrusters input vectors with zeros (used as circular buffer) */
    double number_thrusters = this->allocation_matrix_.rows();
    for(int i=0; i < this->number_of_delays_inputs_; i++) {
        /* Save a vector full of zeros with the size corresponding to the number of thrusters */
        this->thrusters_inputs_.emplace_back(Eigen::VectorXd::Zero(number_thrusters));
    }

    /* Initialize the thrusters last output vector with zeros */
    this->last_thruster_output_ = Eigen::VectorXd::Zero(number_thrusters);

    /* Initiate the ocean disturbances random process distributions */
    this->generator_ = std::default_random_engine();
    for(int i=0; i < 3; i++) distributions_[i] = std::normal_distribution<double>(disturbance_mean(i), disturbance_sigma(i));
}

void AUV::update(double dt, const Eigen::VectorXd &thrust) {

    /* Verify if the size of the thrust vector corresponds to the number of thrusters of the vehicle */
    if(thrust.size() != this->getNumberThrusters()) throw std::invalid_argument("Thrust vector size should be " + std::to_string(this->getNumberThrusters()));

    /* Verify that dt (time difference is positive) */
    if(dt < 0) throw std::invalid_argument("dt parameter should be positive");

    /* Propagate the desired thrust through the thruster model */
    Eigen::VectorXd applied_thrust;
    applied_thrust = this->applyThrustersModel(thrust);

    /* Convert the applied thrust in the motors to forces and torques applied to the rigid body */
    Eigen::Matrix<double, 6, 1> applied_forces_and_torques;
    applied_forces_and_torques = this->convertThrustToGeneralForces(applied_thrust);

    /* Compute the ocean disturbances */
    Eigen::Vector3d ocean_disturbances;
    ocean_disturbances = this->computeOceanDisturbances();

    /* Compute the dynamics of the rigid body - linear and angular acceleration in body frame */
    Eigen::Vector3d v1_dot, v2_dot;
    std::tie(v1_dot, v2_dot) = this->updateDynamics(applied_forces_and_torques);

    /* Integrate the dynamics */
    Eigen::Vector3d v1(this->state_.v1), v2(this->state_.v2);
    v1 = eulerIntegration(dt, v1, v1_dot);
    v2 = eulerIntegration(dt, v2, v2_dot);

    /* Compute the kinematics of the rigid body - linear angular velocity in inertial frame */
    Eigen::Vector3d eta1_dot, eta2_dot;
    std::tie(eta1_dot, eta2_dot) = this->updateKinematics(v1, v2, ocean_disturbances);

    /* Integrate the kinematics */
    Eigen::Vector3d eta1(this->state_.eta1), eta2(this->state_.eta2);
    eta1 = eulerIntegration(dt, eta1, eta1_dot);
    eta2 = eulerIntegration(dt, eta2, eta2_dot);

    /* Wrap angles */
    for(int i=0; i<3; i++) eta2(i) = wrapAngle(eta2(i));

    /* Update the state of the vehicle */
    this->state_.eta1 = eta1;
    this->state_.eta2 = eta2;
    this->state_.v1 = v1;
    this->state_.v2 = v2;
}

Eigen::MatrixXd AUV::applyThrustersModel(const Eigen::VectorXd &thrust) {

    /* Create a vector of input thrust */
    Eigen::VectorXd input_thrust = Eigen::VectorXd(thrust.size());

    /* For each thruster input */
    for(int i=0; i < thrust.size(); i++) {

        /* Saturate the values that go beyond the maximum values */
        input_thrust(i) = saturation(thrust(i), -this->min_max_thruster_input_(1), this->min_max_thruster_input_(1));

        /* Apply a dead-zone */
        if (input_thrust(i) < this->min_max_thruster_input_(0) && input_thrust(i) > -this->min_max_thruster_input_(0)) input_thrust(i) = 0.0;
    }

    /* Rotate the circular buffer of inputs (rotate to the left), and save the new input in the last element  */
    std::rotate(this->thrusters_inputs_.begin(), this->thrusters_inputs_.begin() + 1, this->thrusters_inputs_.end());
    this->thrusters_inputs_.back() = input_thrust;

    /* --- Apply the thrusters (simple-pole + delay) discrete model --- */
    /* load from the thruster buffer the desired inputs, such that: input = u[k-delay] */
    Eigen::VectorXd u_old = this->thrusters_inputs_[0];

    /* Calculate the applied y[k] using the difference equations (discretized model of simple-pole motor + delay) */
    /* y(k+1) = e^{-pole * period} * y(k) + (1-e^{-pole * period}) * u(k-number_of_delays) */
    Eigen::VectorXd y_k1, y_k;
    y_k = this->last_thruster_output_;
    y_k1 = (std::exp(-this->thrusters_pole_ * this->sampling_period_) * y_k) + ((-std::exp(-this->thrusters_pole_ * this->sampling_period_) + 1) * u_old);

    /* Saturate the output between the minimum and maximum values */
    for(int i=0; i < thrust.size(); i++) y_k1(i) = saturation(y_k1(i), -this->min_max_thruster_input_(1), this->min_max_thruster_input_(1));

    /* Save the new output to used in the next iteration */
    this->last_thruster_output_ = y_k1;

    /* Apply the actuators gain (to scale from 0-100 to some other unit, for example RPM) */
    y_k1 *= this->thrusters_gain_;

    /* Convert the output values to forces [N], using the parable curves */
    Eigen::VectorXd output_force = y_k1;
    for(int i=0; i < y_k1.size(); i++) output_force[i] = this->convertThrustToForce(y_k1[i]);

    /* Return the output force applied by each thruster */
    return output_force;
}

double AUV::convertThrustToForce(const double thrust) {

    double a, b, c;
    /* Choose the constants for the parable depending if we are on the positive or negative side of the thrust curve */
    if(thrust >= 0) {
        a = this->lump_param_positive_[0];
        b = this->lump_param_positive_[1];
        c = this->lump_param_positive_[2];
    } else {
        a = this->lump_param_negative_[0];
        b = this->lump_param_negative_[1];
        c = this->lump_param_negative_[2];
    }

    /* Apply the parable formula F = a|x|^2 + b|x| + c */
    return (a * std::pow(thrust, 2)) + (b * std::abs(thrust)) + c;
}

Eigen::Matrix<double, 6, 1> AUV::convertThrustToGeneralForces(const Eigen::VectorXd &thrust) {

    /* Inititate the matrix with all forces and torques with contributions from all vehicles */
    Eigen::Matrix<double, 6, 1> forces_and_torques = Eigen::Matrix<double, 6, 1>::Zero();

    /* Initiate the vector of forces */
    Eigen::Vector3d total_force;
    Eigen::Vector3d total_torque;

    /* Compute the contribution of the force applied by each thruster to the total force */
    for(unsigned int i=0; i < this->getNumberThrusters(); i++) {

        /* Get the contribution of the thruster to the force in Fx, Fy and Fz */
        this->allocation_matrix_.block<1,3>(i,0);

        /* Compute the total force applied in each direction by thruster "i" */
        total_force = thrust[i] * this->allocation_matrix_.block<1,3>(i,0);

        /* Compute the total torque applied about each axis by thruster "i" */
        total_torque = this->allocation_matrix_.block<1,3>(i,3).cross(total_force);

        /* Accumulate the forces and torques from all the thrusters */
        forces_and_torques.block<3,1>(0, 0) += total_force;
        forces_and_torques.block<3,1>(3, 0) += total_torque;
    }

    return forces_and_torques;
}

Eigen::Vector3d AUV::computeOceanDisturbances() {

    Eigen::Vector3d ocean_disturbances = Eigen::Vector3d(0.0, 0.0, 0.0);

    /* Generate random numbers according to the desired mean and standard deviation for x, y and z axis */
    for(int i=0; i<3; i++) {

        // Generate the random number according to the gaussian distribution
        ocean_disturbances(i) = distributions_[i](generator_);

        // Bound the disturbances with limits
        if(ocean_disturbances(i) < disturbance_minimum_(i)) ocean_disturbances(i) = disturbance_minimum_(i);
        if(ocean_disturbances(i) > disturbance_maximum_(i)) ocean_disturbances(i) = disturbance_maximum_(i);
    }

    return ocean_disturbances;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> AUV::updateDynamics(const Eigen::Matrix<double, 6, 1> &forces_and_torques) {

    Eigen::Vector3d v1_dot, v2_dot;

    /* Compute the Coriolis Matrix (6x6) */
    Eigen::Matrix<double, 6, 6> coriolis;
    coriolis = this->computeCoriolisMatrix();

    /* Compute the Damping Matrix (6x6) */
    Eigen::Matrix<double, 6, 6> damping;
    damping = this->computeDampingTerms();

    /* Compute Gravitational Forces Vector (6x1) */
    Eigen::Matrix<double, 6, 1> hydrostatic_forces;
    hydrostatic_forces = this->computeHydrostaticForces();

    /* Compute the updated v_dot */
    Eigen::Matrix<double, 6, 1> v, v_dot;
    v << this->state_.v1, this->state_.v2;
    // Note: Damping terms already contain the (-) sign, therefore we should sum here
    v_dot = this->M_inv_ * (forces_and_torques - hydrostatic_forces - (coriolis * v) + (damping * v));

    /* Get the individual linear and angular velocity terms and return them */
    v1_dot << v_dot(0), v_dot(1), v_dot(2);
    v2_dot << v_dot(3), v_dot(4), v_dot(5);

    return std::make_tuple(v1_dot, v2_dot);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> AUV::updateKinematics(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &ocean_disturbances) {

    Eigen::Vector3d eta1_dot, eta2_dot;

    /* Compute the linear velocity in the inertial frame */
    eta1_dot = (rotationBodyToInertial(this->state_.eta2(0), this->state_.eta2(1), this->state_.eta2(2)) * v1) + ocean_disturbances;

    /* Compute the angular velocity in the inertial frame */
    eta2_dot = rotationAngularBodyToInertial(this->state_.eta2(0), this->state_.eta2(1), this->state_.eta2(2)) * v2;

    return std::make_tuple(eta1_dot, eta2_dot);
}

Eigen::Matrix<double, 6, 6> AUV::computeCoriolisMatrix() {

    Eigen::Matrix<double, 6, 6> coriolis = Eigen::MatrixXd::Zero(6, 6);

    /* Compute the coriolis terms from the mass matrix and the velocity vector in the body frame */
    Eigen::Matrix3d s1, s2;
    s1 = computeSkewSymmetric((this->M11_ * this->state_.v1) + (this->M12_ * this->state_.v2));
    s2 = computeSkewSymmetric((this->M21_ * this->state_.v1) + (this->M22_ * this->state_.v2));

    coriolis.block(0, 3, 3, 3) = -s1;
    coriolis.block(3, 0, 3, 3) = -s1;
    coriolis.block(3, 3, 3, 3) = -s2;

    return coriolis;
}

Eigen::Matrix<double, 6, 6> AUV::computeDampingTerms() {

    Eigen::Matrix<double, 6, 6> damping, quadratic_damping;

    /* Get the state vector corresponding to linear and angular velocities in the Body frame */
    Eigen::Matrix<double, 6, 1> v, quadratic_damping_vector;
    v << this->state_.v1, this->state_.v2;

    /* Compute the damping matrix according to the formulas */
    quadratic_damping_vector = this->Dq_ * v.cwiseAbs();
    quadratic_damping = quadratic_damping_vector.asDiagonal();
    damping = this->Dl_ + quadratic_damping;

    return damping;
}

Eigen::Matrix<double, 6, 1> AUV::computeHydrostaticForces() {

    Eigen::Matrix<double, 6, 1> hydrostatic_forces;

    /* Compute the Weight force applied to the vehicle in the inertial frame */
    double W = this->mass_ * this->gravity_;

    /* Compute the volume of fluid displaced */
    double volume_fluid_displaced = this->computeVolumeFluidDisplaced();

    /* Compute the buoyancy force applied to the vehicle in the inertial frame */
    double B = this->fluid_density_ * this->gravity_ * volume_fluid_displaced;

    /* Compute the hydrostatic forces vector */
    /* NOTE: an assumption is made that xg and yg (center x and y center of gravity of the robot are 0.0) which is not
     * necessarily true. This model can be improved in the future. See Fossen's handbook to marine crafts */
    hydrostatic_forces << (W - B) * sin(this->state_.eta2(1)),
                     -(W - B) * cos(this->state_.eta2(1)) * sin(this->state_.eta2(0)),
                     -(W - B) * cos(this->state_.eta2(1)) * cos(this->state_.eta2(0)),
                      this->zg_ * W * cos(this->state_.eta2(1)) * sin(this->state_.eta2(0)),
                      this->zg_ * W * sin(this->state_.eta2(1)),
                      0.0;

    return hydrostatic_forces;
}

double AUV::computeVolumeFluidDisplaced() {

    double r = this->radius_;

    /* Assume that the vehicle is a sphere that decreases its radius when the center of mass of the vehicle is above the surface of water */
    /* NOTE: this is not the best model and should be improved in future iterations */
    if(this->state_.eta1(2) < 0.0) {
        r = this->radius_ + this->state_.eta1(2);
        if(r < 0.0) r = 0;
    }

    /* Return the volume of the sphere (which corresponds to the amount of fluid displaced by itself */
    return (4.0 / 3.0) * M_PI * std::pow(r, 3);
}