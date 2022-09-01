#pragma once

#include <Eigen/Core>
#include <random>
#include <vector>
#include <tuple>

#include <State.h>

/**
 * @brief     AUV class - implements a Matlab-like simulation of an AUV in C++
 * @author    Marcelo Jacinto
 * @version   1.0.0
 * @date      2021/11/12
 * @copyright MIT
 */
class AUV {

public:

    /**
     * FarolAUV class constructor
     * @param mass The mass of the vehicle in Kg
     * @param fluid_density The density of the fluid the vehicle is in (a.k.a water density)
     * @param zg The center of gravity of the (sphere-like) vehicle
     * @param vehicle_density The density of the vehicle (Kg/m^3)
     * @param inertia_tensor A vector of 3 elements with the diagonal of the inertia matrix
     * @param linear_damping_tensor A vector of 6 elements with the diagonal of the linear damping matrix
     * @param quadratic_damping_tensor A vector of 6 elements with the diagonal of the quadratic damping matrix
     * @param added_mass_tensor A vector of 6 elements with the diagonal of the added mass matrix
     * @param allocation_matrix A matrix with the contributions of each thruster to the forces in X,Y and Z and the arms
     * for computing the moments of inertia later [Fx, Fy, Fz, lx, ly, lz] (each line represents one thruster)
     * @param lump_param_positive The thrust curve parameters for the right side of the curve
     * @param lump_param_negative The thrust curve parameters for the left side of the curve
     * @param min_max_thruster_input The minimum and maximum normalized thruster inputs [min_input, max_input]^T
     * @param thrusters_gain
     * @param thrusters_pole
     * @param thrusters_delay
     * @param sampling_period An approximated sampling period (s) at which the simulation will run (NOTE: this is needed to discretize the
     * thrusters model only and the integration of the dynamics of the vehicle will use the dt variable provided through the update
     * method)
     * @param disturbance_mean A vector with the mean of the ocean disturbances (gaussian process)
     * @param disturbance_sigma A vector with the standard deviation of the ocean disturbances (gaussian process)
     * @param disturbance_minimum A vector with the minimum values for the ocean disturbances
     * @param disturbance_maximum A vector with the maximum values for the ocean disturbances
     */
    AUV(double mass,
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
              const Eigen::Vector3d &disturbance_maximum);

    /**
     * Method to update the state of the vehicle given the thrust applied to each individual thruster.
     * This method verifies if (dt >= 0) and the size of thrust vector is the same as the number of thrusters of the model.
     * If some of these conditions does not verify, a std::invalid_argument exception is thrown
     *
     * @param dt The time difference (in seconds) between the last function call and the disturbance function call
     * @param thrust A vector of n elements with the thrust to apply to each of the n vehicle thursters (normalized between 0 and 1)
     */
    void update(double dt, const Eigen::VectorXd &thrust);

    /**
     * Method that returns a copy of the disturbance state of the vehicle
     * @return A state object with the state of the vehicle
     */
    inline State getState() {
        return this->state_;
    };

    /**
     * Method that sets the disturbance state of the vehicle to a new pre-defined state
     * @param state A state reference that contains the desired state of the vehicle
     */
    inline void setState(const State &state) {
        this->state_ = state;
    };

    /**
     * Method that returns the number of thrusters of the AUV based on the number of lines of the allocation matrix
     * received by the constructor upon object construction
     * @return The number of thrusters of the AUV
     */
    inline unsigned int getNumberThrusters(){
        return this->allocation_matrix_.rows();
    };

private:

    /**
     *
     * @param thrust Method that given the desired thrust input to apply to each thruster (between -100<->100), computes
     * the actually applied thrust in (N). This model saturates the input between -100 and 100, applies a dead-zone,
     * applies a discrete model of a DC-motor (simple pole+delay) using the differences equation:
     * y(k+1) = e^{-pole * period} * y(k) + (1-e^{-pole * period}) * u(k-number_of_delays)
     * After this step, the output y(k+1) is multiplied by a gain (used to convert between units, for example, from 100 to 4500 RPM)
     * Then it call the auxiliar method: convertThrustToForce() to convert the output: gain*y(k+1) to forces expressed
     * in Newton.
     * @return The applied thrust in (N)
     */
    Eigen::MatrixXd applyThrustersModel(const Eigen::VectorXd &thrust);

    /**
     * Takes double with the thrust (in some normalized unit) and converts it to Newtons according to a parabole thrust curve
     * using the equation:
     * force (N) = a*{x}^2 + b*x +c
     * Note: this equations uses the positive_lump_parameters for the right side of the curve and
     * the negative_lump_parameters for the left side of the curve
     * @param thrust The total thrust input (applied by 1 thruster) in some general unit
     * @return The total thrust that corresponds to that input but in Newtons (N)
     */
    double convertThrustToForce(const double thrust);

    /**
     * Takes the force in Newtons (N) applied by each thruster and makes use of the allocation matrix
     * to discover the total net-force applied in X, Y and Z axis of the vehicle (N) and well as the total net-torques
     * applied about X, Y and Z axis (Nm).
     * @param thrust The total thrust applied by each thruster in Newtons (N)
     * @return A vector with [Fx, Fy, Fz, Mx, My, Mz]^T corresponding to the total force and torque applied to the vehicle
     */
    Eigen::Matrix<double, 6, 1> convertThrustToGeneralForces(const Eigen::VectorXd &thrust);

    /**
     * Method that generates random non-rotational ocean disturbances based on a gaussian process
     * @return A vector of 3 elements with the disturbance in X, Y and Z axis
     */
    Eigen::Vector3d computeOceanDisturbances();

    /**
     * Method that updates the Dynamics of the AUV according to Fossen's book for marine crafts
     * @param forces_and_torques A vector of 6 elements with the generalized forces and torques applied in the body
     * frame of the vehicle [Fx, Fy, Fz, Mx, My, Mz]^T
     * @return A tuple with 2 vectors:
     *  v1_dot - linear acceleration in the body frame [u_dot, v_dor, w_dot]^T
     *  v2_dot - angular acceleration in the body frame [p_dot, q_dot, r_dot]^T
     */
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> updateDynamics(const Eigen::Matrix<double, 6, 1> &forces_and_torques);

    /**
     * Method that updates the Kinematics of the vehicle
     * @param v1 A vector of 3 elements with the linear velocity in the body frame [u, v, w]^T
     * @param v2 A vector of 3 elements with the angular velocity in the body frame [p, q, r]^T
     * @param ocean_disturbances A vector of 3 elements with the ocean disturbances in X, Y and Z axis
     * @return A tuple with 2 vectors:
     *  eta1_dot - the linear velocity in the inertial frame [x_dot, y_dot, z_dot]^T
     *  eta2_dot - the angular velocity in the inertial frame [phi_dot, theta_dot, psi_dot]^T
     */
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> updateKinematics(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &ocean_disturbances);

    /**
     * Method that computes the total Coriolis matrix from the mass and added mass matrices and velocity vectors
     * @return A 6x6 Coriolis Matrix
     */
    Eigen::Matrix<double, 6, 6> computeCoriolisMatrix();

    /**
     * Method that computes the total Damping matrix from the linear damping, quadratic damping and velocity vectors
     * @return A 6x6 Damping Matrix
     */
    Eigen::Matrix<double, 6, 6> computeDampingTerms();

    /**
     * Method that computes the total forces applied by gravity and buoyancy
     * @return A 6x1 vector with the forces and torques applied in each direction by gravity and buoyancy
     */
    Eigen::Matrix<double, 6, 1> computeHydrostaticForces();

    /**
     * Method that computes the amount of fluid displaced by a sphere that reduces its radius when above the surface
     * NOTE: This model is not very accurate and should be improved in the future
     * @return A double with the volume of fluid displaced (m^3)
     */
    double computeVolumeFluidDisplaced();

    /* State of the vehicle (position, attitude, body linear velocity and body angular velocity) */
    State state_;

    /* Gravity acceleration */
    double gravity_{9.8};

    /* Mass of the vehicle */
    double mass_;

    /* Density of the fluid the vehicle is in (a.k.a water density) */
    double fluid_density_;

    /* Z-coordinate of the center of gravity of the sphere-like vehicle */
    double zg_;

    /* Radius of the sphere-like vehicle */
    double radius_;

    /* Auxiliar Mass (total Ma+Mc), linear damping and quadratic damping matrices*/
    Eigen::Matrix<double, 6, 6> M_, M_inv_;
    Eigen::Matrix<double, 6, 6> Dl_, Dq_;

    /* Break the Mass matrix (M) into 4 blocks (to use in coriolis matrix computation efficiently) */
    Eigen::Matrix3d M11_, M12_, M21_, M22_;

    /* Thrusters parameters */
    Eigen::MatrixXd allocation_matrix_;
    Eigen::Vector3d lump_param_positive_, lump_param_negative_;
    Eigen::Vector2d min_max_thruster_input_;
    double thrusters_gain_, thrusters_pole_, thrusters_delay_, sampling_period_;

    /* Thrusters discrete input vector for each thruster in previous times */
    std::vector<Eigen::VectorXd> previous_inputs_;

    /* Thrusters number of delays in discrete units, circular buffer with the input of the thrusters in each discrete unit time-step, the last output of the thruster */
    int number_of_delays_inputs_;
    std::vector<Eigen::VectorXd> thrusters_inputs_;
    Eigen::VectorXd last_thruster_output_;

    /* Ocean disturbances as a gaussian process */
    Eigen::Vector3d disturbance_minimum_;
    Eigen::Vector3d disturbance_maximum_;
    std::default_random_engine generator_;
    std::vector<std::normal_distribution<double>> distributions_{3};
};
