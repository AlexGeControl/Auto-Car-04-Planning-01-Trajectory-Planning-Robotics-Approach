#include "vehicle.h"
#include <iostream>
#include <cmath>
#include "../map/map.h"

/**
 * constructor
 */
EgoVehicle::EgoVehicle() {
    // init as start time:
    t = 0.0;
    // init as empty trajectory:
    trajectory_params = TrajectoryGenerator::TrajectoryParams{0.0, {}, {}};
};

/**
 * destructor
 */
EgoVehicle::~EgoVehicle() {

};

/**
 * is engine starting
 * @return true is ego vehicle starting false otherwise
 */
bool EgoVehicle::is_starting() { 
    // if current trajectory is empty:
    return 0 == trajectory_params.s.size(); 
}

/**
 * sync ego vehicle time with system using previous trajectory
 * 
 * @param trajectory Previous trajectory
 */
void EgoVehicle::sync_time(const TrajectoryGenerator::Trajectory &previous_trajectory) {
    t = trajectory_params.T - STEPSIZE * previous_trajectory.N;
}

/**
 * get ego vehicle current trajectory reference
 * 
 * @return Ego vehicle current trajectory reference
 */
TrajectoryGenerator::TrajectoryReference EgoVehicle::get_trajectory_reference() {
    TrajectoryGenerator::TrajectoryReference trajectory_reference;

    // reset time:
    trajectory_reference.timestamp = 0.0;
    
    std::vector<double> p_frenet, v_frenet, a_frenet;
    if (is_starting()) {
        p_frenet = {   s,   d};
        v_frenet = { 0.0, 0.0};
        a_frenet = { 0.0, 0.0};
    } else {
        double s = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.s, 0);
        double vs = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.s, 1);
        double as = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.s, 2);

        double d = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.d, 0);
        double vd = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.d, 1);
        double ad = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.d, 2);

        p_frenet = {   s,   d};
        v_frenet = {  vs,  vd};
        a_frenet = {  as,  ad};
    }

    // frenet s:
    trajectory_reference.s = p_frenet[0];
    trajectory_reference.vs = v_frenet[0];
    trajectory_reference.as = a_frenet[0];
    // frenet d:
    trajectory_reference.d = p_frenet[1];
    trajectory_reference.vd = v_frenet[1];
    trajectory_reference.ad = a_frenet[1];

    return trajectory_reference;
}

/**
 * set ego vehicle trajectory
 * 
 * @param trajectory_params: Parameterized ego vehicle trajectory
 */
void EgoVehicle::set_trajectory(const TrajectoryGenerator::TrajectoryParams &trajectory_params) {
    this->trajectory_params = trajectory_params;
}