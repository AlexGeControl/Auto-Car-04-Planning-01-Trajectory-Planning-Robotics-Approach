#include "driving_strategy.h"

#include <iostream>
#include <cmath>

#include "../map/map.h"

namespace DrivingStrategy {
    /**
        calculate safety distance.

        @param velocity: vehicle velocity
        @return selected safety distance
    */
    double Highway::get_safety_distance(double velocity) {
        return 2.0 * VEHICLE_LENGTH + (velocity * RESPONSE_TIME + 0.5 * std::pow(velocity, 2.0) / MAX_ACCELERATION);
    }

    /**
     *  whether the trajectory is feasible
     * 
     *  @param trajectory_params 
     *  @param stepsize
     *  @return true if feasible false otherwise
    */ 
    bool Highway::is_feasible_trajectory(const TrajectoryGenerator::TrajectoryParams &trajectory_params, double stepsize) {
        // generate trajectory for actuator:
        for (double t = 0.0; t < trajectory_params.T; t += stepsize) {
            double vs = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.s, 1);
            double as = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.s, 2);
            double js = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.s, 3);

            double vd = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.d, 1);
            double ad = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.d, 2);
            double jd = TrajectoryGenerator::evaluate_polynomial_derivative(t, trajectory_params.d, 3);

            double v = std::sqrt(vs*vs + vd*vd);
            double a = std::sqrt(as*as + ad*ad);
            double j = std::sqrt(js*js + jd*jd);

            // violate dynamic constraints -- infeasible
            if (!is_feasible_velocity(v) || !is_feasible_acceleration(a)) {
                /*
                std::cout << "[Feasibility Violation]: " << v << ", " << a << ", " << j << std::endl;
                */
                return false;
            }
        }

        return true;
    }

    /**
     *  whether the trajectory is feasible
     * 
     *  @param trajectory 
     *  @param stepsize
     *  @return true if feasible false otherwise
    */ 
    bool Highway::is_feasible_trajectory(const TrajectoryGenerator::Trajectory &trajectory, double stepsize) {
        for (int i = 0; i < trajectory.N - 3; ++i) {
            double vx = get_velocity_forward_difference(i, trajectory.x, stepsize);
            double vy = get_velocity_forward_difference(i, trajectory.y, stepsize);
            double ax = get_acceleration_forward_difference(i, trajectory.x, stepsize);
            double ay = get_acceleration_forward_difference(i, trajectory.y, stepsize);
            double jx = get_jerk_forward_difference(i, trajectory.x, stepsize);
            double jy = get_jerk_forward_difference(i, trajectory.y, stepsize);

            double v = std::sqrt(vx*vx + vy*vy);
            double a = std::sqrt(ax*ax + ay*ay);
            double j = std::sqrt(jx*jx + jy*jy);

            // violate dynamic constraints -- infeasible
            if (!is_feasible_velocity(v)) {
                // std::cout << "[Feasibility Violation]: " << v << ", " << a << ", " << j << " @ " << i << "--" << trajectory.x[i] << ", " << trajectory.y[i] << std::endl;
                return false;
            }
        }

        return true;
    }

    /**
        calculate flexibity cost
    */
    Highway::FlexibilityCost Highway::get_flexibility_cost(const TrajectoryGenerator::TrajectoryReference &end, const LaneFeasibleZone &zone) {
        // init:
        FlexibilityCost cost{
            0.0,
            0.0,
            0.0,
            0.0,

            0.0
        };

        // lane selection cost:
        double ratio_flexibility = ((end.d == 6.0) ? +1.0 : -1.0);
        cost.flexibility = logistic(ratio_flexibility);
        // available space cost:
        double ratio_space = (zone.s_leading_min - zone.s_following_max) / (zone.s_upper - zone.s_lower);
        if (ratio_space > 1.0) {
            ratio_space = std::pow(ratio_space, 2.0) - 1.0;
        } else {
            ratio_space = -5.0;
        }
        cost.space = 8.0 * logistic(ratio_space);
        // leading vehicle cost:
        double ratio_leading = (zone.is_with_leading_vehicle ? -4.0 : +4.0);
        cost.leading = 4.0 * logistic(ratio_leading);
        // following vehicle cost:
        double ratio_following = (zone.is_with_following_vehicle ? -2.0 : +2.0);
        cost.following = 2.0 * logistic(ratio_following);

        cost.total = cost.flexibility + cost.space + cost.leading;

        return cost;
    }

    /**
     *  calculate efficiency cost
     */
    Highway::EfficiencyCost Highway::get_efficiency_cost(
        const TrajectoryGenerator::TrajectoryReference &start,
        const TrajectoryGenerator::TrajectoryReference &end, const LaneFeasibleZone &zone
    ) {
        // init:
        EfficiencyCost cost{
            0.0,
            0.0,
            0.0,
            0.0,

            0.0
        };

        // s covered:
        double ratio_s_reached = ((end.s - zone.s_upper) / (zone.s_upper - zone.s_lower)) + 0.5;
        cost.s_reached = 2.0 * logistic(ratio_s_reached);
        // vs covered:
        double ratio_vs_reached = ((end.vs - zone.vs_upper) / (zone.vs_upper - zone.vs_lower)) + 0.5;
        cost.vs_reached = 8.0 * logistic(ratio_vs_reached);
        // max speed reached:
        double ratio_max_speed_reached = (end.vs - DrivingStrategy::MAX_VELOCITY) / (1.0 * 0.44704);
        cost.max_speed_reached = 4.0 * logistic(ratio_max_speed_reached);
        // lane change cost:
        double ratio_delta_d = ((end.d == start.d) ? +1.0 : -1.0);;
        cost.delta_d = 1.0 * logistic(ratio_delta_d);

        cost.total = cost.s_reached + cost.vs_reached + cost.max_speed_reached + cost.delta_d;

        return cost;
    }
    /**
        calculate logistic function.

        @param x
        @return logistic(x)
    */
    double Highway::logistic(double x) { return 1.0 - 2.0/(1 + std::exp(-x)); }

    /**
        feasibility specifications.
    */
    double Highway::get_velocity_forward_difference(int i, const std::vector<double> &x, double stepsize) {
        return (x[i + 1] - x[i]) / stepsize;
    }
    double Highway::get_acceleration_forward_difference(int i, const std::vector<double> &x, double stepsize) {
        return (x[i + 2] - 2 * x[i + 1] + x[i]) / std::pow(stepsize, 2.0);
    }
    double Highway::get_jerk_forward_difference(int i, const std::vector<double> &x, double stepsize) {
        return (x[i + 3] - 3 * x[i + 2] + 3 * x[i + 1] - x[i]) / std::pow(stepsize, 3.0);
    }

    bool Highway::is_feasible_velocity(double v) { return v < MAX_VELOCITY; }
    bool Highway::is_feasible_acceleration(double a) { return a < 0.8 * MAX_ACCELERATION; }
    bool Highway::is_feasible_jerk(double j) { return j < MAX_JERK; }
}