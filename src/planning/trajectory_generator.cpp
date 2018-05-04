#include "trajectory_generator.h"

#include <iostream>

#include "../Eigen-3.3/Eigen/Dense"
#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"

#include "driving_strategy.h"

/**
 * constructor
 */
TrajectoryGenerator::TrajectoryGenerator() {

}

/**
 * destructor
 */
TrajectoryGenerator::~TrajectoryGenerator() {

}

/**
 * jerk-minimized trajectory solver
 * @param start Init configuration
 * @param end Final configuration
 * @return JMT params
 */ 
TrajectoryGenerator::TrajectoryParams TrajectoryGenerator::generate_jerk_minimized_trajectory(
    const TrajectoryReference &start, 
    const TrajectoryReference &end
) {
    TrajectoryParams trajectory_params;

    trajectory_params.T = end.timestamp - start.timestamp;

    trajectory_params.s = solve_jerk_minimized_trajectory(
        start.s, start.vs, start.as,
          end.s,   end.vs,   end.as,
        trajectory_params.T
    );
    trajectory_params.d = solve_jerk_minimized_trajectory(
        start.d, start.vd, start.ad,
          end.d,   end.vd,   end.ad,
        trajectory_params.T
    );
    
    return trajectory_params;
}

/**
 * evaluate polynomial
 * 
 * @param t Evaluation point
 * @param coeffs Polynomial params
 * @param order Derivative order
 * @return function value
 */ 
double TrajectoryGenerator::evaluate_polynomial_derivative(double t, const std::vector<double> &coeffs, int order) {
    double result = 0.0;

    for (int N = coeffs.size() - 1; 0 <= N; --N) {
        if (N >= order) {
            double factor = 1.0;
            for (int dn = 0; dn < order; ++dn) {
                factor *= (N - dn);
            }
            result += factor * coeffs[N] * std::pow(t, N - order);
        } else {
            break;
        }
    }

    return result;
}

std::vector<double> TrajectoryGenerator::solve_jerk_minimized_trajectory(
    double x0, double vx0, double ax0,
    double x1, double vx1, double ax1,
    double T
) {
    // pre-computed exponentials of T:
    std::vector<double> T_exp{1.0, T, pow(T, 2.0), pow(T, 3.0),pow(T, 4.0), pow(T, 5.0)};
    
    // system matrix:
    Eigen::MatrixXd A(3, 3); 
    A <<   T_exp[3],    T_exp[4],    T_exp[5], 
         3*T_exp[2],  4*T_exp[3],  5*T_exp[4], 
         6*T_exp[1], 12*T_exp[2], 20*T_exp[3];
    
    // desired output:
    Eigen::VectorXd b(3);
    b <<  x1 - (x0 + vx0*T_exp[1] + 0.5*ax0*T_exp[2]),
         vx1 - (vx0 + ax0*T_exp[1]),
         ax1 - ax0;
    
    // polynomial coefficients:
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
   
    return {x0, vx0, 0.5*ax0, coeffs(0), coeffs(1), coeffs(2)};
}