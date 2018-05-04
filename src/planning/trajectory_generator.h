#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>

class TrajectoryGenerator {
public:
    /**
     * trajectory reference
     */
    typedef struct {
        double timestamp;
        
        double s;
        double vs;
        double as;

        double d;
        double vd;
        double ad;
    } TrajectoryReference;

    /**
     * output trajectory
     */
    typedef struct {
        double T;
        std::vector<double> s;
        std::vector<double> d;
    } TrajectoryParams;

    typedef struct {
        int N;
        std::vector<double> x;
        std::vector<double> y;
    } Trajectory;
    
    /**
     * constructor
     */
    TrajectoryGenerator();

    /**
     * destructor
     */
    virtual ~TrajectoryGenerator();

    /**
     * jerk-minimized trajectory solver
     */ 
    TrajectoryParams generate_jerk_minimized_trajectory(
        const TrajectoryReference &start, 
        const TrajectoryReference &end
    );

    static double evaluate_polynomial_derivative(double t, const std::vector<double> &coeffs, int order);
private:
    std::vector<double> solve_jerk_minimized_trajectory(
        double x0, double vx0, double ax0,
        double x1, double vx1, double ax1,
        double T
    );
};

#endif /* TRAJECTORY_GENERATOR_H */