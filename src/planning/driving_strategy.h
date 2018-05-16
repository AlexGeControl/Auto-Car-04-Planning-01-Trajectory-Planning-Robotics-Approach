#ifndef DRIVING_STRATEGY_H
#define DRIVING_STRATEGY_H

#include "trajectory_generator.h"

namespace DrivingStrategy {
    /**
     * dimensions
     */
    const double VEHICLE_LENGTH = 4.0;
    /**
     * dynamics
     */
    const double MIN_VELOCITY = 40.0 * 0.44704;
    const double MAX_VELOCITY = 49.0 * 0.44704;
    const double VELOCITY_DEVIATION = 10.0 * 0.44704;
    const double MAX_ACCELERATION = 10.0;
    const double MAX_JERK = 10.0;

    /**
     * safety 
     */
    const double RESPONSE_TIME = 1.0;
    
    class Highway {
    public:
        /**
         * lane feasible zone
         */
        typedef struct {
            bool is_with_leading_vehicle;
            bool is_with_following_vehicle;

            double s_leading_min;
            double s_following_max;

            double s_lower;
            double s_upper;
            
            double vs_lower;
            double vs_upper;
        } LaneFeasibleZone;

        /**
         * safety
         */
        static double get_safety_distance(double velocity);

        /**
         * feasibility
         */
        static bool is_feasible_trajectory(const TrajectoryGenerator::TrajectoryParams &trajectory_params, double stepsize);
        static bool is_feasible_trajectory(const TrajectoryGenerator::Trajectory &trajectory, double stepsize);
        
        /**
         * flexibility
         */
        typedef struct {
            double flexibility;
            double space;
            double leading;
            double following;

            double total;
        } FlexibilityCost;
        static FlexibilityCost get_flexibility_cost(const TrajectoryGenerator::TrajectoryReference &end, const LaneFeasibleZone &zone);

        /**
         * efficiency
         */
        typedef struct {
            double s_reached;
            double vs_reached;
            double max_speed_reached;
            double delta_d;

            double total;
        } EfficiencyCost;
        static EfficiencyCost get_efficiency_cost(
            const TrajectoryGenerator::TrajectoryReference &start,
            const TrajectoryGenerator::TrajectoryReference &end, const LaneFeasibleZone &zone
        );

    private:
        static double logistic(double x);

        /**
         * feasibility
         */
        static double get_velocity_forward_difference(int i, const std::vector<double> &x, double stepsize);
        static double get_acceleration_forward_difference(int i, const std::vector<double> &x, double stepsize);
        static double get_jerk_forward_difference(int i, const std::vector<double> &x, double stepsize);

        static bool is_feasible_velocity(double v);
        static bool is_feasible_acceleration(double a);
        static bool is_feasible_jerk(double j);    
    };

    class Urban {
    public:
    };
};

#endif /* DRIVING_STRATEGY_H */