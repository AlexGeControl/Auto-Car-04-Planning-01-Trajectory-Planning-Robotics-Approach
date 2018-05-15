#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

#include "../map/map.h"
#include "../vehicle/vehicle.h"
#include "../prediction/predictor.h"

#include "driving_strategy.h"
#include "trajectory_generator.h"

class Planner {
public:
    /**
     * constructor
     */
    Planner();

    /**
     * destructor
     */
    virtual ~Planner();

    /**
     * trajectory reference generation
     */
    TrajectoryGenerator::Trajectory generate_trajectory(
        Map &map,
        EgoVehicle &ego_vehicle,
        const Predictor::PredictedObjectList &predicted_object_list,
        double stepsize,
        double horizon
    );
private:
    /**
     * environment model
     */
    std::map<int, DrivingStrategy::Highway::LaneFeasibleZone> get_lane_feasible_zones(
        Map &map,
        EgoVehicle &ego_vehicle,
        const Predictor::PredictedObjectList &predicted_object_list,
        double horizon
    );

    /**
     * trajectory generator
     */
    TrajectoryGenerator trajectory_generator;
    std::default_random_engine end_configuration_generator;
    std::vector<TrajectoryGenerator::TrajectoryReference> generate_end_configuration(
        const Map::Lane &ego_lane, 
        const Map::Lane &intended_lane, 
        const DrivingStrategy::Highway::LaneFeasibleZone &lane_feasible_zone,
        double horizon
    );
    TrajectoryGenerator::Trajectory get_optimized_trajectory(
        Map &map,
        EgoVehicle &ego_vehicle,
        const TrajectoryGenerator::TrajectoryParams &newly_generated_trajectory_params,
        double stepsize
    );

    /**
     * target trajectory
     */
    typedef struct {
        int lane;
        double cost;
        TrajectoryGenerator::TrajectoryParams params;
    } TargetTrajectory;

    const int DECISION_THRESHOLD = 4;
    struct {
        int lane;
        int count;
    } decision;
};

#endif /* PLANNER_H */