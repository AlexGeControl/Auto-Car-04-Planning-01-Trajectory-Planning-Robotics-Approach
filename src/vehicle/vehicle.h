#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

#include "../map/map.h"
#include "../planning/trajectory_generator.h"

class EgoVehicle {
public:
    /**
     * actuation delay
     */
    double STEPSIZE = 0.02;
    const int ACTUATION_DELAY = 1;
    /**
     * vehicle pose
     */
    double s;
    double d;
    double x;
    double y;
    double yaw;
    double speed;

    /**
     * target position
     */
    double target_s;
    double target_d;

    /**
     * constructor
     */
    EgoVehicle();

    /**
     * destructor
     */ 
    virtual ~EgoVehicle();

    /**
     * is engine starting
     */
    bool is_starting();

    /**
     * time sync using previous trajectory
     */
    void sync_time(const TrajectoryGenerator::Trajectory &previous_trajectory);

    /**
     * get time passed since last planning
     */
    double get_time() { return t; }
    
    /**
     * get ego vehicle current trajectory reference
     */ 
    TrajectoryGenerator::TrajectoryReference get_trajectory_reference();

    /**
     * get ego vehicle trajector
     */
    const TrajectoryGenerator::TrajectoryParams &get_trajectory() { return trajectory_params; } 

    /**
     * set ego vehicle trajectory
     */
    void set_trajectory(const TrajectoryGenerator::TrajectoryParams &trajectory_params);
private:
    /**
     * parameterized ego vehicle trajectory
     */
    double t;
    TrajectoryGenerator::TrajectoryParams trajectory_params;
};

#endif /* VEHICLE_H */