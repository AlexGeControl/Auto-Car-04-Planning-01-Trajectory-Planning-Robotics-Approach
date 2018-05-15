#include "planner.h"

#include <iostream>
#include <iomanip>
#include <map>
#include <cmath>
#include <random>

#include "driving_strategy.h"
#include "../map/map.h"
#include "../utils/spline/spline.h"

/**
 * constructor
 */
Planner::Planner() {
    decision.lane = Map::LANE_ID::RIGHT_2;
    decision.count = DECISION_THRESHOLD;
}

/**
 * destructor
 */
Planner::~Planner() {

}

/**
 * trajectory reference generation
 */
TrajectoryGenerator::Trajectory Planner::generate_trajectory(
    Map &map,
    EgoVehicle &ego_vehicle,
    const Predictor::PredictedObjectList &predicted_object_list,
    double stepsize,
    double horizon
) {
    TrajectoryGenerator::Trajectory best_trajectory{0, {}, {}};

    // ego vehicle lane:
    const Map::Lane &ego_lane = map.localize_lane(ego_vehicle.s, ego_vehicle.d);
    // init configuration:
    auto start = ego_vehicle.get_trajectory_reference();
    // init target trajectory for each maneuver:
    std::map<int, TargetTrajectory> target_trajectories;
    
    // surrounding environment:
    auto lane_feasible_zones = get_lane_feasible_zones(map, ego_vehicle, predicted_object_list, horizon);
    for (std::map<int, DrivingStrategy::Highway::LaneFeasibleZone>::iterator it = lane_feasible_zones.begin(); lane_feasible_zones.end() != it; ++it) {
        // parse feasible zone:
        Map::LANE_ID intended_lane_id = (Map::LANE_ID) it->first;
        const Map::Lane &intended_lane = map.get_lane_by_id(intended_lane_id);
        const DrivingStrategy::Highway::LaneFeasibleZone &lane_feasible_zone = it->second;

        // init target trajectory:
        target_trajectories[intended_lane_id] = TargetTrajectory{
            intended_lane_id,
            std::numeric_limits<double>::max(),
            {0.0, {}, {}}
        };

        // propose end configurations:
        auto end_configurations = generate_end_configuration(
            ego_lane, 
            intended_lane, lane_feasible_zone, 
            horizon
        );

        TrajectoryGenerator::TrajectoryParams trajectory_params;
        for (const auto &end: end_configurations) {
            // jerk minimized trajectory:
            trajectory_params = trajectory_generator.generate_jerk_minimized_trajectory(start, end);
            
            // if feasible:
            if (DrivingStrategy::Highway::is_feasible_trajectory(trajectory_params, stepsize)) {
                // a. efficiency:
                double efficiency_cost = DrivingStrategy::Highway::get_efficiency_cost(start, end, lane_feasible_zone);
                // b. flexibility:
                double flexibility_cost = DrivingStrategy::Highway::get_flexibility_cost(end, lane_feasible_zone);
                // c. safety:

                // total cost:
                double total_cost = efficiency_cost + flexibility_cost;
                std::cout << "[Total Cost]: \n" << "\t" << total_cost << std::endl << std::endl;

                // whether optimal:
                if (total_cost < target_trajectories[intended_lane_id].cost) {
                    target_trajectories[intended_lane_id].cost = total_cost;
                    target_trajectories[intended_lane_id].params = trajectory_params;
                }
            }
        }

        // keep only feasible lane:
        if (target_trajectories[intended_lane_id].params.T == 0.0) {
            target_trajectories.erase(intended_lane_id);
        }
    }

    if (0 == target_trajectories.size()) {
        std::cout << "[NO Feasible Trajectory]!!!" << std::endl;
    } else {
        // identify optimal target trajectory:
        TargetTrajectory optimal_target_trajectory{
            0,
            std::numeric_limits<double>::max(),
            {0.0, {}, {}}            
        };
        for (std::map<int, TargetTrajectory>::iterator it = target_trajectories.begin(); target_trajectories.end() != it; ++it) {
            TargetTrajectory &target_trajectory = it->second;

            if (target_trajectory.cost < optimal_target_trajectory.cost) {
                optimal_target_trajectory = target_trajectory;
            }
        }

        // decision smooth:
        if (decision.lane != optimal_target_trajectory.lane) {
            decision.lane = optimal_target_trajectory.lane;
            decision.count = 0;
        } 
        else if (decision.count >= DECISION_THRESHOLD) {
            // generate new trajectory:
            best_trajectory = get_optimized_trajectory(map, ego_vehicle, optimal_target_trajectory.params, stepsize);

            // set ego vehicle trajectory:
            ego_vehicle.set_trajectory(optimal_target_trajectory.params);
            std::cout << "[Update]" << std::endl;
        } else {
            ++decision.count;
        }
    }

    return best_trajectory;
}

/**
 * get lane feasible zone in surrounding environment
 * 
 * @param map: HD map
 * @param ego_vehicle: Ego vehicle
 * @param predicted_object_list: Predicted objects
 * @param horizon Planning horizon
 * @return Planned trajectory
 */
std::map<int, DrivingStrategy::Highway::LaneFeasibleZone> Planner::get_lane_feasible_zones(
    Map &map,
    EgoVehicle &ego_vehicle,
    const Predictor::PredictedObjectList &predicted_object_list,
    double horizon
) {
    std::map<int, DrivingStrategy::Highway::LaneFeasibleZone> lane_feasible_zones;

    // localize ego vehicle:
    const Map::Lane &ego_lane = map.localize_lane(ego_vehicle.s, ego_vehicle.d);
    // current dynamics in frenet:
    TrajectoryGenerator::TrajectoryReference ego_trajectory_reference = ego_vehicle.get_trajectory_reference();
    double effective_acceleration = std::min(
        +DrivingStrategy::MAX_ACCELERATION,
        ego_trajectory_reference.as + 0.5 * (DrivingStrategy::MAX_JERK * horizon)
    );
    double effective_deceleration = std::max(
        -DrivingStrategy::MAX_ACCELERATION,
        ego_trajectory_reference.as - 0.5 * (DrivingStrategy::MAX_JERK * horizon)
    );
    // get valid maneuvers:
    auto feasible_maneuvers = map.get_valid_maneuvers(ego_lane);

    // estimate feasible zone for each maneuver:
    if (feasible_maneuvers.size() > 0) {
        lane_feasible_zones.clear();

        // init containers for s lower & upper bounds:
        for (auto &feasible_maneuver: feasible_maneuvers) {
            // next lane:
            const Map::Lane &next_lane = map.get_next_lane(ego_lane, feasible_maneuver);

            DrivingStrategy::Highway::LaneFeasibleZone lane_feasible_zone;

            // init as without leading vehicle:
            lane_feasible_zone.is_with_leading_vehicle = false;
            lane_feasible_zone.is_with_following_vehicle = false;

            // init position region:
            lane_feasible_zone.s_following_max = std::numeric_limits<double>::min();
            lane_feasible_zone.s_leading_min = std::numeric_limits<double>::max();
            lane_feasible_zone.s_lower = ego_vehicle.s + horizon * ego_trajectory_reference.vs + std::max(
                0.5 * std::pow(horizon, 2.0) * effective_deceleration,
                0.5 * std::pow(ego_trajectory_reference.vs, 2.0) / effective_deceleration
            );
            lane_feasible_zone.s_upper = lane_feasible_zone.s_lower + std::min(
                // speed up at max acceleration
                horizon * ego_trajectory_reference.vs + 0.5 * std::pow(horizon, 2.0) * effective_acceleration,
                // constant speed at lane upper speed
                horizon * next_lane.speed_limit_upper
            );

            // init velocity region: 
            lane_feasible_zone.vs_lower = std::max(
                0.0,
                ego_trajectory_reference.vs + horizon * effective_deceleration
            );

            lane_feasible_zone.vs_upper = std::min(
                next_lane.speed_limit_upper,
                ego_trajectory_reference.vs + horizon * effective_acceleration
            );

            lane_feasible_zones[next_lane.id] = lane_feasible_zone;
        }

        // set s lower & upper bounds according to surrounding object prediction:
        for (const auto &predicted_object: predicted_object_list) {
            size_t N = predicted_object.prediction.size();

            const Predictor::Prediction &init_pose = predicted_object.prediction[0];
            const Predictor::Prediction &terminal_pose = predicted_object.prediction[N - 1];

            // init state:
            double s0 = init_pose.s;
            double d0 = init_pose.d;
            double vs0 = init_pose.vs;

            // terminal state:
            double s1 = terminal_pose.s;
            double d1 = terminal_pose.d;
            double vs1 = terminal_pose.vs;

            // occupied lane:
            const Map::Lane &object_lane = map.localize_lane(s0, d0);

            // localize and classify surrounding objects according to their s coord relative to ego:
            if (lane_feasible_zones.end() == lane_feasible_zones.find(object_lane.id)) {
                continue;
            }

            DrivingStrategy::Highway::LaneFeasibleZone &lane_feasible_zone = lane_feasible_zones[object_lane.id];
            if (s0 < ego_vehicle.s) {
                lane_feasible_zone.s_following_max = std::max(lane_feasible_zone.s_following_max, s1);
                // object behind ego:
                if (object_lane.id != ego_lane.id) {
                    if (vs1 > ego_trajectory_reference.vs) {
                        double safety_distance = DrivingStrategy::Highway::get_safety_distance(vs1 - ego_trajectory_reference.vs);
                        double s_lower_proposed = s1 + safety_distance;
                        if (s_lower_proposed > lane_feasible_zone.s_lower) {
                            lane_feasible_zone.is_with_following_vehicle = true;
                            lane_feasible_zone.s_lower = s_lower_proposed;
                            lane_feasible_zone.vs_lower = vs1;
                        }
                    } else {
                        // too slow:
                    }
                } 
            } else {
                lane_feasible_zone.s_leading_min = std::min(lane_feasible_zone.s_leading_min, s1);

                if (s1 < lane_feasible_zone.s_upper) {
                    double safety_distance = DrivingStrategy::Highway::get_safety_distance(vs1 - ego_trajectory_reference.vs);
                    double s_lower_proposed = s1 + safety_distance;
                    if (s_lower_proposed > lane_feasible_zone.s_lower) {
                        lane_feasible_zone.is_with_following_vehicle = true;
                        lane_feasible_zone.s_lower = s_lower_proposed;
                        lane_feasible_zone.vs_lower = vs1;
                    }                    
                } else {
                    // object in front of ego:
                    if (vs1 < lane_feasible_zone.vs_upper) {
                        double safety_distance = DrivingStrategy::Highway::get_safety_distance(vs1 - lane_feasible_zone.vs_lower);
                        double s_upper_proposed = s1 - safety_distance;
                        if (s_upper_proposed < lane_feasible_zone.s_upper) {
                            lane_feasible_zone.is_with_leading_vehicle = true;
                            lane_feasible_zone.s_upper = s_upper_proposed;
                            lane_feasible_zone.vs_upper = vs1;
                        }
                    } else {
                        // too fast:
                    }
                }
            }
        }

        // eliminate infeasible zones:
        for (std::map<int, DrivingStrategy::Highway::LaneFeasibleZone>::iterator it = lane_feasible_zones.begin(); lane_feasible_zones.end() != it; ++it) {
            Map::LANE_ID intended_lane_id = (Map::LANE_ID) it->first;
            DrivingStrategy::Highway::LaneFeasibleZone &lane_feasible_zone = it->second;

            if (lane_feasible_zone.s_lower > lane_feasible_zone.s_upper || lane_feasible_zone.vs_lower > lane_feasible_zone.vs_upper) {
                lane_feasible_zones.erase(it);
                continue;
            }

            if (!lane_feasible_zone.is_with_leading_vehicle) {
                lane_feasible_zone.vs_lower = ego_trajectory_reference.vs;
                lane_feasible_zone.vs_upper = DrivingStrategy::MAX_VELOCITY;
            }
            /* DEBUG Feasible Zone
            std::cout << "\t[Lane]: " << it->first << std::endl;
            std::cout << "\t\t[Other Vehicle Presence]: " << lane_feasible_zone.is_with_leading_vehicle << ", " << lane_feasible_zone.is_with_following_vehicle << std::endl;
            std::cout << "\t\t[Available Area]: " << lane_feasible_zone.s_leading_min << ", " << lane_feasible_zone.s_following_max << std::endl;
            std::cout << "\t\t[Planning S]: " << lane_feasible_zone.s_upper << ", " << lane_feasible_zone.s_lower << std::endl;
            std::cout << "\t\t[Planning VS]: " << lane_feasible_zone.vs_upper << ", " << lane_feasible_zone.vs_lower << std::endl;
            */
        }
    }

    return lane_feasible_zones;
}

/**
 * generate end configuration
 * 
 * @param ego_lane
 * @param intended_lane
 * @param lane_feasible_zone
 * @param horizon
 * @return vector of trajectory reference
 */
std::vector<TrajectoryGenerator::TrajectoryReference> Planner::generate_end_configuration(
    const Map::Lane &ego_lane, 
    const Map::Lane &intended_lane, 
    const DrivingStrategy::Highway::LaneFeasibleZone &lane_feasible_zone,
    double horizon
) {
    std::vector<TrajectoryGenerator::TrajectoryReference> result;

    // for (double delta_horizon = -0.10 * horizon; delta_horizon < +0.20 * horizon; delta_horizon += 0.10 * horizon) {
    for (double s_factor = 0.20; s_factor <= 1.00; s_factor += 0.10) {
        for (double vs_factor = 0.20; vs_factor <= 1.00; vs_factor += 0.10) {
            // init:
            TrajectoryGenerator::TrajectoryReference end{
                // horizon:
                0.0, 
                // s:
                0.0, 0.0, 0.0, 
                // d:
                0.0, 0.0, 0.0
            }; 
            
            // horizon:
            end.timestamp = horizon;

            // s states:
            end.s = s_factor * lane_feasible_zone.s_lower + (1.0 - s_factor) * lane_feasible_zone.s_upper;
            end.vs = vs_factor * lane_feasible_zone.vs_lower + (1.0 - vs_factor) * lane_feasible_zone.vs_upper;

            // accelerate until reach speed lower limit:
            if (end.vs < intended_lane.speed_limit_lower) {
                end.as = 0.9 * (1.0 - std::pow(end.vs / intended_lane.speed_limit_upper, 2.0)) * DrivingStrategy::MAX_ACCELERATION;   
            }
            end.d = intended_lane.center_line;

            result.push_back(end);
        }
    }
    // }

    return result;
}

TrajectoryGenerator::Trajectory Planner::get_optimized_trajectory(
    Map &map,
    EgoVehicle &ego_vehicle,
    const TrajectoryGenerator::TrajectoryParams &newly_generated_trajectory_params,
    double stepsize
) {
    TrajectoryGenerator::Trajectory trajectory{0, {}, {}};

    // spline:
    std::vector<double> t_input;
    std::vector<double> x_output, y_output;
    
    tk::spline x_smoothed, y_smoothed;

    // previous trajectory
    double t0 = ego_vehicle.get_time();
    if (t0 > 0.0) {
        const TrajectoryGenerator::TrajectoryParams &previous_trajectory_params = ego_vehicle.get_trajectory();
        for (double t = 0.5 * stepsize; t < t0; t += stepsize) {
            double s = TrajectoryGenerator::evaluate_polynomial_derivative(t, previous_trajectory_params.s, 0);
            double d = TrajectoryGenerator::evaluate_polynomial_derivative(t, previous_trajectory_params.d, 0);

            auto position_world = map.get_world_frame_position_smoothed(s, d);

            double x = position_world[0];
            double y = position_world[1];

            t_input.push_back(t - t0);
            x_output.push_back(x);
            y_output.push_back(y);
        }
    }

    // newly generated trajectory:
    for (double t = 0.5*stepsize; t < newly_generated_trajectory_params.T; t += stepsize) {
        double s = TrajectoryGenerator::evaluate_polynomial_derivative(t, newly_generated_trajectory_params.s, 0);
        double d = TrajectoryGenerator::evaluate_polynomial_derivative(t, newly_generated_trajectory_params.d, 0);

        auto position_world = map.get_world_frame_position_smoothed(s, d);

        double x = position_world[0];
        double y = position_world[1];

        t_input.push_back(t);
        x_output.push_back(x);
        y_output.push_back(y);
    }

    x_smoothed.set_points(t_input, x_output);
    y_smoothed.set_points(t_input, y_output);

    for (double t = 0.0; t < newly_generated_trajectory_params.T; t += stepsize) {
        double x = x_smoothed(t);
        double y = y_smoothed(t);

        trajectory.N += 1;
        trajectory.x.push_back(x);
        trajectory.y.push_back(y);                
    }
    
    return trajectory;    
}