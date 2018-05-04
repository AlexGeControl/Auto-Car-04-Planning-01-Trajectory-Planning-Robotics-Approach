#include <cmath>
#include "predictor.h"

/**
 * constructor
 */
Predictor::Predictor(Map &map): m_map(map) {

}

/**
 * destructor
 */
Predictor::~Predictor() {

}

void Predictor::init() {

}

/**
    update predictor state with new observation.

    @param fused_object_list sensor fusion input.
*/
void Predictor::update(const FusedObjectList &fused_object_list) {

}

/**
    generate new prediction.

    @param fused_object_list sensor fusion input.
    @param t0 starting timestamp
    @param stepsize prediction time stepsize
    @param horizon prediction horizon
*/
Predictor::PredictedObjectList Predictor::predict(const Predictor::FusedObjectList &fused_object_list, double t0, double stepsize, double horizon) {
    // maneuver 01: keep lane
    return (this->*m_maneuver[KEEP_LANE])(fused_object_list, t0, stepsize, horizon);
    // maneuver 02: drive freespace
    // return (this->*m_maneuver[DRIVE_FREESPACE])(fused_object_list, t0, stepsize, horizon);

    // TODO: implement Autonomous Multiple Model(AMM) prediction 
}

/**
    predict with keep lane maneuver.

    @param fused_object_list sensor fusion input.
    @param t0 starting timestamp
    @param stepsize prediction time stepsize
    @param horizon prediction horizon
*/
Predictor::PredictedObjectList Predictor::keep_lane(const Predictor::FusedObjectList &fused_object_list, double t0, double stepsize, double horizon) {
    PredictedObjectList predicted_object_list;

    for (const FusedObject &fused_object: fused_object_list) {
        PredictedObject predicted_object;

        // set id:
        predicted_object.id = static_cast<int>(fused_object[0]);

        // init position:
        double s = fused_object[5];
        double d = fused_object[6];

        // keep only objects at the same side:
        if (d < 0.0) {
            continue;
        }

        // init velocity:
        double vx = fused_object[3];
        double vy = fused_object[4];

        // predict using keep lane maneuver:
        double vs = sqrt(vx*vx + vy*vy);

        for (double delta_t = 0.0; delta_t <= horizon; delta_t += stepsize) {
            Prediction prediction;

            prediction.timestamp = t0 + delta_t;

            prediction.s = s + delta_t * vs;
            prediction.d = d;

            auto position = m_map.get_world_frame_position(prediction.s, prediction.d);

            prediction.x = position[0];
            prediction.y = position[1];

            prediction.vs = vs;

            predicted_object.prediction.push_back(prediction);
        }

        predicted_object_list.push_back(predicted_object);
    }

    return predicted_object_list;    
}

/**
    predict with drive freespace maneuver.

    @param fused_object_list sensor fusion input.
    @param t0 starting timestamp
    @param stepsize prediction time stepsize
    @param horizon prediction horizon
*/
Predictor::PredictedObjectList Predictor::drive_freespace(const Predictor::FusedObjectList &fused_object_list, double t0, double stepsize, double horizon) {
    PredictedObjectList predicted_object_list;

    for (const FusedObject &fused_object: fused_object_list) {
        PredictedObject predicted_object;

        // set id:
        predicted_object.id = static_cast<int>(fused_object[0]);

        // init position:
        double x = fused_object[1];
        double y = fused_object[2];

        // keep only objects at the same side:
        double d = fused_object[6];
        if (d < 0.0) {
            continue;
        }
        
        // init velocity:
        double vx = fused_object[3];
        double vy = fused_object[4];
        double vs = sqrt(vx*vx + vy*vy);
        double theta = atan2(vy, vx);

        // predict using drive freespace maneuver:
        for (double delta_t = 0.0; delta_t <= horizon; delta_t += stepsize) {
            Prediction prediction;

            prediction.timestamp = t0 + delta_t;
            
            prediction.x = x + delta_t * vx;
            prediction.y = y + delta_t * vy;

            auto position = m_map.get_frenet_frame_position(prediction.x, prediction.y, theta);

            prediction.s = position[0];
            prediction.d = position[1];

            prediction.vs = vs;

            predicted_object.prediction.push_back(prediction);
        }

        predicted_object_list.push_back(predicted_object);
    }

    return predicted_object_list;    
}

