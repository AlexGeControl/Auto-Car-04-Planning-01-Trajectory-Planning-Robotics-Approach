#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <vector>
#include "../map/map.h"

class Predictor {
public:
    /**
     * sensor fusion input type
     */
    typedef std::vector<double> FusedObject;
    typedef std::vector<FusedObject> FusedObjectList;

    /**
     * object prediction output type 
     */
    typedef struct {
        double timestamp;

        double s;
        double d;
        double x;
        double y;

        double vs;
    } Prediction;

    typedef struct {
        int id;
        std::vector<Prediction> prediction;
    } PredictedObject;

    typedef std::vector<PredictedObject> PredictedObjectList;

    /**
     * constructor
     */
    Predictor(Map &map);

    /**
     * destructor
     */
    virtual ~Predictor(); 

    /**
     * initializer
     */
    void init();

    /**
     * update predictor state using new observation:
     */ 
    void update(const FusedObjectList &fused_object_list);

    /**
     * generate prediction from object fusion
     */
    PredictedObjectList predict(const FusedObjectList &fused_object_list, double t0, double stepsize, double horizon);
private:
    Map &m_map;

    /**
     * maneuver signature
     */ 
    typedef PredictedObjectList (Predictor::*Maneuver)(const FusedObjectList &fused_object_list, double t0, double stepsize, double horizon);

    /**
     * total number of maneuvers
     */ 
    enum {
        KEEP_LANE,
        DRIVE_FREESPACE,
        NUM_MANEUVERS
    };
    
    /**
     * maneuver 01: follow lane
     */
    PredictedObjectList keep_lane(const FusedObjectList &fused_object_list, double t0, double stepsize, double horizon);
    /**
     * maneuver 02: free space
     */
    PredictedObjectList drive_freespace(const FusedObjectList &fused_object_list, double t0, double stepsize, double horizon);

    Maneuver m_maneuver[NUM_MANEUVERS] = {
        &Predictor::keep_lane,
        &Predictor::drive_freespace
    };
};

#endif /* PREDICTOR_H */