#ifndef MAP_H
#define MAP_H

#include <vector>
#include <map>
#include "../utils/csv/csv.h"
#include "../utils/spline/spline.h"
// #include "../utils/matplotlib/matplotlibcpp.h"
#include "../planning/driving_strategy.h"

class Map {
public:
    /**
     * visual object
     */
    typedef struct {
        std::string name;
        std::vector<double> x;
        std::vector<double> y;
        std::string format;
    } Object;

    typedef std::vector<Object> ObjectList;

    /**
     * lane layout
     */
    typedef struct {
        int id;
        // geometry:
        double left_boundary;
        double right_boundary;
        double center_line;
        // traffic info:
        double speed_limit_lower;
        double speed_limit_upper;
    } Lane;

    const double LANE_WIDTH = 4.0;
    const double ROUND_TRIP_S = 6945.554;
    const double SPEED_LIMIT_LOWER_MPS = DrivingStrategy::MIN_VELOCITY;
    const double SPEED_LIMIT_UPPER_MPS = DrivingStrategy::MAX_VELOCITY;

    enum LANE_ID {
        RIGHT_1 = +1,
        RIGHT_2 = +2,
        RIGHT_3 = +3,

        NUM_LANE = 3 
    };

    enum MANEUVER {
         LANE_CHANGE_LEFT = -1,
                KEEP_LANE =  0,
        LANE_CHANGE_RIGHT = +1
    };

    /**
     * constructor
     */
    Map();

    /**
     * destructor
     */
    virtual ~Map();

    /**
     * io
     */
    void read(std::string map_filename);
    void write(std::string map_filename);

    /**
     * improve longitudinal resolution
     */
    void improve_resolution(double ratio);

    /**
     * coordinate transform 
     */
    std::vector<double> get_frenet_frame_position(double x, double y, double theta);

    std::vector<double> get_world_frame_position(double s, double d);
    std::vector<double> get_frenet_frame_vector(double x, double y, double theta, double vx, double vy);

    std::vector<double> get_world_frame_position_smoothed(double s, double d);
    /**
     * lane localization
     */
    const Lane& localize_lane(double s, double d);
    const Lane& localize_lane(double x, double y, double theta);

    /**
     * lane transition
     */
    const Lane& get_lane_by_id(LANE_ID lane_id) { return LANE[lane_id]; }
    const std::vector<MANEUVER>& get_valid_maneuvers(const Lane &lane);
    const Lane& get_next_lane(const Lane &current_lane, MANEUVER maneuver);

    /**
     * debug
     */
    // void show(const ObjectList &Object_list);
    void evaluate_precision();
private:
    /**
     * resolution
     */
    const double INVALID_S_RESOLUTION = std::numeric_limits<double>::max();

    // number of anchor points for resolution improvement:
    size_t M = 4;
    // longitudinal resolution:
    double center_line_s_resolution;

    /**
     * geometry
     */
    std::vector<double> center_line_s;
    std::vector<double> center_line_x;
    std::vector<double> center_line_y;
    std::vector<double> center_line_dx;
    std::vector<double> center_line_dy;

    tk::spline center_line_x_smoothed;
    tk::spline center_line_y_smoothed;
    tk::spline center_line_dx_smoothed;
    tk::spline center_line_dy_smoothed;
    /**
     * lane
     */
    std::map<int, Lane> LANE = {
        {RIGHT_1, {RIGHT_1, + 0.0, + 4.0, + 2.5, SPEED_LIMIT_LOWER_MPS, SPEED_LIMIT_UPPER_MPS}},
        {RIGHT_2, {RIGHT_2, + 4.0, + 8.0, + 6.0, SPEED_LIMIT_LOWER_MPS, SPEED_LIMIT_UPPER_MPS}},
        {RIGHT_3, {RIGHT_3, + 8.0, +12.0, + 9.5, SPEED_LIMIT_LOWER_MPS, SPEED_LIMIT_UPPER_MPS}}
    };

    std::map<int, std::vector<MANEUVER>> VALID_MANEUVERS = {
        {RIGHT_1, {                  KEEP_LANE, LANE_CHANGE_RIGHT}},
        {RIGHT_2, {LANE_CHANGE_LEFT, KEEP_LANE, LANE_CHANGE_RIGHT}},
        {RIGHT_3, {LANE_CHANGE_LEFT, KEEP_LANE                   }},
    };

    void add_anchor_points(
        double s0, double ds, 
        std::vector<double> &spline_s_points, std::vector<double> &spline_x_points, std::vector<double> &spline_y_points
    );
    void add_normal_d(
        double tx, double ty,
        std::vector<double> &center_line_dx, std::vector<double> &center_line_dy
    );

    double distance(double x0, double y0, double x1, double y1);

    int get_closest_waypoint(double x, double y);
    int get_next_waypoint(double x, double y, double theta);
};

#endif /* MAP_H */