

#include <iostream>
#include <fstream>
#include <cmath>

#include "../utils/spline/spline.h"
//#include "../utils/matplotlib/matplotlibcpp.h"

#include "map.h"

namespace csv = io;
// namespace plt = matplotlibcpp;

/**
    map constructor.
*/
Map::Map() {
    center_line_s_resolution = INVALID_S_RESOLUTION;
}

/**
    map destructor.
*/
Map::~Map() {
    ;
}

/**
    load HD map.

    @param map_filename The filename of HD map.
*/
void Map::read(std::string map_filename) {
    // define map format:
    csv::CSVReader<5, io::trim_chars<>, io::no_quote_escape<' '>> hd_map(map_filename);
  	hd_map.read_header(io::ignore_extra_column, "s", "x", "y","dx", "dy");

    // parse map:
  	double s, x, y, dx, dy;
  	while(hd_map.read_row(s, x, y, dx, dy)){
    	center_line_s.push_back(s);
        center_line_x.push_back(x);
        center_line_y.push_back(y);
        center_line_dx.push_back(dx);
        center_line_dy.push_back(dy);
  	};
    // round trip:
    center_line_s.push_back(ROUND_TRIP_S);
    center_line_x.push_back(center_line_x[0]);
    center_line_y.push_back(center_line_y[0]);
    center_line_dx.push_back(center_line_dx[0]);
    center_line_dy.push_back(center_line_dy[0]);

    // parse s resolution:
    if (center_line_s.size() > 1) {
        size_t N = center_line_s.size();

        center_line_s_resolution = 0.0;
        for (size_t i = 1; i < N; ++i) {
            center_line_s_resolution += (center_line_s[i] - center_line_s[i - 1]);
        }
        center_line_s_resolution /= (N - 1);         
    }

    center_line_x_smoothed.set_points(center_line_s, center_line_x);
    center_line_y_smoothed.set_points(center_line_s, center_line_y);
    center_line_dx_smoothed.set_points(center_line_s, center_line_dx);
    center_line_dy_smoothed.set_points(center_line_s, center_line_dy);
}

/**
    write HD map as CSV file.

    @param map_filename The filename of HD map.
*/
void Map::write(std::string map_filename) {
    std::ofstream hd_map;

    hd_map.open(map_filename);

    // write header:
    hd_map << "x y s dx dy\n";
    // write data:
    size_t N = center_line_s.size();
    for (size_t i = 0; i < N; ++i) {
        hd_map << std::dec << center_line_x[i] << " ";
        hd_map << std::dec << center_line_y[i] << " ";
        hd_map << std::dec << center_line_s[i] << " ";
        hd_map << std::dec << center_line_dx[i] << " ";
        hd_map << std::dec << center_line_dy[i] << "\n";
    }

    hd_map.close();
}

/**
    improve HD map resolution.

    @param ratio The filename of HD map.
*/
void Map::improve_resolution(double ratio) {
    if (
        (INVALID_S_RESOLUTION != center_line_s_resolution) && (1.0 < ratio)
    ) {
        size_t N = center_line_s.size();

        // set anchor waypoints:
        std::vector<double> spline_s_points, spline_x_points, spline_y_points;
        // waypoints except the last one:
        spline_s_points.push_back(center_line_s[0]);
        spline_x_points.push_back(center_line_x[0]);
        spline_y_points.push_back(center_line_y[0]);
        for (size_t i = 1; i < N; ++i) {
            add_anchor_points(
                center_line_s[i - 1], center_line_s[i] - center_line_s[i - 1], 
                spline_s_points, spline_x_points, spline_y_points
            );

            spline_s_points.push_back(center_line_s[i]);
            spline_x_points.push_back(center_line_x[i]);
            spline_y_points.push_back(center_line_y[i]);
        }
        // last waypoint wrap around:
        add_anchor_points(
            center_line_s[N - 1], ROUND_TRIP_S - center_line_s[N - 1], 
            spline_s_points, spline_x_points, spline_y_points
        );

        spline_s_points.push_back(ROUND_TRIP_S);
        spline_x_points.push_back(center_line_x[0]);
        spline_y_points.push_back(center_line_y[0]);

        // spline for reference line:
        tk::spline spline_x, spline_y;
        spline_x.set_points(spline_s_points, spline_x_points);
        spline_y.set_points(spline_s_points, spline_y_points);

        // improve longitudinal resolution:
        std::vector<double> new_center_line_s, new_center_line_x, new_center_line_y, new_center_line_dx, new_center_line_dy;

        double new_center_line_s_resolution = center_line_s_resolution / ratio;

        new_center_line_s.push_back(center_line_s[0]);
        new_center_line_x.push_back(center_line_x[0]);
        new_center_line_y.push_back(center_line_y[0]);
        for (double s = center_line_s[0] + new_center_line_s_resolution; s < ROUND_TRIP_S; s += new_center_line_s_resolution) {
            size_t n = new_center_line_s.size();

            double s0 = new_center_line_s[n - 1];
            double x0 = new_center_line_x[n - 1];
            double y0 = new_center_line_y[n - 1];

            double x = spline_x(s);
            double y = spline_y(s);

            double dx = x - x0;
            double dy = y - y0;

            new_center_line_s.push_back(s0 + sqrt(dx * dx + dy * dy));
            new_center_line_x.push_back(x);
            new_center_line_y.push_back(y);
        }

        for (size_t i = 1; i < new_center_line_s.size(); ++i) {
            add_normal_d(
                new_center_line_x[i] - new_center_line_x[i - 1],
                new_center_line_y[i] - new_center_line_y[i - 1],
                new_center_line_dx,
                new_center_line_dy
            );
        }
        add_normal_d(
            new_center_line_x[0] - new_center_line_x[new_center_line_s.size() - 1],
            new_center_line_y[0] - new_center_line_y[new_center_line_s.size() - 1],
            new_center_line_dx,
            new_center_line_dy
        );

        center_line_s = new_center_line_s;
        center_line_x = new_center_line_x;
        center_line_y = new_center_line_y;
        center_line_dx = new_center_line_dx;
        center_line_dy = new_center_line_dy;

        std::cout << "[Resolution Improvement]: from " << std::dec << center_line_s_resolution << " to " << new_center_line_s_resolution <<std::endl;
    }
}

/**
    transform position in world frame to frenet frame.

    @param x world x.
    @param y world y.
    @param theta world theta.    
    @return position in frenet frame.
*/
std::vector<double> Map::get_frenet_frame_position(double x, double y, double theta)
{
	int next_waypoint_index = get_next_waypoint(x,y, theta);

	int prev_waypoint_index = (0 == next_waypoint_index ? center_line_x.size() - 1 : next_waypoint_index - 1);

    // normalized tangential vector:
	double tx = center_line_x[next_waypoint_index]-center_line_x[prev_waypoint_index];
	double ty = center_line_y[next_waypoint_index]-center_line_y[prev_waypoint_index];
    double norm_t = sqrt(tx*tx + ty*ty);
    tx /= norm_t;
    ty /= norm_t;

    // translation relative to previous waypoint:
	double dx = x - center_line_x[prev_waypoint_index];
	double dy = y - center_line_y[prev_waypoint_index];

	// find the projection of translation onto t
	double norm_proj = (dx*tx+dy*ty);
	double proj_x = norm_proj * tx;
	double proj_y = norm_proj * ty;

    // frenet s:
    double frenet_s = center_line_s[prev_waypoint_index] + norm_proj; 

    // frenet d:
	double frenet_d = distance(dx,dy,proj_x,proj_y);

    // determine the sign of d:
    double perp_x = dx - proj_x;
    double perp_y = dy - proj_y;
    double nx = center_line_dx[prev_waypoint_index];
    double ny = center_line_dy[prev_waypoint_index];
    if (perp_x*nx + perp_y*ny < 0.0) {
        frenet_d *= -1;
    }

	return {frenet_s,frenet_d};
}

/**
    transform position in frenet frame to world frame.

    @param s frenet s.
    @param d frenet d.
    @return position in world frame.
*/
std::vector<double> Map::get_world_frame_position(double s, double d)
{
    // prevent overflow:
    s = fmod(s, ROUND_TRIP_S);

    // identify segment:
    std::vector<double>::iterator upper = std::upper_bound(
        center_line_s.begin(), center_line_s.end(), 
        s
    );

    int N = center_line_s.size();
    int waypoint_end_id = (upper - center_line_s.begin()) % N;
    int waypoint_start_id = (waypoint_end_id + N - 1) % N;

	double heading = atan2(
        (center_line_y[waypoint_end_id]-center_line_y[waypoint_start_id]),
        (center_line_x[waypoint_end_id]-center_line_x[waypoint_start_id])
    );
	
	double seg_s = (s-center_line_s[waypoint_start_id]);
	double seg_x = center_line_x[waypoint_start_id]+seg_s*cos(heading);
	double seg_y = center_line_y[waypoint_start_id]+seg_s*sin(heading);
	
	double world_x = seg_x + d * center_line_dx[waypoint_start_id];
	double world_y = seg_y + d * center_line_dy[waypoint_start_id];

	return {world_x, world_y};
}

/**
    transform position in frenet frame to world frame using spline smoother.

    @param s frenet s.
    @param d frenet d.
    @return position in world frame.
*/
std::vector<double> Map::get_world_frame_position_smoothed(double s, double d)
{
    // rectify s:
    s = fmod(s, ROUND_TRIP_S);

    // estimate normal vector:
    double dx = center_line_dx_smoothed(s);
    double dy = center_line_dy_smoothed(s);

    // unit normal vector:
    double norm_d = sqrt(dx*dx + dy*dy);
    dx /= norm_d;
    dy /= norm_d;

    // center line:
    double world_x = center_line_x_smoothed(s);
    double world_y = center_line_y_smoothed(s);

    // add normal shift:
    world_x += d * dx;
    world_y += d * dy;
    
	return {world_x, world_y};
}

/**
    transform vector in world frame to frenet frame .

    @param x world x.
    @param y world y.
    @param theta world theta. 
    @param vx world vx.
    @param vy world vy.   
    @return vector in frenet frame.
*/
std::vector<double> Map::get_frenet_frame_vector(double x, double y, double theta, double vx, double vy) {
    // localize lane segment:
	int next_waypoint_index = get_next_waypoint(x,y, theta);
	int prev_waypoint_index = (0 == next_waypoint_index ? center_line_x.size() - 1 : next_waypoint_index - 1);

    // unit normal vector:
    double nx = center_line_dx[prev_waypoint_index];
    double ny = center_line_dy[prev_waypoint_index];

    double vs = (-vx*ny + vy*nx);
    double vd = (+vx*nx + vy*ny);

    return {vs, vd};
}

/**
    localize lane with object's frenet frame coords.

    @param s frenet s.
    @param d frenet d.
    @return position in world frame.
*/
const Map::Lane& Map::localize_lane(double s, double d) {
    // localize lane using d:
    int id = std::ceil(d / LANE_WIDTH);

    return LANE[id];
}
/**
    localize lane with object's world frame coords.

    @param x world x.
    @param y world y.
    @param theta world theta.
    @return position in frenet frame.
*/
const Map::Lane& Map::localize_lane(double x, double y, double theta) {
    // transform to frenet:
    auto position = get_frenet_frame_position(x, y, theta);

    double s = position[0];
    double d = position[1];

    return localize_lane(s, d);
}

/**
    get valid maneuvers of selected lane

    @param lane: selected lane
    @return: valid maneuvers
*/
const std::vector<Map::MANEUVER>& Map::get_valid_maneuvers(const Map::Lane &lane) {
    return VALID_MANEUVERS[lane.id];
}

/**
    get next lane

    @param current_lane: current lane
    @param maneuver: select maneuver
    @return: next lane
*/
const Map::Lane& Map::get_next_lane(const Map::Lane &current_lane, Map::MANEUVER maneuver) {
    // lane transition:
    int next_lane_id = current_lane.id + maneuver;

    return LANE[next_lane_id];
}

/**
    visualize HD map.

void Map::show(const ObjectList &object_list) {
    std::cout << "[Map Resolution]: " << std::dec << center_line_s_resolution << std::endl;
    
    plt::named_plot("Map", center_line_x, center_line_y);

    for (const Object &object: object_list) {
       plt::named_plot(object.name, object.x, object.y, object.format);
    }

    plt::legend();

    plt::show("Path Planning");
}
*/

/**
    analyze HD map precision.
*/
void Map::evaluate_precision() {
    double total_error = 0.0;
    double total_count = 0;

    for (size_t i = 0; i < center_line_s.size(); ++i) {
        double s = center_line_s[i];

        std::map<int, Lane>::iterator it;
        for (it = LANE.begin(); it != LANE.end(); ++it) {
            const Lane &lane = it->second;

            double d = lane.center_line;
            auto position_world = get_world_frame_position(s, d);

            double x = position_world[0];
            double y = position_world[1];
            double dx = center_line_dx[i];
            double dy = center_line_dy[i];

            double theta = M_PI / 2.0 + atan2(dy, dx);

            auto position_frenet = get_frenet_frame_position(x, y, theta);

            // update stats:
            total_error += sqrt(pow(position_frenet[0] - s, 2.0) + pow(position_frenet[1] - d, 2.0));
            total_count += 1;
        }
    }

    std::cout << "[Precision]: " << total_error / total_count << std::endl;
}

void Map::add_anchor_points(
    double s0, double ds, 
    std::vector<double> &spline_s_points, std::vector<double> &spline_x_points, std::vector<double> &spline_y_points
) {
    for (size_t i = 1; i < M; ++i) {
        double s = s0 + i * ds / M;

        auto waypoint = get_world_frame_position(s, 0.0);

        spline_s_points.push_back(s);
        spline_x_points.push_back(waypoint[0]);
        spline_y_points.push_back(waypoint[1]);
    }    
}

void Map::add_normal_d(
    double tx, double ty,
    std::vector<double> &center_line_dx, std::vector<double> &center_line_dy
) {
    double norm_t = sqrt(tx*tx + ty*ty);

    double dx = +ty / norm_t;
    double dy = -tx / norm_t;

    center_line_dx.push_back(dx);
    center_line_dy.push_back(dy);
}

double Map::distance(double x0, double y0, double x1, double y1) {
    return sqrt(pow(x1 - x0, 2.0) + pow(y1 - y0, 2.0));
}

int Map::get_closest_waypoint(double x, double y)
{
	double closest_distance = std::numeric_limits<double>::max();
	int closest_waypoint_index = -1;

	for(int i = 0; i < center_line_x.size(); i++)
	{
		double waypoint_x = center_line_x[i];
		double waypoint_y = center_line_y[i];

		double current_distance = distance(x,y,waypoint_x,waypoint_y);

		if(current_distance < closest_distance)
		{
			closest_distance = current_distance;
			closest_waypoint_index = i;
		}
	}

	return closest_waypoint_index;
}

int Map::get_next_waypoint(double x, double y, double theta)
{
	int next_waypoint_index = get_closest_waypoint(x,y);

	double waypoint_x = center_line_x[next_waypoint_index];
	double waypoint_y = center_line_y[next_waypoint_index];

	double heading = atan2(
        (waypoint_y-y),
        (waypoint_x-x)
    );

	double angle = fabs(theta - heading);

	angle = std::min(2 * M_PI - angle, angle);

	if(angle > M_PI / 4)
	{
		next_waypoint_index++;
		if (next_waypoint_index == center_line_x.size())
		{
			next_waypoint_index = 0;
		}
	}

	return next_waypoint_index;
}