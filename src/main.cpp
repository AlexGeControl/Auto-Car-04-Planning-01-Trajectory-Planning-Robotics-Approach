#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "utils/json/json.hpp"
#include "utils/spline/spline.h"
//#include "utils/matplotlib/matplotlibcpp.h"

#include "map/map.h"
#include "vehicle/vehicle.h"
#include "prediction/predictor.h"
#include "planning/planner.h"
#include "planning/driving_strategy.h"
#include "planning/trajectory_generator.h"

using namespace std;

// inter-process communication
using json = nlohmann::json;

// visualization
//namespace plt = matplotlibcpp;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double mph2mps(double x) { return x * 0.44704; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
    	return "";
  	} else if (b1 != string::npos && b2 != string::npos) {
    	return s.substr(b1, b2 - b1 + 2);
  	}
  	
	return "";
}

/**
    visualize path planning state.

	@param 
	@param
	@return 
*/
void visualize_state(Map &map, const EgoVehicle &ego_vehicle, const Predictor::PredictedObjectList &predicted_object_list) {
	Map::ObjectList object_list;

	// ego vehicle:
	Map::Object ego;
	ego.name = "ego";
	ego.x = vector<double>{ego_vehicle.x};
	ego.y = vector<double>{ego_vehicle.y};
	ego.format = "x";

	object_list.push_back(ego);

	// surrounding objects:
	for (const auto &predicted_object: predicted_object_list) {
		Map::Object object;

		object.name = to_string(predicted_object.id);
		vector<double> predicted_x, predicted_y;
		for (const auto &prediction: predicted_object.prediction) {
			predicted_x.push_back(prediction.x);
			predicted_y.push_back(prediction.y);
		}
		object.x = predicted_x;
		object.y = predicted_y;
		object.format = "--";

		object_list.push_back(object);
	}

	// map.show(object_list);
}

int main() {
	uWS::Hub h;

	bool debug = false;

	// HD map:
	Map map;
	map.read("../data/udacity-map-original.csv");
	
	// ego vehicle:
	EgoVehicle ego_vehicle;

	// predictor:
	Predictor predictor(map);

	// planner:
	double PLANNING_HORIZON = 1.80;
	double PLANNING_INTERVAL = 0.10;
	Planner planner;

	h.onMessage(
			[	
				&debug,
				// HD map:
				&map,
				// ego vehicle:
				&ego_vehicle,
				// prediction:
				&predictor,
				// planning:
				&PLANNING_HORIZON,
				&PLANNING_INTERVAL,
				&planner
			](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				//auto sdata = string(data).substr(0, length);
				//cout << sdata << endl;
				if (length && length > 2 && data[0] == '4' && data[1] == '2') {

					auto s = hasData(data);

					if (s != "") {
						auto j = json::parse(s);
						
						string event = j[0].get<string>();
						
						if (event == "telemetry") {
							// j[1] is the data JSON object
							
							// Main car's localization Data
							ego_vehicle.s = j[1]["s"];
							ego_vehicle.d = j[1]["d"];
							ego_vehicle.x = j[1]["x"];
							ego_vehicle.y = j[1]["y"];
							// heading in degree:
							ego_vehicle.yaw = deg2rad(j[1]["yaw"]);
							// speed in mph:
							ego_vehicle.speed = mph2mps(j[1]["speed"]);

							// Previous path data given to the Planner
							auto previous_trajectory_x = j[1]["previous_path_x"];
							auto previous_trajectory_y = j[1]["previous_path_y"];

							TrajectoryGenerator::Trajectory previous_trajectory{0, {}, {}};
							for (int i = 0; i < previous_trajectory_x.size(); ++i) {
								double x = previous_trajectory_x[i];
								double y = previous_trajectory_y[i];

								previous_trajectory.N += 1; 
								previous_trajectory.x.push_back(x);
								previous_trajectory.y.push_back(y); 
							}
							ego_vehicle.sync_time(previous_trajectory);

							// Previous path's end s and d values 
							double end_path_s = j[1]["end_path_s"];
							double end_path_d = j[1]["end_path_d"];

							// Sensor Fusion Data, a list of all other cars on the same side of the road.
							vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
							
							// rectify round trip behavior:
							if (ego_vehicle.s < 211.649354934692 || ego_vehicle.s > 6520.02080917358) {
								if (ego_vehicle.s < 211.649354934692) {
									ego_vehicle.s += 6945.554;
								}
								
								for (auto &fused_object: sensor_fusion) {
									if (fused_object[5] < 6520.02080917358) {
										fused_object[5] += 6945.554;
									}
								}
							}

							TrajectoryGenerator::Trajectory trajectory;
							// DEBUG: round up testcase
							if (debug) {
								cout << "[Round Up Test]" << endl;

								trajectory.N = 1;
								
								auto position = map.get_world_frame_position_smoothed(6782.5570526123, 6.0);
								trajectory.x.push_back(position[0]);
								trajectory.y.push_back(position[1]);
								
								position = map.get_world_frame_position_smoothed(6828.09141921997, 6.0);
								trajectory.x.push_back(position[0]);
								trajectory.y.push_back(position[1]);
								
								position = map.get_world_frame_position_smoothed(6871.54959487915, 6.0);
								trajectory.x.push_back(position[0]);
								trajectory.y.push_back(position[1]);
								
								position = map.get_world_frame_position_smoothed(6914.14925765991, 6.0);
								trajectory.x.push_back(position[0]);
								trajectory.y.push_back(position[1]);								
								
								debug = false;
							} else {
								if (previous_trajectory.N * ego_vehicle.STEPSIZE < PLANNING_HORIZON - PLANNING_INTERVAL) {
									// Object prediction:
									auto predicted_object_list = predictor.predict(ego_vehicle, sensor_fusion, 0.0, ego_vehicle.STEPSIZE, PLANNING_HORIZON);
									// Planning:
									trajectory = planner.generate_trajectory(map, ego_vehicle, predicted_object_list, ego_vehicle.STEPSIZE, PLANNING_HORIZON);
									// fallback to previous trajectory:
									if (0 == trajectory.N) {
										trajectory = previous_trajectory;
									}
								} else {
									trajectory = previous_trajectory;
								}
							}
								
							// Actuate:
							json msgJson;

							msgJson["next_x"] = trajectory.x;
							msgJson["next_y"] = trajectory.y;

							auto msg = "42[\"control\","+ msgJson.dump()+"]";

							//this_thread::sleep_for(chrono::milliseconds(1000));
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
						}
					} else {
						// Manual driving
						std::string msg = "42[\"manual\",{}]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
			}
		}
		);

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest(
			[](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,size_t, size_t) {
				const std::string s = "<h1>Hello world!</h1>";
				if (req.getUrl().valueLength == 1) {
					res->end(s.data(), s.length());
				} else {
					// i guess this should be done more gracefully?
					res->end(nullptr, 0);
				}
		}
		);

	h.onConnection(
			[&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
			std::cout << "Connected!!!" << std::endl;
		}
		);

	h.onDisconnection(
			[&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
				ws.close();
				std::cout << "Disconnected" << std::endl;
		}
		);

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}
