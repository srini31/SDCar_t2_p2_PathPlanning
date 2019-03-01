#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
	
//#define TEST1
//#define TEST2

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;
	//gloabl variables
	int lane = 1; //start in middle lane
	double ref_vel = 0; //49.5 will cause jerk exceeded flag

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy]
		(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];
					//vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
					//for each car (outer vector), all sensor fusion values are in the inner vector
					json msgJson;
					//actual x,y points that are used by the path planner
					vector<double> next_x_vals;
					vector<double> next_y_vals;
					/**
					 * TODO: define a path made up of (x,y) points that the car will visit
					 *   sequentially every .02 seconds
					 */
					/////////////////////////////////////////////////
					#ifdef TEST1
					//move in a straight line
					double dist_inc = 0.5;
					for (int i = 0; i < 50; ++i) {
						next_x_vals.push_back(car_x + (dist_inc*i)*cos(deg2rad(car_yaw)));
						next_y_vals.push_back(car_y + (dist_inc*i)*sin(deg2rad(car_yaw)));
					}
					#endif

					#ifdef TEST2
					//moves the car in a circle
					int prev_size = previous_path_x.size(); //previous path of the car in last run
				 //create a list of points spaces X distance apart and interpolate with a spline
					vector<double> ptsx;
					vector<double> ptsy;
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					//if prev_size is empty start from car position, get one more previous position
					if (prev_size < 2) {
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_x - sin(car_yaw);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);
						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					} else {
						//reference = prev car end point state
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];

						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						double ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev)
							ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);
						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
					}
					#endif
					// num of points in previous path
					int prev_size = previous_path_x.size();

					// use the car_s from previous path, if there are points left unconsumed by the car, this is the current position of the car
					// the simulator updates with certain freq, but we just go with the previous_path
					//there is a lag between program execution -> simulator input -> simulator reporting current position
					if (prev_size > 0) {
						car_s = end_path_s;
					}
					bool too_close = false, apply_brake = false;

					//find ref_velocity to use
					for (int i = 0; i < sensor_fusion.size(); i++) { //for each car
						float d = sensor_fusion[i][6];
						//int car_lane = -1;

						//lane -- indicates which lane our car is in 
						//lane=0 --> 2+4*0-2 = 0 && 2+4*0+2 = 4 -- it means the other car is in the same lane (lane 0, left most lane)
						//lane=1 --> 2+4*1-2 = 4 && 2+4*1+2 = 8 -- it means the other car is in the same lane (lane 1, middle lane)
						if ((d > 2 + 4 * lane - 2) && (d < 2 + 4 * lane + 2)) {
							// Find car speed.
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx*vx + vy * vy); //velocity magnitude
							double check_car_s = sensor_fusion[i][5];
							//our car, car_s is still at the end of previous path, not where simulator reports, 
							//check_car_s is where the simulator reports the other car
							//prev_size*0.02 == prev_size points left to go for our  car, 0.02 is the delta_t 
							//prev_size*0.02*check_speed is the distance this other car would cover in that time
							//add this distance to the other cars distance, this is the actual distance this other car is from our car
							check_car_s += ((double)prev_size*0.02*check_speed);//if using previous points, can project s value out in time
							
							//calculate cost of staying in lane -- distance between cars is the cost assumed here
							//no cost function is applied, as goal lane is not specified and time to goal is not specified
							//check for s values greater than our car and the s gap
							if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
								//lower the reference velocity to avoid crash
								//ref_vel = 29.5;//mph
								too_close = true;
							}

							if ((check_car_s > car_s) && ((check_car_s - car_s) < 10)) {
								apply_brake = true;
							}
						} // if ((d > 2 + 4 * lane - 2) && (d < 2 + 4 * lane + 2))
					} //for (int i = 0; i < sensor_fusion.size(); i++) //for each car

					/* //test new logic below instead of the Q & A logic
					if (too_close) { 
						ref_vel -= .224; // slow down
					} else if(ref_vel < 49.5) {
						ref_vel += .224; //speed up if under 50
					} */
					//sometimes the car ahead brakes and the gradual slow down results in an accident
					//in this scenario, slow down quickly
					if (apply_brake) {
						ref_vel -= 2.24; // slow down very quickly and stay in lane 
						apply_brake = false;
					}
					///////////////////
					//Finite state machine - change lanes as required
					//add lane change logic here to change lanes, PLCL, PLCR, LCL, LCR
					bool can_change = false;
					if (too_close) {
						//loop through the cars and see if its possible to switch lanes
						int new_lane = get_successor_state(lane);
						//check if other lanes are open, if no option available slow down
						can_change = can_change_lane(sensor_fusion, new_lane, prev_size, car_s);

						if (can_change) {
							lane = new_lane; //"LCL" or "LCR"
							//updating current lane will update the spline automatically
						}	else { //"KL"
							ref_vel -= .224; // slow down and stay in lane 
						}
					} else if(ref_vel < 49.5) {
						ref_vel += .224; //speed up if speed is under 50 and not too close
					}
					
					//create a list of waypoints spaced at 30m, these are interpolated by a spline
					vector<double> ptsx;
					vector<double> ptsy;

					//reference x, y, yaw states - either at current position or last point of previous path
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					// if previous points are almost empty, use the cars current position
					//using the previous points will ensure car moves smoothly without jumping to a new point
					if (prev_size < 2) {
						// get two poits based on the car driving angle
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);

						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);

						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					} else {
						// use the end points of previous path as reference
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];
						//here two points are already availale from the vector
						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev); //back calculate from the points

						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);

						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
					}
					
					// In Frenet, add 30m spaced points ahead of current reference point
					//get x,y location of those points and add to ptsx, ptsy
					//2+4*lane means -> 2 is center of left lane (lane=0), 6=center of 2nd lane(lane=1), 10=center of 3rd lane(lane=2)
					vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					for (int i = 0; i < ptsx.size(); i++) {
						//shift car reference angle to 0 degrees
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;

						ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
						ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
					}

					// Create a spline.
					tk::spline s;
					s.set_points(ptsx, ptsy); //set input points for the spline

					//actual x,y points that are used by the path planner
					//vector<double> next_x_vals, next_y_vals;
					//add all points from previous path for continuity
					for (int i = 0; i < prev_size; i++) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					// adjust spacing of spline points to travel at required velocity
					//car moves 50 times/sec or once every 0.02 sec
					double target_x = 30.0;
					double target_y = s(target_x); //get y position from spline equation for x=30 m
					//length of the hypotenuse for given x,y is the distance to the point 30m away
					double target_dist = sqrt(target_x*target_x + target_y * target_y);

					double x_add_on = 0;
					//prev_size = previous_path_x.size() 
					//50 - prev_size because we only need 50 points, and some are not yet consumed in the previous path
					//for (int i = 1; i < 50 - prev_size; i++) {
					for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
						
						//the ref_vel is adjusted over time, it affects the number of way points
						//higher ref_vel meands lower N points -- same as moving faster between points
						double N = target_dist / (0.02*ref_vel / 2.24);
						double x_point = x_add_on + target_x / N;
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						//rotate reference points back to original position
						x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
						y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

						x_point += ref_x; //ref_x == double car_x = j[1]["x"];
						y_point += ref_y; //ref_y == double car_y = j[1]["y"];

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}
					/* code that rotated the points earlier -- reference
					for (int i = 0; i < ptsx.size(); i++) {
						//shift car reference angle to 0 degrees
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;

						ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
						ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
					} */
					//////////////////////////////////////////////////
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}  // end websocket if
	}); // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
		char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}