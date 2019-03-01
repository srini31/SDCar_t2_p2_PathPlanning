#pragma once
#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

// Helper functions related to waypoints and converting from XY to Frenet or vice versa
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
	const vector<double> &maps_y) {
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); ++i) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
	const vector<double> &maps_y) {
	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = std::min(2 * pi() - angle, angle);

	if (angle > pi() / 2) {
		++closestWaypoint;
		if (closestWaypoint == maps_x.size()) {
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
	const vector<double> &maps_x,
	const vector<double> &maps_y) {
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y * n_y) / (n_x*n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; ++i) {
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return { frenet_s,frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
	const vector<double> &maps_x,
	const vector<double> &maps_y) {
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
		++prev_wp;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
		(maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return { x,y };
}

// get a possible successor state/lane from the current lane
int get_successor_state(int lane) {
	// Provides the possible next states given the current state for the FSM 
	int state;
	//states.push_back("KL");

	if (lane == 0) { 				//if no car in right lane, change to lane =1 (right)
		state = 1; //"LCR"
	}
	else if (lane == 1) { 	//change to lane = 0 (left) or lane = 1 (right)
    //randomly pick a state
		int randNum = rand() % 2; // Generate a random number between 0 and 1
		if (randNum == 0) {
			state = 0; //"LCL"
		} else	if (randNum == 1) {
			state = 2; //"LCR"
		}
	}
	else if (lane == 2) { 	//change to lane = 1 (left)
		state = 1; //"LCL"
	}
	return state;
}

//determine if changing to the new lane is possible
//change lane if there is no car within 30 m (ahead/back, using abs) from our car in the new lane
bool can_change_lane(vector<vector<double>> sensor_fusion, int lane, int prev_size, double car_s) {

	bool change_lane = true;

	for (int i = 0; i < sensor_fusion.size(); i++) { //for each car
		float d = sensor_fusion[i][6];

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

			//if any car is close to our car in this lane, cannot change lane
			if (abs(check_car_s - car_s) < 30) {
				change_lane = false;
				break; // no need to check further
			}

		} // if ((d > 2 + 4 * lane - 2) && (d < 2 + 4 * lane + 2))
	} //for (int i = 0; i < sensor_fusion.size(); i++) //for each car
	return change_lane;
}

#endif  // HELPERS_H