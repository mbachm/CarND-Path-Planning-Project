//
//  utils.h
//  path_planning
//
//  Created by Bachmann, Michael on 18.10.17.
//

#ifndef utils_h
#define utils_h

#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "simplePredictionVehicle.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Static function which returns the lane after given state was realized
// @param direction StateMachineState of the SimplePredictionVehicle
// @param lane Current lane of the SimplePredictionVehicle
// @output Lane after given state was realized
static int calculate_lane_after_state_change(SimplePredictionVehicle::StateMachineState direction, const int lane)
{
  int delta = 0;
  if (direction == SimplePredictionVehicle::LCL && lane > 0)
  {
    delta = -1;
  } else if (direction == SimplePredictionVehicle::LCR && lane < 2)
  {
    delta = 1;
  }
  return lane + delta;
}

// Calculates euclidean distance
// @param x1 First points x position
// @param y1 First points y position
// @param x2 Second points x position
// @param y2 Second points y position
// @output Euclidean distance as double value
double distance(double x1, double y1, double x2, double y2);

// Calculates the closest waypoint
// @param x Current x position of car
// @param y Current y position of car
// @param maps_x Vector of x position of waypoints
// @param maps_y Vector of y position of waypoints
// @output Index of closest waypoint in waypoints/maps_x/maps_y vector
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

// Calculates the next waypoint
// @param x Current x position of car
// @param y Current y position of car
// @param theta Current angel of the car on cartesian map
// @param maps_x Vector of x position of waypoints
// @param maps_y Vector of y position of waypoints
// @output Index of next waypoint to reach in waypoints/maps_x/maps_y vector
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Converts given (x,y) point with angle theta into frenet (s,d) coordinate system
// @param x Current x position of car
// @param y Current y position of car
// @param theta Current angel of the car on cartesian map
// @param maps_x Vector of x position of waypoints
// @param maps_y Vector of y position of waypoints
// @output Frenet (s,d) position of car
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Converts given (s,d) point to cartesian coordinate system point (x,y)
// @param s Current s position of car
// @param d Current d position of car
// @param maps_s Vector of s position of waypoints
// @param maps_x Vector of x position of waypoints
// @param maps_y Vector of y position of waypoints
// @output Cartesian (x,y) position of car
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

// Calculates possible future states depending on the current state and the current lane of the vehicle
// @param currentState Current state
// @param lane Current lane
// @output Vector of possible future states
vector<SimplePredictionVehicle::StateMachineState> predict_successor_states(SimplePredictionVehicle::StateMachineState currentState, const int lane);

// Checks if the path of a vehicle will collide with the given second_vehicle
// @param path_for_test_v Vector with predicted path of vehicle in cartesian coordinate system
// @param current_speed Current speed of car
// @param current_s Current s position of car
// @param second_vehicle The vehicle to check if it will collide with or not
// @param maps_s Vector of s position of waypoints
// @param maps_x Vector of x position of waypoints
// @param maps_y Vector of y position of waypoints
// @output True if car will collide with given other vehicle, false otherwise
bool will_collide(vector<vector<double>> path_for_test_v, const double current_speed, const int current_s, SimplePredictionVehicle second_vehicle, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* utils_h */
