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

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

vector<SimplePredictionVehicle::StateMachineState> predict_successor_states(SimplePredictionVehicle::StateMachineState currentState, const int lane);

bool will_collide(vector<vector<double>> path_for_test_v, const double current_speed, const int current_s, SimplePredictionVehicle second_vehicle, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* utils_h */
