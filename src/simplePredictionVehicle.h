//
//  SimplePredictionVehicle.hpp
//  path_planning
//
//  Created by Bachmann, Michael (415) on 21.10.17.
//

#ifndef simplePredictionVehicle_h
#define simplePredictionVehicle_h

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class SimplePredictionVehicle {
private:
  void predict_current_lane();
  
public:
  enum StateMachineState {KL, PLCL, PLCR, LCL, LCR};
  int id;
  int lane;
  double s;
  double d;
  double speed;
  StateMachineState predictedState;
  
  /**
   * Constructor
   */
  SimplePredictionVehicle(int id, double s, double d, double speed);
  
  /**
   * Destructor
   */
  virtual ~SimplePredictionVehicle();
  
  static bool sort_by_s_distance(const SimplePredictionVehicle &a, const SimplePredictionVehicle &b)
  {
    return a.s < b.s;
  };
  
  double s_position_at(double t);
  
  vector<double> generate_predictions(int horizon);
  
  void predict_current_state(double x, double y, double vx, double vy, double car_yaw, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y);
};

#endif /* simplePredictionVehicle_h */
