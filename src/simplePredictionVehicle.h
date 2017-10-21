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
  
  double s_position_at(double t);
  
  vector<double> generate_predictions(int horizon);
};

#endif /* simplePredictionVehicle_h */
