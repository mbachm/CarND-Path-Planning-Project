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
  void predictOwnLane();
  void predictOwnState();
  
public:
  enum StateMachineState {KL, PLCL, PLCR, LCL, LCR};
  int lane;
  int s;
  int d;
  int speed;
  int acceleration;
  StateMachineState predictedState;
  
  /**
   * Constructor
   */
  SimplePredictionVehicle(int s, int d, int speed, int acceleration);
  
  /**
   * Destructor
   */
  virtual ~SimplePredictionVehicle();
  
  vector<int> state_at(int t);
  
  vector<vector<int> > generate_predictions(int horizon);
};

#endif /* simplePredictionVehicle_h */
