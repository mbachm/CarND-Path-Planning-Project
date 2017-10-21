//
//  vehicle.hpp
//  path_planning
//
//  Created by Bachmann, Michael (415) on 21.10.17.
//

#ifndef vehicle_h
#define vehicle_h

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "simplePredictionVehicle.h"

using namespace std;

class Vehicle: public SimplePredictionVehicle{
public:
  
  struct collider{
    
    bool collision ; // is there a collision?
    int  time; // time collision happens
    
  };
  
  int L = 1;
  
  int preferred_buffer = 6; // impacts "keep lane" behavior.
  
  int target_speed;
  
  int lanes_available;
  
  int max_acceleration;
  
  int goal_lane;
  
  int goal_s;
  
  StateMachineState state;
  
  /**
   * Constructor
   */
  Vehicle(int lane, int s, int d, int speed, int acceleration);
  
  /**
   * Destructor
   */
  virtual ~Vehicle();
  
  vector<StateMachineState> predict_successor_states(StateMachineState currentState);
  
  void update_state(map<int, vector <vector<int> > > predictions);
  
  void configure(vector<int> road_data);
  
  void increment(int dt);
  
  vector<int> state_at(int t);
  
  bool collides_with(Vehicle other, int at_time);
  
  collider will_collide_with(Vehicle other, int timesteps);
  
  void realize_state(map<int, vector < vector<int> > > predictions);
  
  void realize_constant_speed();
  
  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);
  
  void realize_keep_lane(map<int, vector< vector<int> > > predictions);
  
  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);
  
  void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);
  
  vector<vector<int> > generate_predictions(int horizon);
  
};

#endif /* vehicle_h */
