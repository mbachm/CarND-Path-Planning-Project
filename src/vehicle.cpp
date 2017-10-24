//
//  vehicle.cpp
//  path_planning
//
//  Created by Bachmann, Michael (415) on 21.10.17.
//

#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

constexpr double Vehicle::_max_speed;

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int id, int lane) : SimplePredictionVehicle(id, 0, 0, 0.0)
{
  this->lane = lane;
  state = KL;
}

Vehicle::Vehicle(int id, int lane, double s, double d, double speed) : Vehicle(id, lane)
{
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->ref_speed = 0.0; //mph
  this->target_speed = this->_max_speed; // mph
}

Vehicle::~Vehicle() {}

vector<Vehicle::StateMachineState> Vehicle::predict_successor_states(Vehicle::StateMachineState currentState)
{
  vector<Vehicle::StateMachineState> states;
  
  switch (currentState) {
    case KL:
      states.push_back(KL);
      if (this->lane != 0)
      {
        states.push_back(PLCL);
      }
      if (this->lane != 2)
      {
        states.push_back(PLCR);
      }
      break;
    case PLCL:
      states.push_back(LCL);
      break;
    case PLCR:
      states.push_back(LCR);
      break;
    case LCL:
      states.push_back(LCL);
      states.push_back(KL);
      break;
    case LCR:
      states.push_back(LCR);
      states.push_back(KL);
  }
  return states;
}

void Vehicle::update_state(vector<vector<SimplePredictionVehicle>> vehiclePredictions, const bool too_close) {
  /*
   
   */
  
  vector<Vehicle::StateMachineState> states = predict_successor_states(this->state);
  
  //state = "KL"; // this is an example of how you change state.
  vector<double> costs;
  double cost;
  for (Vehicle::StateMachineState test_state : states) {
    cost = 0;
    // create copy of our vehicle
    Vehicle test_v = Vehicle(this-> id, this->lane, this->s, this->d, this->speed);
    test_v.state = test_state;
    test_v.realize_state(vehiclePredictions, too_close);
    // predict one step into future, for selected state
//    vector<int> test_v_state = test_v.state_at(1);
    test_v.generate_predictions();
    int pred_lane = test_v.lane;
    double pred_s = test_v.s;
    double pred_d = test_v.d;
    double pred_v = test_v.speed;
    cout << "pred lane: " << pred_lane << " s: " << pred_s << " d: " << pred_d << " speed: " << pred_v << endl;
    
    cout << "tested state: " << test_state << endl;
    
    // check for collisions
    vector<vector<SimplePredictionVehicle>>::iterator it = vehiclePredictions.begin();
    vector<vector<SimplePredictionVehicle>> in_front;
    while(it != vehiclePredictions.end())
    {
//      int index = it->first;
//      vector<vector<int>> v = it->second;
//      // check predictions one step in future as well
//      if ((v[1][0] == pred_lane) && (abs(v[1][1] - pred_s) <= L) && index != -1) {
//        cout << "coll w/ car: " << index << ", "
//        << v[1][0] << " " << pred_lane << ", "
//        << v[1][1] << " " << pred_s << endl;
//        cost += 1000;
//      }
//      it++;
    }
    
    cost += 1*(pred_v-this->target_speed);
    //cost += 1*(pow(3 - pred_lane, 2));
    //cost += 10*(1 - exp(-abs(pred_lane - 3)/(300 - (double)pred_s)));
    if (pred_lane < 0 || pred_lane > 3) {
      cost += 1000;
    }
    switch (test_state) {
      case PLCL:
        cost += 10*(pow(pred_lane+1- this->goal_lane, 2));
        break;
      case PLCR:
        cost += 10*(pow(pred_lane-1-this->goal_lane, 2));
        break;
      default:
        cost += 4*(pow(pred_lane-this->goal_lane, 2));
        break;
    }
    
    cout << "cost: " << cost << endl;
    costs.push_back(cost);
  }
  double min_cost = 99999;
  int min_cost_index = 0;
  for (int i = 0; i < costs.size(); i++) {
    //cout << "cost[" << i << "]: " << costs[i] << endl;
    if (costs[i] < min_cost) {
      min_cost = costs[i];
      min_cost_index = i;
      
    }
  }
  
  state = states[min_cost_index];
  cout << "chosen state: " << state << endl;
}

void Vehicle::increment(int dt = 1) {
  
  this->s += this->speed * dt;
//  this->speed += this->acceleration * dt;
}

bool Vehicle::collides_with(Vehicle other, int at_time) {
  
  /*
   Simple collision detection.
   */
  int check1 = s_position_at(at_time);
  int check2 = other.s_position_at(at_time);
  return (this->lane == other.lane) && (abs(check1-check2) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
  
  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;
  
  for (int t = 0; t < timesteps+1; t++)
  {
    if( collides_with(other, t) )
    {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }
  
  return collider_temp;
}

void Vehicle::realize_state(vector<vector<SimplePredictionVehicle>> vehiclePredictions, bool too_close) {
  
  /*
   Given a state, realize it by adjusting acceleration and lane.
   Note - lane changes happen instantaneously.
   */
  Vehicle::StateMachineState state = this->state;
  switch (state) {
    case SimplePredictionVehicle::KL:
      realize_keep_lane(vehiclePredictions, too_close);
      break;
    case SimplePredictionVehicle::LCL:
      realize_lane_change(vehiclePredictions, LCL);
      //TODO: change lane to left
      break;
    case SimplePredictionVehicle::LCR:
      realize_lane_change(vehiclePredictions, LCR);
      //TODO: change lane to right
      break;
    case SimplePredictionVehicle::PLCL:
      //TODO: Prepare lane change (adjust speed & wait a bit)
      break;
    case SimplePredictionVehicle::PLCR:
      //TODO: Prepare lane change (adjust speed & wait a bit)
      break;
  }
}

void Vehicle::realize_keep_lane(vector<vector<SimplePredictionVehicle>> predictions, bool too_close) {
  calculateTargetSpeed(predictions, too_close, this->lane, this->s, this->target_speed);
  adjustSpeedWithoutJerk(this->ref_speed, this->target_speed);
}

void Vehicle::realize_lane_change(vector<vector<SimplePredictionVehicle>> predictions, StateMachineState direction) {
  int delta = -1;
  if (direction == LCL)
  {
    delta = 1;
  }
  this->lane += delta;
  calculateTargetSpeed(predictions, false, this->lane, this->s, this->target_speed);
  adjustSpeedWithoutJerk(this->ref_speed, this->target_speed);
}

void Vehicle::realize_prep_lane_change(vector<vector<SimplePredictionVehicle>>, StateMachineState direction)
{
//  int delta = -1;
//  if (direction == PLCL)
//  {
//    delta = 1;
//  }
//  int lane = this->lane + delta;
//
//  map<int, vector<vector<int> > >::iterator it = predictions.begin();
//  vector<vector<vector<int> > > at_behind;
//  while(it != predictions.end())
//  {
//    int v_id = it->first;
//    vector<vector<int> > v = it->second;
//
//    if((v[0][0] == lane) && (v[0][1] <= this->s))
//    {
//      at_behind.push_back(v);
//
//    }
//    it++;
//  }
//  if(at_behind.size() > 0)
//  {
//
//    int max_s = -1000;
//    vector<vector<int> > nearest_behind = {};
//    for(int i = 0; i < at_behind.size(); i++)
//    {
//      if((at_behind[i][0][1]) > max_s)
//      {
//        max_s = at_behind[i][0][1];
//        nearest_behind = at_behind[i];
//      }
//    }
//    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
//    int delta_v = this->speed - target_vel;
//    int delta_s = this->s - nearest_behind[0][1];
//    if(delta_v != 0)
//    {
//
//      int time = -2 * delta_s/delta_v;
//      int a;
//      if (time == 0)
//      {
//        a = this->acceleration;
//      }
//      else
//      {
//        a = delta_v/time;
//      }
//      if(a > this->max_acceleration)
//      {
//        a = this->max_acceleration;
//      }
//      if(a < -this->max_acceleration)
//      {
//        a = -this->max_acceleration;
//      }
//      this->acceleration = a;
//    }
//    else
//    {
//      int my_min_acc = max(-this->max_acceleration,-delta_s);
//      this->acceleration = my_min_acc;
//    }
//  }
}
