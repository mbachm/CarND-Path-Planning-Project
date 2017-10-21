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

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int d, int speed, int acceleration) :SimplePredictionVehicle(s, d, speed, acceleration){
  
  this->lane = lane;
  state = KL;
  max_acceleration = -1;
  
}

Vehicle::~Vehicle() {}

vector<Vehicle::StateMachineState> Vehicle::predict_successor_states(Vehicle::StateMachineState currentState)
{
  vector<Vehicle::StateMachineState> states;
  
  switch (currentState) {
    case KL:
      states.push_back(KL);
      states.push_back(PLCL);
      states.push_back(PLCR);
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

void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
  /*
   
   */
  
  vector<Vehicle::StateMachineState> states = predict_successor_states(this->state);
  
  //state = "KL"; // this is an example of how you change state.
  vector<double> costs;
  double cost;
  for (Vehicle::StateMachineState test_state : states) {
    cost = 0;
    // create copy of our vehicle
    Vehicle test_v = Vehicle(this->lane, this->s, this->d, this->speed, this->acceleration);
    test_v.state = test_state;
    test_v.realize_state(predictions);
    // predict one step into future, for selected state
    vector<int> test_v_state = test_v.state_at(1);
    int pred_lane = test_v_state[0];
    int pred_s = test_v_state[1];
    int pred_v = test_v_state[2];
    int pred_a = test_v_state[3];
    cout << "pred lane: " << pred_lane << " s: " << pred_s << " v: " << pred_v << " a: " << pred_a << endl;
    
    cout << "tested state: " << test_state << endl;
    
    // check for collisions
    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
      int index = it->first;
      vector<vector<int>> v = it->second;
      // check predictions one step in future as well
      if ((v[1][0] == pred_lane) && (abs(v[1][1] - pred_s) <= L) && index != -1) {
        cout << "coll w/ car: " << index << ", "
        << v[1][0] << " " << pred_lane << ", "
        << v[1][1] << " " << pred_s << endl;
        cost += 1000;
      }
      it++;
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

void Vehicle::configure(vector<int> road_data) {
  /*
   Called by simulator before simulation begins. Sets various
   parameters which will impact the ego vehicle.
   */
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}

void Vehicle::increment(int dt = 1) {
  
  this->s += this->speed * dt;
  this->speed += this->acceleration * dt;
}

vector<int> Vehicle::state_at(int t) {
  
  /*
   Predicts state of vehicle in t seconds (assuming constant acceleration)
   */
  int s = this->s + this->speed * t + this->acceleration * t * t / 2;
  int v = this->speed + this->acceleration * t;
  //TODO: Check this
  return {this->lane, s, v, this->acceleration};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {
  
  /*
   Simple collision detection.
   */
  vector<int> check1 = state_at(at_time);
  vector<int> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
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

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
  
  /*
   Given a state, realize it by adjusting acceleration and lane.
   Note - lane changes happen instantaneously.
   */
  Vehicle::StateMachineState state = this->state;
  //TODO:
  /*
  if(state.compare("CS") == 0)
  {
    realize_constant_speed();
  }
  else if(state.compare("KL") == 0)
  {
    realize_keep_lane(predictions);
  }
  else if(state.compare("LCL") == 0)
  {
    realize_lane_change(predictions, "L");
  }
  else if(state.compare("LCR") == 0)
  {
    realize_lane_change(predictions, "R");
  }
  else if(state.compare("PLCL") == 0)
  {
    realize_prep_lane_change(predictions, "L");
  }
  else if(state.compare("PLCR") == 0)
  {
    realize_prep_lane_change(predictions, "R");
  }*/
  
}

void Vehicle::realize_constant_speed() {
  acceleration = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {
  
  int delta_v_til_target = target_speed - speed;
  int max_acc = min(max_acceleration, delta_v_til_target);
  
  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > in_front;
  while(it != predictions.end())
  {
    
    int v_id = it->first;
    
    vector<vector<int> > v = it->second;
    
    if((v[0][0] == lane) && (v[0][1] > s))
    {
      in_front.push_back(v);
      
    }
    it++;
  }
  
  if(in_front.size() > 0)
  {
    int min_s = 1000;
    vector<vector<int>> leading = {};
    for(int i = 0; i < in_front.size(); i++)
    {
      if((in_front[i][0][1]-s) < min_s)
      {
        min_s = (in_front[i][0][1]-s);
        leading = in_front[i];
      }
    }
    
    int next_pos = leading[1][1];
    int my_next = s + this->speed;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }
  
  return max_acc;
  
}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
  this->acceleration = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
  int delta = -1;
  if (direction.compare("L") == 0)
  {
    delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->acceleration = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
  int delta = -1;
  if (direction.compare("L") == 0)
  {
    delta = 1;
  }
  int lane = this->lane + delta;
  
  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > at_behind;
  while(it != predictions.end())
  {
    int v_id = it->first;
    vector<vector<int> > v = it->second;
    
    if((v[0][0] == lane) && (v[0][1] <= this->s))
    {
      at_behind.push_back(v);
      
    }
    it++;
  }
  if(at_behind.size() > 0)
  {
    
    int max_s = -1000;
    vector<vector<int> > nearest_behind = {};
    for(int i = 0; i < at_behind.size(); i++)
    {
      if((at_behind[i][0][1]) > max_s)
      {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }
    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    int delta_v = this->speed - target_vel;
    int delta_s = this->s - nearest_behind[0][1];
    if(delta_v != 0)
    {
      
      int time = -2 * delta_s/delta_v;
      int a;
      if (time == 0)
      {
        a = this->acceleration;
      }
      else
      {
        a = delta_v/time;
      }
      if(a > this->max_acceleration)
      {
        a = this->max_acceleration;
      }
      if(a < -this->max_acceleration)
      {
        a = -this->max_acceleration;
      }
      this->acceleration = a;
    }
    else
    {
      int my_min_acc = max(-this->max_acceleration,-delta_s);
      this->acceleration = my_min_acc;
    }
    
  }
  
}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {
  
  vector<vector<int> > predictions;
  for( int i = 0; i < horizon; i++)
  {
    vector<int> check1 = state_at(i);
    vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;
  
}
