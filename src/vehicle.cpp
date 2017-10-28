//
//  vehicle.cpp
//  path_planning
//
//  Created by Bachmann, Michael on 21.10.17.
//

#include <iostream>
#include "vehicle.h"
#include "spline.h"
#include "utils.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

constexpr double Vehicle::_max_speed;
constexpr int Vehicle::waypoints_distance;

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int id, int lane) : SimplePredictionVehicle(id, 0, 0, 0.0)
{
  this->lane = lane;
  state = KL;
  _too_close = false;
}

Vehicle::Vehicle(int id, int lane, vector<vector<double>> waypoints) : Vehicle(id, lane)
{
  this->map_waypoints_x = waypoints[0];
  this->map_waypoints_y = waypoints[1];
  this->map_waypoints_s = waypoints[2];
  this->map_waypoints_dx = waypoints[3];
  this->map_waypoints_dy = waypoints[4];
}

Vehicle::Vehicle(int id, int lane, double s, double d, double speed, vector<vector<double>> waypoints) : Vehicle(id, lane, waypoints)
{
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->ref_speed = 0.0; //mph
  this->target_speed = this->_max_speed; // mph
}

Vehicle::~Vehicle() {}

Vehicle Vehicle::copy_vehicle_with(SimplePredictionVehicle::StateMachineState new_state)
{
  vector<vector<double>> waypoints = {this->map_waypoints_x, this->map_waypoints_y, this->map_waypoints_s, this->map_waypoints_dx, this->map_waypoints_dy};
  Vehicle copy_vehicle = Vehicle(this-> id, calculate_lane_after_state_change(new_state, this->lane), this->s, this->d, this->speed, waypoints);
  copy_vehicle.x = this->x;
  copy_vehicle.y = this->y;
  copy_vehicle.yaw = this->yaw;
  return copy_vehicle;
}

vector<vector<SimplePredictionVehicle>> Vehicle::categorize_vehicles_into_lanes(vector<vector<double>> sensor_fusion, const int prev_size)
{
  vector<SimplePredictionVehicle> lane_0_detected_vehicles;
  vector<SimplePredictionVehicle> lane_1_detected_vehicles;
  vector<SimplePredictionVehicle> lane_2_detected_vehicles;
  this->_too_close = false;
  //find ref_v to use
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    double s = sensor_fusion[i][5];
    double d = sensor_fusion[i][6];
    if ((d < 0.0) && !(-15.0 < s - this->s < ((double)3*waypoints_distance)))
    {
      //ignore vehicles on other side of road and vehicle that are too far behind/away
      continue;
    }
    int id = sensor_fusion[i][0];
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    SimplePredictionVehicle new_vehicle = SimplePredictionVehicle(id, s, d, check_speed);
    new_vehicle.predict_current_state(x, y, vx, vy, this->yaw, map_waypoints_x, map_waypoints_y);
    vector<double> predictions = new_vehicle.generate_predictions();
    
    if (new_vehicle.lane == 0)
    {
      lane_0_detected_vehicles.push_back(new_vehicle);
    }
    else if (new_vehicle.lane == 1)
    {
      lane_1_detected_vehicles.push_back(new_vehicle);
    }
    else if (new_vehicle.lane == 2)
    {
      lane_2_detected_vehicles.push_back(new_vehicle);
    }
    
    if(this->lane == new_vehicle.lane)
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      
      s+=((double)prev_size*0.02*check_speed); //if using previous points can project s value outwars in time
      
      //check s values greater than mine and smaller gap than 30 m
      if (s > this->s && s-this->s < 30)
      {
        this->_too_close = true;
      }
    }
  }
  return {lane_0_detected_vehicles, lane_1_detected_vehicles, lane_2_detected_vehicles};
}

vector<vector<double>> Vehicle::calculate_path_depending_on_state(const int prev_size, vector<double> previous_path_x, vector<double> previous_path_y)
{
  
  // list of spaced (x,y) points, evenly spaced 30 m
  // will interpolate waypoints with spline
  vector<double> ptsx;
  vector<double> ptsy;
  
  // reference x, y , yaw
  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = deg2rad(this->yaw);
  
  // if path is empty, use currenct location as starting reference
  if(prev_size < 2)
  {
    double prev_car_x = this->x - cos(this->yaw);
    double prev_car_y = this->y - sin(this->yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(this->x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(this->y);
  }
  else
  {
    // Redefine reference state as previous path and points
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
    
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    
    //Use two points to make the path tangent to the prevoius path points
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  // Add evenly waypoints in Frenet
  vector<double> next_wp0 = getXY(this->s+waypoints_distance, (2+4*this->lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(this->s+2*waypoints_distance, (2+4*this->lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(this->s+3*waypoints_distance, (2+4*this->lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  //transform to local car coordinates
  for(int i = 0; i < ptsx.size(); i++)
  {
    //shift car reference angle to 0 degrees
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;
    
    ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
  }
  
  //create a spline
  tk::spline spline;
  
  // set (x,y) points to the spline
  spline.set_points(ptsx, ptsy);
  
  //define the actual (x,y) points we use for planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  for(int i = 0; i<prev_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  //Calculate how to break up spline points so thate we travel at our desired reference velocity
  double target_x = 30.0; //horizon point
  double target_y = spline(target_x);
  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
  
  double x_add_on = 0;
  
  //fill up the rest of our path planner
  for (int i = 1; i <= 30-prev_size; i++)
  {
    // N * 0.02 * velocity = d, N = number of points
    double N = (target_dist/(0.02*this->ref_speed/2.24));
    double x_point = x_add_on+(target_x)/N;
    double y_point = spline(x_point);
    
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    //rotate back to normal after rotation it earlier
    x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));
    
    x_point += ref_x;
    y_point += ref_y;
    
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  return {next_x_vals, next_y_vals};
}

void Vehicle::calculate_next_best_state(vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane, const bool too_close, vector<double> previous_path_x, vector<double> previous_path_y) {
  vector<Vehicle::StateMachineState> states = predict_successor_states(this->state, this->lane);
  
  vector<double> costs;
  vector<vector<double>> waypoints = {this->map_waypoints_x, this->map_waypoints_y, this->map_waypoints_s, this->map_waypoints_dx, this->map_waypoints_dy};
  for (Vehicle::StateMachineState test_state : states) {
    double cost = 0.0;
    // create copy of our vehicle
    Vehicle test_v = this->copy_vehicle_with(test_state);
    test_v.state = test_state;
    vector<vector<double>> path_for_test_v = test_v.calculate_path_depending_on_state(previous_path_x.size(), previous_path_x, previous_path_y);
    
    // check for collisions
    vector<SimplePredictionVehicle> predictions_to_check = categorized_vehicles_by_lane[test_v.lane];
    for (SimplePredictionVehicle second_vehicle : predictions_to_check)
    {
      if(will_collide(path_for_test_v, test_v.speed, test_v.s, second_vehicle, map_waypoints_s, map_waypoints_x, map_waypoints_y))
      {
        cost += 1000.0;
      }
    }
    
    double pred_v = getSpeedOfNearestCarInFront(predictions_to_check, test_v.s, 2*waypoints_distance);
    
    cost += 2*this->_max_speed-pred_v;
    cost += 3*abs(this->lane-test_v.lane);
    if (test_v.lane < 0 || test_v.lane > 3) {
      cost += 1000.0;
    }
    
    costs.push_back(cost);
  }
  double min_cost = 99999.0;
  int min_cost_index = 0;
  for (int i = 0; i < costs.size(); i++) {
    if (costs[i] < min_cost) {
      min_cost = costs[i];
      min_cost_index = i;
    }
  }

  this->state = states[min_cost_index];
  realize_state(categorized_vehicles_by_lane, too_close);
}

void Vehicle::realize_state(vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane, bool too_close) {
  
  /*
   Given a state, realize it by adjusting acceleration and lane.
   Note - lane changes happen instantaneously.
   */
  Vehicle::StateMachineState state = this->state;
  switch (state) {
    case SimplePredictionVehicle::KL:
      realize_keep_lane(categorized_vehicles_by_lane, too_close);
      break;
    case SimplePredictionVehicle::LCL:
      realize_lane_change(categorized_vehicles_by_lane, LCL);
      break;
    case SimplePredictionVehicle::LCR:
      realize_lane_change(categorized_vehicles_by_lane, LCR);
      break;
  }
}

void Vehicle::realize_keep_lane(vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane, bool too_close) {
  calculateTargetSpeed(categorized_vehicles_by_lane, too_close, this->lane, this->s, this->target_speed);
  adjustSpeedWithoutJerk(this->ref_speed, this->target_speed);
}

void Vehicle::realize_lane_change(vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane, StateMachineState direction) {
  this->lane = calculate_lane_after_state_change(direction, this->lane);
  calculateTargetSpeed(categorized_vehicles_by_lane, false, this->lane, this->s, this->target_speed);
  adjustSpeedWithoutJerk(this->ref_speed, this->target_speed);
}

vector<vector<double>> Vehicle::plan_path(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d)
{
  
  int prev_size = previous_path_x.size();
  if(prev_size > 0)
  {
    this->s = end_path_s;
  }
  if (this->state != KL)
  {
    if ((this->lane == 0 && 0.0 < this->d < 3.0) || (this->lane == 1 && 5.0 < this->d < 7.0) || (this->lane == 2 && 9.0 < this->d < 12.0)) {
      this->state = KL;
    }
  }
  vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane = categorize_vehicles_into_lanes(sensor_fusion, prev_size);
  calculate_next_best_state(categorized_vehicles_by_lane, this->_too_close, previous_path_x, previous_path_y);
  return calculate_path_depending_on_state(prev_size, previous_path_x, previous_path_y);
}
