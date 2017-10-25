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
private:
  static constexpr double _max_speed = 49.5; //mph
  bool _too_close;
  struct collider{
    bool collision ; // is there a collision?
    int  time; // time collision happens
  };
  
  static void getSpeedOfNearestCarInFront(vector<SimplePredictionVehicle> &detected_vehicles, const double s, double &target_speed) {
    sort(detected_vehicles.begin(), detected_vehicles.end(), SimplePredictionVehicle::sort_by_s_distance);
    for (SimplePredictionVehicle vehicle : detected_vehicles) {
      if (s < vehicle.s)
      {
        target_speed = vehicle.speed;
        break;
      }
    }
  }
  
  static void calculateTargetSpeed(vector<vector<SimplePredictionVehicle>> &cars_in_lanes, bool too_close, const int lane, const double s, double &target_speed) {
    if (lane == 0 && cars_in_lanes[0].size() > 0 && too_close)
    {
      //            cout << "reducing speed due to lane 0" << endl << endl;
      getSpeedOfNearestCarInFront(cars_in_lanes[0], s, target_speed);
    }
    else if (lane == 1 && cars_in_lanes[1].size() > 0 && too_close)
    {
      //            cout << "reducing speed due to lane 1" << endl << endl;
      getSpeedOfNearestCarInFront(cars_in_lanes[1], s, target_speed);
    }
    else if (cars_in_lanes[2].size() > 0 && too_close)
    {
      //            cout << "reducing speed due to lane 2" << endl << endl;
      getSpeedOfNearestCarInFront(cars_in_lanes[2], s, target_speed);
    } else
    {
      //            cout << "set target speed to max speed" << endl << endl;
      target_speed = _max_speed;
    }
  }
  
  static void adjustSpeedWithoutJerk(double &ref_speed, const double target_speed) {
    if (ref_speed > target_speed)
    {
      ref_speed -= 0.112;
      if ((ref_speed - target_speed) < 0.112) {
        ref_speed = target_speed;
      }
    }
    else if(ref_speed != target_speed && ref_speed < _max_speed)
    {
      ref_speed += 0.224;
    }
  }
  
  vector<vector<SimplePredictionVehicle>> categorize_vehicles_into_lanes(vector<vector<double>> sensor_fusion, const int prev_size);
  vector<vector<double>> calculate_path_depending_on_state(const int prev_size, vector<double> previous_path_x, vector<double> previous_path_y);
  void update_state(vector<vector<SimplePredictionVehicle>> vehiclePredictions, const bool too_close);
  bool collides_with(SimplePredictionVehicle other, int at_time);
  collider will_collide_with(SimplePredictionVehicle other, int timesteps);
  void realize_state(vector<vector<SimplePredictionVehicle>> vehiclePredictions, bool too_close);
  void realize_keep_lane(vector<vector<SimplePredictionVehicle>> predictions, bool too_close);
  void realize_lane_change(vector<vector<SimplePredictionVehicle>> predictions, StateMachineState direction);
  
public:
  
  double x;
  double y;
  double yaw;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  StateMachineState state;
  
  double target_speed;
  int goal_lane;
  int goal_s;
  
  
  /**
   * Constructors
   */
  Vehicle(int id, int lane);
  Vehicle(int id, int lane, vector<vector<double>> waypoints);
  Vehicle(int id, int lane, double s, double d, double speed);
  
  /**
   * Destructor
   */
  virtual ~Vehicle();
  
  /**
   *
   */
  vector<vector<double>> plan_path(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
  
};

#endif /* vehicle_h */
