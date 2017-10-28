//
//  vehicle.h
//  path_planning
//
//  Created by Bachmann, Michael on 21.10.17.
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
  // maximum speed in mph
  static constexpr double _max_speed = 49.5;
  // True if vehicle is to close to one before, false otherwise
  bool _too_close;

  // Returns maximum speed if no vehicle is ahead, otherwise the speed with a little buffer of the next car ahead of us
  // @param detected_vehicles Vector of detected SimplePredictionVehicle
  // @param s Current s value of our car
  // @param distance_to_check Optional parameter which defines how far ahead our car looks. Default value is 20.0
  // @output Maximum possible speed to drive
  static double getSpeedOfNearestCarInFront(vector<SimplePredictionVehicle> &detected_vehicles, const double s, const double distance_to_check = 20.0) {
    sort(detected_vehicles.begin(), detected_vehicles.end(), SimplePredictionVehicle::sort_by_s_distance);
    for (SimplePredictionVehicle vehicle : detected_vehicles) {
      if (s < vehicle.s && abs(s - vehicle.s) < distance_to_check)
      {
        return vehicle.speed + 5.0;
      }
    }
    return _max_speed;
  }
  
  // Calculates the new target speed of the vehicle with the predicted vehicles ahead with the current lane in mind.
  // @param cars_in_lanes Vector of vector witg detected SimplePredictionVehicle. Each lane has it's own vector
  // @param too_close Value which indicates if the car too close to the nearest one in the current lane
  // @param lane Current lane
  // @param s Current s position
  // @param target_speed Current target_speed. Will be adjusted
  // @output Maximum possible speed to drive
  static void calculateTargetSpeed(vector<vector<SimplePredictionVehicle>> &categorized_vehicles_by_lane, bool too_close, const int lane, const double s, double &target_speed) {
    if (lane == 0 && categorized_vehicles_by_lane[0].size() > 0 && too_close)
    {
      target_speed = getSpeedOfNearestCarInFront(categorized_vehicles_by_lane[0], s);
    }
    else if (lane == 1 && categorized_vehicles_by_lane[1].size() > 0 && too_close)
    {
      target_speed = getSpeedOfNearestCarInFront(categorized_vehicles_by_lane[1], s);
    }
    else if (categorized_vehicles_by_lane[2].size() > 0 && too_close)
    {
      target_speed = getSpeedOfNearestCarInFront(categorized_vehicles_by_lane[2], s);
    } else
    {
      target_speed = _max_speed;
    }
  }
  
  // Adjusted current reference speed without jerk
  // @param ref_speed Current ref_speed
  // @param target_speed Speed to achieve
  static void adjustSpeedWithoutJerk(double &ref_speed, const double target_speed) {
    // Based on Udacity Project Walkthrough and Q&A (https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d)
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
  
  
  // Method to safely copy current vehicle with a new state. Needed in prediction step
  // @param new_state The new state to achieve
  // @output Copied vehicle with new state
  Vehicle copy_vehicle_with(SimplePredictionVehicle::StateMachineState new_state);
  
  // Returns a vector of vector with SimplePredictionVehicle depending on their lane
  // @param sensor_fusion Sensor_fusion data
  // @param prev_size Previous waypoint size. Needed to predict future s value of the car
  // @output vector<vector<SimplePredictionVehicle>> of SimplePredictionVehicle seperated by their current lanes
  vector<vector<SimplePredictionVehicle>> categorize_vehicles_into_lanes(vector<vector<double>> sensor_fusion, const int prev_size);
  
  // Calculates the new path of the vehicle depending on the current state
  // @param prev_size Size of waypoins of the previous given path which had not been passed by the vehicle
  // @param previous_path_x Vector with x position of previous waypoints which had not been passed by the vehicle
  // @param previous_path_y Vector with y position of previous waypoints which had not been passed by the vehicle
  // @output Vector with vectors of x and y with future waypoints
  vector<vector<double>> calculate_path_depending_on_state(const int prev_size, vector<double> previous_path_x, vector<double> previous_path_y);
  
  // Calculates the next best state given on the vehiclePredictions, if we are to close to the nearest other vehicle and the previouspath
  // @param categorized_vehicles_by_lane Detected vehicles categorized by lane
  // @param too_close Boolean value if the car is to close to the nearest one ahead or not
  // @param previous_path_x x values of previous path
  // @param previous_path_y y values of previous path
  void calculate_next_best_state(vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane, const bool too_close, vector<double> previous_path_x, vector<double> previous_path_y);
  
  // Realizes the state which was predicted and saved in calculate_next_best_state
  // @param categorized_vehicles_by_lane Detected vehicles categorized by lane
  // @param too_close Boolean value if the car is to close to the nearest one ahead or not
  void realize_state(vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane, bool too_close);
  
  // Realizes keep lane with given vehicle predictions
  // @param categorized_vehicles_by_lane Detected vehicles categorized by lane
  // @param too_close Boolean value if the car is to close to the nearest one ahead or not
  void realize_keep_lane(vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane, bool too_close);
  
  // Realizes lane change with given direction
  // @param categorized_vehicles_by_lane Detected vehicles categorized by lane
  // @param direction LCL or LCR. Direction in which the vehicle should change lane
  void realize_lane_change(vector<vector<SimplePredictionVehicle>> categorized_vehicles_by_lane, StateMachineState direction);
  
public:
  // Distance of waypoints to calculate path with
  static constexpr int waypoints_distance = 30;
  // Current x position
  double x;
  // Current y position
  double y;
  //Current angle on cartesian map
  double yaw;
  // Given x position of map_waypoints
  vector<double> map_waypoints_x;
  // Given y position of map_waypoints
  vector<double> map_waypoints_y;
  // Given s position of map_waypoints
  vector<double> map_waypoints_s;
  // Given dx position of map_waypoints
  vector<double> map_waypoints_dx;
  // Given dy position of map_waypoints
  vector<double> map_waypoints_dy;
  // Current state
  StateMachineState state;
  // Speed to achieve
  double target_speed;
  
  // Constructor
  Vehicle(int id, int lane);
  // Convenience constructor
  Vehicle(int id, int lane, vector<vector<double>> waypoints);
  // Convenience constructor
  Vehicle(int id, int lane, double s, double d, double speed, vector<vector<double>> waypoints);
  
  // Destructor
  virtual ~Vehicle();

  //
  vector<vector<double>> plan_path(vector<vector<double>> sensor_fusion, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
  
};

#endif /* vehicle_h */
