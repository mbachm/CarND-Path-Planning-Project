//
//  SimplePredictionVehicle.h
//  path_planning
//
//  Created by Bachmann, Michael on 21.10.17.
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
  // Term3 simulator works with 0.02 timestep
  static constexpr double _simulator_time_step = 0.02;
  // Predicts the current lane of the vehicle depending of its frenet d value
  void predict_current_lane();
  
public:
  // State of a vehicle: KL
  // KL = Keep lane
  // LCL = Lane change left
  // LCR = Lane change right
  enum StateMachineState {KL, LCL, LCR};
  // Id of the car
  int id;
  // Current lane
  int lane;
  // Current frenet s value
  double s;
  // Current frenet d value
  double d;
  // Current speed
  double speed;
  // Speed to target
  double ref_speed;
  // Current state of the vehicle. See StateMachineState explanation
  StateMachineState predictedState;
  
  // Constructor
  // @param id The id of the SimplePredictionVehicle
  // @param s Frenet s position of the SimplePredictionVehicle
  // @param d Frenet d position of the SimplePredictionVehicle
  // @param speed Current speed of the SimplePredictionVehicle
  SimplePredictionVehicle(int id, double s, double d, double speed);
  
  
  // Destructor
  virtual ~SimplePredictionVehicle();
  
  // Static function which returns true if left hand parameter is lower than the right hand one
  // @param a A SimplePredictionVehicle
  // @param b A SimplePredictionVehicle
  // @output If left hands frenet s position is smaller true, false otherwise
  static bool sort_by_s_distance(const SimplePredictionVehicle &a, const SimplePredictionVehicle &b)
  {
    return a.s < b.s;
  };
  
  // Predicts the s position of the SimplePredictionVehicle when the time t elapsed
  // @param t Time to pass
  // @output Future frenet s position of the vehicle
  double s_position_at(double t);
  
  // Generates a predicition of the future path with frenet s coordinates of the SimplePredictionVehicle
  // @param horizon Steps to predict into the future. Default value is 50
  // @output Vector<double> of future frenet s position of the vehicle
  vector<double> generate_predictions(int horizon = 50);
  
  // @param id The id of the SimplePredictionVehicle
  // @param x x position of the SimplePredictionVehicle according to the map
  // @param y y position of the SimplePredictionVehicle according to the map
  // @param vx Change of the x position of the SimplePredictionVehicle according to the map
  // @param vy Change of the y position of the SimplePredictionVehicle according to the map
  // @param car_yaw Current angle of the car according to the map
  // @param map_waypoints_x The x position of given waypoints of the track. Needed for provided `getFrenet` or `getXY` function of utils
  // @param map_waypoints_y The y position of given waypoints of the track. Needed for provided `getFrenet` or `getXY` function of utils
  void predict_current_state(double x, double y, double vx, double vy, double car_yaw, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y);
};

#endif /* simplePredictionVehicle_h */
