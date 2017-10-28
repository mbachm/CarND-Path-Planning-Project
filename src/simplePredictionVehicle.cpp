//
//  SimplePredictionVehicle.cpp
//  path_planning
//
//  Created by Bachmann, Michael on 21.10.17.
//

#include "simplePredictionVehicle.h"
#include "utils.h"

SimplePredictionVehicle::SimplePredictionVehicle(int id, double s, double d, double speed)
{
  this->id = id;
  this->s = s;
  this->d = d;
  this->speed = speed;
  
  predict_current_lane();
}

SimplePredictionVehicle::~SimplePredictionVehicle() {}

void SimplePredictionVehicle::predict_current_lane()
{
  if (d<4) {
    this->lane = 0;
  } else if (d < 8) {
    this->lane = 1;
  } else {
    this->lane = 2;
  }
}

double SimplePredictionVehicle::s_position_at(double t)
{
  return this->s + this->speed * t;
}

vector<double> SimplePredictionVehicle::generate_predictions(int horizon)
{
  
  vector<double> predictions;
  
  for(int i = 0; i < horizon; i++)
  {
    predictions.push_back(s_position_at(i * 0.02));
  }
  
  return predictions;
}

void SimplePredictionVehicle::predict_current_state(double x, double y, double vx, double vy, double car_yaw, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y)
{
  vector<double> frenetOld = getFrenet(x, y, car_yaw, map_waypoints_x, map_waypoints_y);
  //Predict 20 timesteps into the future
  double c = 20 * 0.02;
  double futureX = x + c * vx;
  double futureY = y + c * vy;
  vector<double> frenetNew = getFrenet(futureX, futureY, car_yaw, map_waypoints_x, map_waypoints_y);
  double d_change = frenetOld[1] - frenetNew[1];
  if (d_change > 1)
  {
    this->predictedState = LCR;
  } else if (d_change < -1)
  {
    this->predictedState = LCL;
  } else
  {
    this->predictedState = KL;
  }
}
