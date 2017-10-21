//
//  SimplePredictionVehicle.cpp
//  path_planning
//
//  Created by Bachmann, Michael (415) on 21.10.17.
//

#include "simplePredictionVehicle.h"

SimplePredictionVehicle::SimplePredictionVehicle(int id, double s, double d, double speed)
{
  this->id = id;
  this->s = s;
  this->d = d;
  this->speed = speed;
  //Other car's in simulator seem to not change their lane
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

vector<double> SimplePredictionVehicle::generate_predictions(int horizon = 50) {
  
  vector<double> predictions;
  
  for( int i = 0; i < horizon; i++)
  {
    predictions.push_back(s_position_at(i * 0.02));
  }
  
  return predictions;
}
