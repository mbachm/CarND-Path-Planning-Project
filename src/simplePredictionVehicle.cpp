//
//  SimplePredictionVehicle.cpp
//  path_planning
//
//  Created by Bachmann, Michael (415) on 21.10.17.
//

#include "simplePredictionVehicle.h"

SimplePredictionVehicle::SimplePredictionVehicle(int s, int d, int speed, int acceleration)
{
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->acceleration = acceleration;
}

SimplePredictionVehicle::~SimplePredictionVehicle() {}

vector<int> SimplePredictionVehicle::state_at(int t)
{
  /*
   Predicts state of vehicle in t seconds (assuming constant acceleration)
   */
  int s = this->s + this->speed * t + this->acceleration * t * t / 2;
  int v = this->speed + this->acceleration * t;
  //TODO: check again
  return {this->lane, s, v, this->acceleration};
}

vector<vector<int> > SimplePredictionVehicle::generate_predictions(int horizon = 50) {
  
  vector<vector<int> > predictions;
  
  for( int i = 0; i < horizon; i++)
  {
    vector<int> check1 = state_at(i*0.02);
    vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;
  
}
