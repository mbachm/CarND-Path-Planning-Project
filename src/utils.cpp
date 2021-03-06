//
//  utils.cpp
//  path_planning
//
//  Created by Bachmann, Michael on 18.10.17.
//

#include "utils.h"

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;
  
  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
    
  }
  
  return closestWaypoint;
  
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
  
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  
  double heading = atan2( (map_y-y),(map_x-x) );
  
  double angle = abs(theta-heading);
  
  if(angle > pi()/4)
  {
    closestWaypoint++;
  }
  
  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
  
  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }
  
  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];
  
  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;
  
  double frenet_d = distance(x_x,x_y,proj_x,proj_y);
  
  //see if d value is positive or negative by comparing it to a center point
  
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);
  
  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }
  
  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }
  
  frenet_s += distance(0,0,proj_x,proj_y);
  
  return {frenet_s,frenet_d};
  
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;
  
  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }
  
  int wp2 = (prev_wp+1)%maps_x.size();
  
  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
  
  double perp_heading = heading-pi()/2;
  
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  
  return {x,y};
}

vector<SimplePredictionVehicle::StateMachineState> predict_successor_states(SimplePredictionVehicle::StateMachineState currentState, const int lane)
{
  vector<SimplePredictionVehicle::StateMachineState> states;
  
  switch (currentState) {
    case SimplePredictionVehicle::KL:
      states.push_back(SimplePredictionVehicle::KL);
      if (lane != 0)
      {
        states.push_back(SimplePredictionVehicle::LCL);
      }
      if (lane != 2)
      {
        states.push_back(SimplePredictionVehicle::LCR);
      }
      break;
    case SimplePredictionVehicle::LCL:
      states.push_back(SimplePredictionVehicle::LCL);
      states.push_back(SimplePredictionVehicle::KL);
      break;
    case SimplePredictionVehicle::LCR:
      states.push_back(SimplePredictionVehicle::LCR);
      states.push_back(SimplePredictionVehicle::KL);
  }
  return states;
}

// Check if it is save to change lane with the other vehicle behind us in other lane or not
static bool vehicle_is_far_enough_behind_and_slower(int current_s, double current_speed, const SimplePredictionVehicle &second_vehicle) {
  return second_vehicle.s + 30.0 < current_s  && second_vehicle.speed < current_speed && 5.0 < current_speed - second_vehicle.speed;
}

bool will_collide(vector<vector<double>> path_for_test_v, const double current_speed, const int current_s, SimplePredictionVehicle second_vehicle, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  vector<double> ptsx = path_for_test_v[0];
  vector<double> ptsy = path_for_test_v[1];
  vector<double> s_values = second_vehicle.generate_predictions(ptsx.size());
  double d = second_vehicle.d;
  
  for(int i = 0; i<ptsx.size(); ++i)
  {
    vector<double> xy = getXY(s_values[i], d, maps_s, maps_x, maps_y);
    if (abs(ptsx[i] - xy[0]) <= 20.0 && abs(ptsy[i] - xy[1]) <= 20.0)
    {
      if(vehicle_is_far_enough_behind_and_slower(current_s, current_speed, second_vehicle)) {
        return false;
      }
      return true;
    }
  }
  return false;
}
