#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "json.hpp"
#include "utils.h"
#include "spline.h"
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

static void getSpeedOfNearestCarInFront(vector<SimplePredictionVehicle> &detected_vehicles, const Vehicle &my_car, double &targed_speed) {
  for (SimplePredictionVehicle vehicle : detected_vehicles) {
    if (vehicle.s > my_car.s)
    {
      targed_speed = vehicle.speed;
      break;
    }
  }
}

static void calculateTargetSpeed(int lane, vector<vector<SimplePredictionVehicle>> &cars_in_lanes, double max_speed, double &targed_speed, bool too_close, Vehicle my_car) {
  if (lane == 0 && cars_in_lanes[0].size() > 0 && too_close)
  {
    //            cout << "reducing speed due to lane 0" << endl << endl;
    sort(cars_in_lanes[0].begin(), cars_in_lanes[0].end(), SimplePredictionVehicle::sort_by_s_distance);
    getSpeedOfNearestCarInFront(cars_in_lanes[0], my_car, targed_speed);
  }
  else if (lane == 1 && cars_in_lanes[1].size() > 0 && too_close)
  {
    //            cout << "reducing speed due to lane 1" << endl << endl;
    sort(cars_in_lanes[1].begin(), cars_in_lanes[1].end(), SimplePredictionVehicle::sort_by_s_distance);
    getSpeedOfNearestCarInFront(cars_in_lanes[1], my_car, targed_speed);
  }
  else if (cars_in_lanes[2].size() > 0 && too_close)
  {
    //            cout << "reducing speed due to lane 2" << endl << endl;
    sort(cars_in_lanes[2].begin(), cars_in_lanes[2].end(), SimplePredictionVehicle::sort_by_s_distance);
    getSpeedOfNearestCarInFront(cars_in_lanes[2], my_car, targed_speed);
  } else
  {
    //            cout << "set target speed to max speed" << endl << endl;
    targed_speed = max_speed;
  }
}

static void adjustSpeedWithoutJerk(double max_speed, double &ref_vel, double targed_speed) {
  if (ref_vel > targed_speed)
  {
    //            cout << "reducing speed" << endl << endl;
    ref_vel -= 0.112;
    if ((ref_vel - targed_speed) < 0.112) {
      ref_vel = targed_speed;
    }
  }
  else if(ref_vel != targed_speed && ref_vel < max_speed)
  {
    //            cout << "add speed" << endl << endl;
    ref_vel += 0.224;
  }
}

int main() {
  uWS::Hub h;
  
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  
  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  //start lane
  int lane = 1;
  
  // reference velocity speed
  static double ref_vel = 0.0; //mph
  static double max_speed = 49.5; //mph
  static double targed_speed = max_speed; // mph
  //own vehicle
  static Vehicle my_car = Vehicle(-1, lane);
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      
      auto s = hasData(data);
      
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_yaw = j[1]["yaw"];
          
          my_car.s = j[1]["s"];;
          my_car.d = j[1]["d"];
          my_car.speed = j[1]["speed"];
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          json msgJson;
          
          int prev_size = previous_path_x.size();
          if(prev_size > 0)
          {
            my_car.s = end_path_s;
          }
          
          vector<SimplePredictionVehicle> lane_0_detected_vehicles;
          vector<SimplePredictionVehicle> lane_1_detected_vehicles;
          vector<SimplePredictionVehicle> lane_2_detected_vehicles;
          bool too_close = false;
          //find ref_v to use
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            if ((d < 0.0) && !(-20.0 < s - my_car.s < 100.0))
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
            new_vehicle.predict_current_state(x, y, vx, vy, car_yaw, map_waypoints_x, map_waypoints_y);
            vector<double> predictions = new_vehicle.generate_predictions(50);
            
            //debug
//            for (auto i = predictions.begin(); i != predictions.end(); ++i)
//              cout << *i << ' ';
//            cout << endl << endl;
            
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
            
            if(lane == new_vehicle.lane)
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              
              s+=((double)prev_size*0.02*check_speed); //if using previous points can project s value outwars in time
              
              //check s values greater than mine and smaller gap than 30 m
              if (s > my_car.s && s-my_car.s < 30)
              {
                too_close = true;
              }
            }
          }
          
          vector<vector<SimplePredictionVehicle>> vehicles = {lane_0_detected_vehicles, lane_1_detected_vehicles, lane_2_detected_vehicles};
          calculateTargetSpeed(lane, vehicles, max_speed, targed_speed, too_close, my_car);
          adjustSpeedWithoutJerk(max_speed, ref_vel, targed_speed);
          
          // list of spaced (x,y) ponts, evenly spaced 30 m
          // will interpolate waypoints with spline
          vector<double> ptsx;
          vector<double> ptsy;
          
          // reference x, y , yaw
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if path is empty, use currenct location as starting reference
          if(prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
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
          
          // Calculate the x,y waypoints for left, middel and right lane
          vector<double> next_wp0 = getXY(my_car.s+30, (2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(my_car.s+60, (2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(my_car.s+90, (2+8*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
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
          tk::spline s;
          
          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);
          
          //define the actual (x,y) points we use for planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for(int i = 0; i<prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //Calculate how to break up spline points so thate we travel at our desired reference velocity // N * 0.02 * velocity = d, N = number of points
          double target_x = 30.0; //horizon point
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          
          double x_add_on = 0;
          
          //fill up the rest of our path planner
          for (int i = 1; i <= 50-prev_size; i++)
          {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
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
          
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          
          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
  
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
  
  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
