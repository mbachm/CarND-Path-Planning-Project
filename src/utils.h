//
//  helper.h
//  path_planning
//
//  Created by Bachmann, Michael (415) on 18.10.17.
//

#ifndef utils_h
#define utils_h

#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* utils_h */
