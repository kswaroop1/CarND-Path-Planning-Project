#ifndef ROAD_H
#define ROAD_H
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include "WayPoint.h"
#include "Vehicle.h"
#include "TrajectoryGenerator.h"
#include "spline.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

class Road {
public:
  vector<WayPoint> waypoints;
  int update_width = 70;
  static constexpr int ego_key = -1;

  int camera_center;

  vector<double> prev_path_x, prev_path_y, next_x_vals, next_y_vals;
  double end_path_s, end_path_d;
  double car_x, car_y, car_s, car_d, car_yaw;

  TrajectoryGenerator ptg;

  vector<tg_state> veh_states;
  tg_state ego_state;
  map<int, Vehicle> vehicles;
  int vehicles_added = 0;

  Road(string map_file);
  virtual ~Road();

  void updateNewPathState(const vector<double>& prev_path_x_, const vector<double>& prev_path_y_, double end_path_s_, double end_path_d_, double car_x_, double car_y_, double car_s_, double car_d_, double car_yaw_);
  void updateCarPositions(nlohmann::json sensor_fusion);
  Vehicle get_ego();
  tg_state GetBehavarialGoal();
  tuple<vector<double>, vector<double>> GetTrajectory(const traj_at& ptg_traj);
  tuple<vector<double>, vector<double>> advance();
  void add_ego(Vehicle v);

  constexpr double pi() { return M_PI; }
  double deg2rad(double x) { return x * pi() / 180; }
  double rad2deg(double x) { return x * 180 / pi(); }

  double distance(double x1, double y1, double x2, double y2)  { return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));  }
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  Point getFrenet(double x, double y, double theta);
  Point Road::getXY(double s, double d);
  tk::spline calcSpline(const vector<double>& x_vals, const vector<double>& y_vals);
};


#endif