#ifndef ROAD_H
#define ROAD_H
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include "WayPoint.h"
#include "Vehicle.h"
#include "json.hpp"
#include "TrajectoryGenerator.h"

using namespace std;

// for convenience
using json = nlohmann::json;

using namespace std;

class Road {
public:
  vector<WayPoint> waypoints;
  int update_width = 70;
  int ego_key = -1;

  int lane_width;
  int num_lanes;
  int speed_limit;

  int camera_center;

  TrajectoryGenerator ptg;

  map<int, Vehicle> vehicles;
  int vehicles_added = 0;

  Road(int num_lanes, int lane_width, int speed_limit, string map_file);
  virtual ~Road();

  void updateCarPositions(json sensor_fusion);
  Vehicle get_ego();
  void advance();
  void add_ego(Vehicle v);

  constexpr double pi() { return M_PI; }
  double deg2rad(double x) { return x * pi() / 180; }
  double rad2deg(double x) { return x * 180 / pi(); }

  double distance(double x1, double y1, double x2, double y2)  { return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));  }
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  Point getFrenet(double x, double y, double theta);
  Point Road::getXY(double s, double d);
};
#endif