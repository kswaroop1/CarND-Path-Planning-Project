#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "WayPoint.h"

using namespace std;

class Vehicle {
public:
  struct collider {
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  int L = 1;
  int preferred_buffer = 6; // impacts "keep lane" behavior.
  int lane;
  double s;
  double v;
  double a;

  double x, y, vx, vy, yaw;
  int target_speed;
  int lanes_available;
  int max_acceleration;
  int goal_lane;
  int goal_s;
  string state;

  struct snapshot {
    int lane;
    double s, v, a;
    string state;
    snapshot(int l1, double s1, double v1, double a1) : lane(l1), s(s1), v(v1), a(a1) {}
    snapshot(const Vehicle& h) : lane(h.lane), s(h.s), v(h.v), a(h.a), state(h.state) {}
  };

  Vehicle(int lane, double s, double v, double a);
  virtual ~Vehicle();

  void update_state(map<int, vector <snapshot> > predictions);
  void configure(int target_speed, int lanes_available, int max_acceleration, int goal_lane, int goal_s);
  string display();

  void increment(int dt);
  snapshot state_at(int t);

  bool collides_with(Vehicle other, int at_time);
  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector <snapshot> > predictions);
  void realize_constant_speed();
  int _max_accel_for_lane(map<int, vector<snapshot> > predictions, int lane, int s);
  void realize_keep_lane(map<int, vector<snapshot> > predictions);
  void realize_lane_change(map<int, vector<snapshot> > predictions, string direction);
  void realize_prep_lane_change(map<int, vector<snapshot> > predictions, string direction);

  vector<snapshot> generate_predictions(int horizon);

  vector<snapshot> _trajectory_for_state(const string& state, map<int, vector<snapshot>> predictions, int horizon = 5);
  double _calculate_cost(const vector<snapshot>& traj, const map<int, vector<snapshot>>& predictions);

  Point toVehicleCoordinates(Point p);
  Point toGlobalCoordinates(Point p);
};

#endif