#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include <map>
#include <string>
#include "WayPoint.h"

using namespace std;

constexpr double DESIRED_BUFFER = 1.5; //# timesteps
constexpr int PLANNING_HORIZON = 5; //2

struct lane_s { int lane; double s; };

class Vehicle {
public:
  struct collider {
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  static constexpr int L = 1; // vehicle length, for simple collission detection
  static constexpr int preferred_buffer = 3; // impacts "keep lane" behavior.

  int lane;
  double s, v, a, d; // s is freenet distance ahead, not speed; v is velocity; a is accelaration; d is freenet displacement

  double x, y, vx, vy, yaw;
  int target_speed;
  int max_acceleration;
  int goal_lane = 2;  // middle lane preferred
  int goal_s;
  string behavior_state;

  struct snapshot {
    int lane;
    double s, v, a;
    string behavior_state;
    snapshot(const int l1, const double s1, const double v1, const double a1) : lane(l1), s(s1), v(v1), a(a1) {}
    explicit snapshot(const Vehicle& h) : lane(h.lane), s(h.s), v(h.v), a(h.a), behavior_state(h.behavior_state) {}
    //snapshot(const snapshot& s) : lane(s.lane), s(s.s), v(s.v), a(s.a), behavior_state(s.behavior_state) {}
    snapshot() : lane(0), s(0), v(0), a(0), behavior_state("CS") {}
  };
  snapshot goal;

  Vehicle(int lane, double s, double v, double a, double d);
  virtual ~Vehicle();

  void update_state(map<int, vector<lane_s>> predictions);
  string display();

  void increment(const int dt = 1);
  snapshot snapshot_at(const int t) const;
  double s_at(const int dt) const                                   { return s + v*dt + a*dt*dt/2; }

  bool collides_with(const Vehicle& other, const int at_time) const;
  collider will_collide_with(const Vehicle& other, const int timesteps) const;

  void realize_state(const map<int, vector<lane_s>>& predictions);
  void realize_constant_speed();
  int _max_accel_for_lane(const map<int, vector<lane_s>>& predictions, const int lane, const int s) const;
  void realize_keep_lane(const map<int, vector<lane_s>>& predictions);
  void realize_lane_change(const map<int, vector<lane_s>>& predictions, string direction);
  void realize_prep_lane_change(const map<int, vector<lane_s>>& predictions, string direction);

  vector<lane_s> generate_predictions(const int horizon = PLANNING_HORIZON) const;

  vector<snapshot> _trajectory_for_state(const string& state, map<int, vector<lane_s>> predictions, int horizon = PLANNING_HORIZON);
  double _calculate_cost(const vector<snapshot>& traj, const map<int, vector<lane_s>>& predictions);

  Point toVehicleCoordinates(Point p);
  Point toGlobalCoordinates(Point p);
};

#endif