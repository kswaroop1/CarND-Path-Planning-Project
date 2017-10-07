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
  int s;
  int v;
  int a;
  int target_speed;
  int lanes_available;
  int max_acceleration;
  int goal_lane;
  int goal_s;
  string state;

  struct snapshot {
    int lane, s, v, a;
    string state;
    snapshot(const Vehicle& h) : lane(h.lane), s(h.s), v(h.v), a(h.a), state(h.state) {}
  };

  Vehicle(int lane, int s, int v, int a);
  virtual ~Vehicle();

  void update_state(map<int, vector <vector<int> > > predictions);
  void configure(vector<int> road_data);
  string display();

  void increment(int dt);
  vector<int> state_at(int t);

  bool collides_with(Vehicle other, int at_time);
  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector < vector<int> > > predictions);
  void realize_constant_speed();
  int _max_accel_for_lane(map<int, vector<vector<int> > > predictions, int lane, int s);
  void realize_keep_lane(map<int, vector< vector<int> > > predictions);
  void realize_lane_change(map<int, vector< vector<int> > > predictions, string direction);
  void realize_prep_lane_change(map<int, vector< vector<int> > > predictions, string direction);

  vector<vector<int> > generate_predictions(int horizon);

  vector<snapshot> _trajectory_for_state(const string& state, map<int, vector<vector<int>>> predictions, int horizon = 5);
  double _calculate_cost(const vector<snapshot>& traj, const map<int, vector<vector<int>>>& predictions);
};

#endif