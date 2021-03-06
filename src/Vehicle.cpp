#include "Vehicle.h"
#include "cost_functions.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <limits>
#include <algorithm>

/**
* Initializes Vehicle
*/
Vehicle::Vehicle(int lane, double s, double v, double a, double d): target_speed(0.0), goal_s(0.0) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->d = d;
  behavior_state = "CS";
  max_acceleration = -1;

  x = y = vx = vy = yaw = 0.0;
}

Vehicle::~Vehicle() {}

void Vehicle::update_state(map<int, vector<lane_s>> predictions) {
  /*
  Updates the "behavior_state" of the vehicle by assigning one of the
  following values to 'self.behavior_state':

  "KL" - Keep Lane
  - The vehicle will attempt to drive its target speed, unless there is
  traffic in front of it, in which case it will slow down.

  "LCL" or "LCR" - Lane Change Left / Right
  - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
  behavior for the "KL" behavior_state in the new lane.

  "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
  - The vehicle will find the nearest vehicle in the adjacent lane which is
  BEHIND itself and will adjust speed to try to get behind that vehicle.

  INPUTS
  - predictions
  A dictionary. The keys are ids of other vehicles and the values are arrays
  where each entry corresponds to the vehicle's predicted location at the
  corresponding timestep. The FIRST element in the array gives the vehicle's
  current position. Example (showing a car with id 3 moving at 2 m/s):

  {
  3 : [
  {"s" : 4, "lane": 0},
  {"s" : 6, "lane": 0},
  {"s" : 8, "lane": 0},
  {"s" : 10, "lane": 0},
  ]
  }

  */

  vector<string> states{ "KL" };
  if (lane > 1) { states.push_back("LCL"); states.push_back("PLCL"); }
  if (lane < (lanes_available - 1)) { states.push_back("LCR"); states.push_back("PLCR"); }

  string min_cost_state;
  auto min_cost = std::numeric_limits<double>::max();
  for (auto& st : states) {
    auto traj = _trajectory_for_state(st, predictions);
    const auto cost = _calculate_cost(traj, predictions);
    if (cost < min_cost) {
      min_cost_state = st;
      min_cost = cost;
      goal = traj[traj.size() - 1];
    }
  }

  behavior_state = min_cost_state;
}

string Vehicle::display() {
  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(const int dt) {
  s += v * dt;
  v += a * dt;
  v = max(0.0, v);
}

Vehicle::snapshot Vehicle::snapshot_at(const int t) const {
  /*
  Predicts behavior_state of vehicle in t seconds (assuming constant acceleration)
  */
  auto s = this->s + this->v * t + this->a * t * t / 2;
  auto v = this->v + this->a * t; v = max(0.0, v);
  return { this->lane, s, v, this->a };
}

bool Vehicle::collides_with(const Vehicle& other, const int at_time) const {
  /*
  Simple collision detection.
  */
  auto check1 = snapshot_at(at_time);
  auto check2 = other.snapshot_at(at_time);
  return (check1.lane == check2.lane) && (abs(check1.s - check2.s) <= L);
}

Vehicle::collider Vehicle::will_collide_with(const Vehicle& other, const int timesteps) const {
  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps + 1; t++) {
    if (collides_with(other, t)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(const map<int, vector<lane_s>>& predictions) {
  /*
  Given a behavior_state, realize it by adjusting acceleration and lane.
  Note - lane changes happen instantaneously.
  */
  string behavior_state = this->behavior_state;
  if (behavior_state.compare("CS") == 0) {
    realize_constant_speed();
  } else if (behavior_state.compare("KL") == 0) {
    realize_keep_lane(predictions);
  } else if (behavior_state.compare("LCL") == 0) {
    realize_lane_change(predictions, "L");
  } else if (behavior_state.compare("LCR") == 0) {
    realize_lane_change(predictions, "R");
  } else if (behavior_state.compare("PLCL") == 0) {
    realize_prep_lane_change(predictions, "L");
  } else if (behavior_state.compare("PLCR") == 0) {
    realize_prep_lane_change(predictions, "R");
  }
}

void Vehicle::realize_constant_speed() {
  a = 0;
}

int Vehicle::_max_accel_for_lane(const map<int, vector<lane_s>>& predictions, const int lane, const int s) const
{
  const int delta_v_til_target = target_speed - v;
  auto max_acc = min(max_acceleration, delta_v_til_target);

  auto it = predictions.begin();
  vector<vector<lane_s>> in_front;
  while (it != predictions.end()) {
    auto v_id = it->first;
    auto& v = it->second;

    if ((v[0].lane == lane) && (v[0].s > s)) {
      in_front.push_back(v);
    }
    ++it;
  }

  if (in_front.size() > 0) {
    auto min_s = 1000;
    vector<lane_s> leading = {};
    for (size_t i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0].s - s) < min_s) {
        min_s = (in_front[i][0].s - s);
        leading = in_front[i];
      }
    }
    
    if (leading.size() < 2) return max_acc; // some thing bad has already happened at this stage, eg car out of lane

    const auto next_pos = leading[1].s;
    const int my_next = s + this->v;
    const auto separation_next = next_pos - my_next;
    const int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }
  return max_acc;
}

void Vehicle::realize_keep_lane(const map<int, vector<lane_s>>& predictions) {
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(const map<int, vector<lane_s>>& predictions, string direction) {
  int delta = -1;
  if (direction.compare("R") == 0) {
    delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(const map<int, vector<lane_s>>& predictions, string direction) {
  auto delta = -1;
  if (direction.compare("R") == 0) delta = 1;
  const auto lane = this->lane + delta;

  auto it = predictions.begin();
  vector<vector<lane_s> > at_behind;
  while (it != predictions.end()) {
    auto v_id = it->first;
    auto v = it->second;

    if ((v[0].lane == lane) && (v[0].s <= this->s)) {
      at_behind.push_back(v);
    }
    it++;
  }
  if (at_behind.size() > 0) {
    auto max_s = -1000;
    vector<lane_s> nearest_behind = {};
    for (size_t i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0].s) > max_s) {
        max_s = at_behind[i][0].s;
        nearest_behind = at_behind[i];
      }
    }
    const auto target_vel = nearest_behind[1].s - nearest_behind[0].s;
    const int delta_v = this->v - target_vel;
    const int delta_s = this->s - nearest_behind[0].s;
    if (delta_v != 0) {
      const auto time = -2 * delta_s / delta_v;
      int a = (time == 0) ? this->a : delta_v / time;
      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }
      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }
      this->a = a;
    } else {
      const auto my_min_acc = max(-this->max_acceleration, -delta_s);
      this->a = my_min_acc;
    }
  }
}

vector<lane_s> Vehicle::generate_predictions(const int horizon) const {
  vector<lane_s> predictions;
  for (auto i = 0; i < horizon; i++) predictions.push_back({lane, s_at(i)});
  return predictions;
}

vector<Vehicle::snapshot> Vehicle::_trajectory_for_state(const string& behavior_state, map<int, vector<lane_s>> predictions, int horizon) {
  vector<snapshot> trajectory;
  snapshot snap{ *this };
  trajectory.push_back(snap);

  for (auto i = 0; i < horizon; i++) {
    Vehicle vv = *this; // { lane, s, v, a };
    vv.behavior_state = behavior_state;
    vv.realize_state(predictions);
    vv.increment();
    trajectory.push_back(snapshot(vv));
    for (auto& pp : predictions) {
      pp.second.erase(pp.second.begin());
    }
  }
  return trajectory;
}

bool check_collision(const Vehicle::snapshot& snapshot, double s_previous, double s_now) {
  auto s = snapshot.s;
  auto v = snapshot.v;
  auto v_target = s_now - s_previous;
  if (s_previous < s) {
    if (s_now >= s)
      return true;
    else
      return false;
  }
  if (s_previous > s) {
    if (s_now <= s)
      return true;
    else
      return false;
  }
  if (s_previous == s) {
    if (v_target > v)
      return false;
    else
      return true;
  }
  throw "Value_Error";
}
map<int, vector<lane_s>> filter_predictions_by_lane(const map<int, vector<lane_s>>& predictions, int lane) {
  map<int, vector<lane_s>> filtered;
  for (const auto& ii : predictions) {
    const auto v_id = ii.first;
    const auto& predicted_traj = ii.second;
    if (predicted_traj[0].lane == lane && v_id != -1)
      filtered[v_id] = predicted_traj;
  }
  return filtered;
}
trajdata get_helper_data(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<lane_s>>& predictions) {
  auto& t = trajectory;
  auto& current_snapshot = t[0];
  auto& first = t[1];
  auto& last = t[t.size() - 1];
  auto end_distance_to_goal = vehicle.goal_s - last.s;
  auto end_lanes_from_goal = abs(vehicle.goal_lane - last.lane);
  auto dt = double(trajectory.size()); // 1s increaments
  auto proposed_lane = first.lane;
  auto avg_speed = (last.s - current_snapshot.s) / dt;

  vector<double> accels;
  auto closest_approach = 999999;
  auto collides = false; auto collides_at = 999999;
  //auto& last_snap = trajectory[0];
  auto filtered = filter_predictions_by_lane(predictions, proposed_lane);

  for (auto i = 1; i < PLANNING_HORIZON; i++) {
    auto lane = t[i].lane; auto s = t[i].s; auto v = t[i].v; auto a = t[i].a;
    accels.push_back(a);
    for (auto& vv : filtered) {
      auto v_id = vv.first; auto& v = vv.second;
      auto behavior_state = v[i];
      auto last_state = v[i - 1];
      if (check_collision(t[i], last_state.s, behavior_state.s)) {
        collides = true;
        collides_at = min(i, collides_at);
      }
      auto dist = abs(behavior_state.s - s);
      if (dist < closest_approach) closest_approach = dist;
    }
    //last_snap = t[i];
  }
  auto max_accel = *std::max_element(accels.begin(), accels.end());
  auto sum_sq_accels = 0.0; for (auto a : accels) sum_sq_accels += a*a;
  auto rms_accel = sum_sq_accels / accels.size();

  trajdata data;
  data.proposed_lane = proposed_lane;
  data.avg_speed = avg_speed;
  data.max_acceleration = max_accel;
  data.rms_acceleration = rms_accel;
  data.closest_approach = closest_approach;
  data.end_distance_to_goal = end_distance_to_goal;
  data.end_lanes_from_goal = end_lanes_from_goal;
  data.collides = collides;
  data.collides_at = collides_at;
  return data;
}
double Vehicle::_calculate_cost(const vector<snapshot>& traj, const map<int, vector<lane_s>>& predictions) {
  trajdata data{ get_helper_data(*this, traj, predictions) };
  auto cost = 0.0;
  for (auto& cf : { distance_from_goal_lane, inefficiency_cost, collision_cost, buffer_cost, change_lane_cost }) {
    cost += cf(*this, traj, predictions, data);
  }

  return cost;
}

Point Vehicle::toVehicleCoordinates(Point p) {
  double shift_x = p.x - x, shift_y = p.y - y;
  return { shift_x*cos(0 - yaw) - shift_y*sin(0 - yaw),
           shift_x*sin(0 - yaw) + shift_y*cos(0 - yaw) };
}

Point Vehicle::toGlobalCoordinates(Point p) {
  auto x_ref = p.x, y_ref = p.y;
  return { x_ref*cos(yaw) - y_ref*sin(yaw) + x,
           x_ref*sin(yaw) + y_ref*cos(yaw) + y };
}
