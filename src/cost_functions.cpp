#include "cost_functions.h"

double change_lane_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data) {
  //Penalizes lane changes AWAY from the goal lane and rewards lane changes TOWARDS the goal lane.
  auto proposed_lanes = data.end_lanes_from_goal;
  auto cur_lanes = trajectory[0].lane;
  auto cost = 0.0;
  if (proposed_lanes > cur_lanes) cost = COMFORT;
  if (proposed_lanes < cur_lanes) cost = -COMFORT;
  //if (cost != 0) cout << "!! \n \ncost for lane change is " << cost << endl;
  return cost;
}
double distance_from_goal_lane(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data) {
  auto distance = abs(data.end_distance_to_goal);
  distance = max(distance, 1.0);
  auto time_to_goal = float(distance) / data.avg_speed;
  auto lanes = data.end_lanes_from_goal;
  auto multiplier = float(5 * lanes / time_to_goal);
  auto cost = multiplier * REACH_GOAL;
  return cost;
}
double inefficiency_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data) {
  auto speed = data.avg_speed;
  auto target_speed = vehicle.target_speed;
  auto diff = target_speed - speed;
  auto pct = float(diff) / target_speed;
  auto multiplier = pct * pct;
  return multiplier * EFFICIENCY;
}
double collision_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data) {
  if (data.collides) {
    auto time_til_collision = data.collides_at;
    auto exponent = time_til_collision * time_til_collision;
    auto mult = exp(-exponent);
    return mult * COLLISION;
  }
  return 0;
}
double buffer_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data) {
  auto closest = data.closest_approach;
  if (closest == 0) return 10 * DANGER;

  auto timesteps_away = closest / data.avg_speed;
  if (timesteps_away > DESIRED_BUFFER) return 0.0;

  auto multiplier = (timesteps_away / DESIRED_BUFFER);
  multiplier *= multiplier;
  multiplier = 1.0 - multiplier;
  return multiplier * DANGER;
}
