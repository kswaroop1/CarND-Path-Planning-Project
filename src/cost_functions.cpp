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


// TODO - complete this function
vector<double> JMT(vector< double> start, vector <double> end, double T)
{
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
  corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
  length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */
  auto vv = VectorXd(3);
  vv[0] = end[0] - (start[0] + start[1] * T + start[2] * T*T / 2.0);
  vv[1] = end[1] - (start[1] + start[2] * T);
  vv[2] = end[2] - start[2];

  auto tt = MatrixXd(3, 3);
  tt(0, 0) = T*T*T;
  tt(0, 1) = tt(0, 0)*T;
  tt(0, 2) = tt(0, 1)*T;
  tt(1, 0) = 3 * T*T;
  tt(1, 1) = 4 * tt(0, 0);
  tt(1, 2) = 5 * tt(0, 1);
  tt(2, 0) = 6 * T;
  tt(2, 1) = 4 * tt(1, 0);
  tt(2, 2) = 5 * tt(1, 1);

  auto ss = VectorXd{ tt.inverse()*vv };
  return { start[0],start[1],start[2] / 2.0,ss[0],ss[1],ss[2] };
}


// from TrajectoryExercise2/helpers.py
double logistic(double x) { return 2.0 / (1.0 + exp(-x)) - 1.0; }
double toEquation(const vector<double>& coeffs, double x) {
  auto total = 0.0;
  for (auto i = 0; i < coeffs.size(); ++i) total += coeffs[i] * pow(x, i);
  return total;
}
vector<double> differentiate(const vector<double>& coeffs) {
  vector<double> diff;
  for (auto i = 1; i < coeffs.size(); ++i) diff.push_back(i * coeffs[i]);
  return diff;
}