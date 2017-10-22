#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "Vehicle.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//# priority levels for costs
constexpr int COLLISION = (int)10e6;
constexpr int DANGER = (int)10e5;
constexpr int REACH_GOAL = (int)10e5;
constexpr int COMFORT = (int)10e4;
constexpr int EFFICIENCY = (int)10e2;

constexpr double DESIRED_BUFFER = 1.5; //# timesteps
constexpr int PLANNING_HORIZON = 2;

struct trajdata {
  double proposed_lane, avg_speed, max_acceleration, rms_acceleration, closest_approach, end_distance_to_goal, end_lanes_from_goal, collides_at;
  bool collides;
};

double change_lane_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data);
double distance_from_goal_lane(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data);
double inefficiency_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data);
double collision_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data);
double buffer_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<Vehicle::snapshot>>& predictions, const trajdata& data);

#endif