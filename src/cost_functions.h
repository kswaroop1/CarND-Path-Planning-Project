#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H
#include "vehicle.h"

//# priority levels for costs
const int COLLISION = (int)10e6;
const int DANGER = (int)10e5;
const int REACH_GOAL = (int)10e5;
const int COMFORT = (int)10e4;
const int EFFICIENCY = (int)10e2;

const double DESIRED_BUFFER = 1.5; //# timesteps
const int PLANNING_HORIZON = 2;

struct trajdata {
  double proposed_lane, avg_speed, max_acceleration, rms_acceleration, closest_approach, end_distance_to_goal, end_lanes_from_goal, collides_at;
  bool collides;
};

double change_lane_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<vector<int>>>& predictions, const trajdata& data);
double distance_from_goal_lane(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<vector<int>>>& predictions, const trajdata& data);
double inefficiency_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<vector<int>>>& predictions, const trajdata& data);
double collision_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<vector<int>>>& predictions, const trajdata& data);
double buffer_cost(const Vehicle& vehicle, const vector<Vehicle::snapshot>& trajectory, const map<int, vector<vector<int>>>& predictions, const trajdata& data);

#endif