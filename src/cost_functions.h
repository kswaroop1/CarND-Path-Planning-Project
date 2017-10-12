#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "vehicle.h"

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


vector<double> JMT(vector< double> start, vector <double> end, double T);

// from TrajectoryExercise2/constants.py
constexpr int n_samples = 10;
constexpr double sigma_s[] = { 10.0, 4.0, 2.0 };
constexpr double sigma_d[] = { 1.0, 1.0, 1.0 };
constexpr double sigma_t = 2.0;

constexpr double max_jerk = 10.0; // m/s/s/s
constexpr double max_accel = 10.0; // m/s/s
constexpr double max_speed = 49.5/2.24; // m/s == 49.5 MPH

constexpr double expected_jerk_in_one_sec = 2.0; // m/s/s
constexpr double expected_acc_in_one_sec = 1.0; // m/s
constexpr double vehicle_radius = 1.5; // model vehicle as circle to simplify collision detection

// from TrajectoryExercise2/helpers.py
double logistic(double x); // { return 2.0 / (1.0 + exp(-x)) - 1.0; }
double toEquation(const vector<double>& coeffs, double x);
vector<double> differentiate(const vector<double>& coeffs);

#endif