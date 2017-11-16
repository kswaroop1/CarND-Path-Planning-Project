#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H
#include <vector>
#include "WayPoint.h"
#include <random>

using namespace std;

struct JMT {
  double a0, a1, a2, a3, a4, a5;

  JMT(const triple& start, const triple& end, const double T);
  double f0(const double t) const { return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t; }
  double f1(const double t) const { return a1 + 2 * a2*t + 3 * a3*t*t + 4 * a4*t*t*t + 5 * a5*t*t*t*t; }
  double f2(const double t) const { return 2 * a2 + 3 * 2 * a3*t + 4 * 3 * a4*t*t + 5 * 4 * a5*t*t*t; }
  double f3(const double t) const { return 3 * 2 * a3 + 4 * 3 * 2 * a4*t + 5 * 4 * 3 * a5*t*t; }
  triple state_at(const double t) const;
  vector<double> f0_vals(double uptoT, double interval) const;
};

struct traj_at {
  JMT s_coeffs, d_coeffs;
  double t;

  double time_diff_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double s_diff_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double d_diff_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double collision_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double buffer_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double stays_on_road_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double exceeds_speed_limit_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double efficiency_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double max_accel_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double total_accel_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double max_jerk_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double total_jerk_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;
  double calculate_cost(const tg_state& tvehAt0, const tg_state& delta, const double T, const vector<tg_state>& vehicles) const;

  // from TrajectoryExercise2/helpers.py
  static double logistic(double x);                        // { return 2.0 / (1.0 + exp(-x)) - 1.0; }

  tuple<vector<double>, vector<double>> trajectory(double interval) const { return { s_coeffs.f0_vals(t, interval), d_coeffs.f0_vals(t, interval) }; }
};

// from TrajectoryExercise2/constants.py
namespace tg {
  constexpr int n_samples = 10;
  constexpr double sigma_s[] = { 10.0, 4.0, 2.0 };
  constexpr double sigma_d[] = { 1.0, 1.0, 1.0 };
  constexpr double sigma_t = 2.0;

  constexpr double max_jerk = 10.0;                 // m/s/s/s
  constexpr double max_accel = 10.0;                // m/s/s
  constexpr double max_speed = 49.5 / 2.24;         // m/s == 49.5 MPH

  constexpr double expected_jerk_in_one_sec = 2.0;  // m/s/s
  constexpr double expected_acc_in_one_sec = 1.0;   // m/s
  constexpr double vehicle_radius = 1.5;            // model vehicle as circle to simplify collision detection

  constexpr double ptg_time_diff_cost = 10e4;
  constexpr double ptg_s_diff_cost = 10e4;
  constexpr double ptg_d_diff_cost = 10e5;
  constexpr double ptg_efficiency_cost = 10e3;
  constexpr double ptg_max_jerk_cost = 10e2;
  constexpr double ptg_total_jerk_cost = 10e4;
  constexpr double ptg_collision_cost = 10e6;
  constexpr double ptg_buffer_cost = 10e3;
  constexpr double ptg_total_accel_cost = 10e2;
  constexpr double ptg_max_accel_cost = 10e2;
  constexpr double ptg_exceeds_speed_cost = 10e5;
  constexpr double ptg_stays_on_road_cost = 10e7;

//  static double toEquation(const vector<double>& coeffs, double x);
//  static vector<double> differentiate(const vector<double>& coeffs);
};

struct TrajectoryGenerator {
  typedef normal_distribution<double> nd;

  TrajectoryGenerator();
  traj_at PTG(const tg_state& start, const tg_state& targetAt0, const tg_state& delta, const double T, const vector<tg_state>& predictions);

  tg_state perturb_goal(const tg_state& s);
  nd s_distribs[3], d_distribs[3];
};

#endif
