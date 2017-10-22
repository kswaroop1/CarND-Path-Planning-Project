#include "Eigen-3.3/Eigen/Dense"
#include "TrajectoryGenerator.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

JMT::JMT(const triple& start, const triple& end, const double T)
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
  vv[0] = end.a0 - (start.a0 + start.a1 * T + start.a2 * T*T / 2.0);
  vv[1] = end.a1 - (start.a1 + start.a2 * T);
  vv[2] = end.a2 - start.a2;

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
  a0 = start.a0; a1 = start.a1; a2 = start.a2 / 2.0;
  a3 = ss[0]; a4 = ss[1]; a5 = ss[2];
}

triple JMT::state_at(const double t) const {
  return { f0(t), f1(t), f2(t) };
}

// from TrajectoryExercise2/helpers.py
double traj_at::logistic(const double x) { return 2.0 / (1.0 + exp(-x)) - 1.0; }
double toEquation(const vector<double>& coeffs, const double x) {
  auto total = 0.0;
  for (auto i = 0; i < coeffs.size(); ++i) total += coeffs[i] * pow(x, i);
  return total;
}
vector<double> differentiate(const vector<double>& coeffs) {
  vector<double> diff;
  for (auto i = 1; i < coeffs.size(); ++i) diff.push_back(i * coeffs[i]);
  return diff;
}


double traj_at::time_diff_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  return tg::ptg_time_diff_cost * logistic(fabs(t - T) / T);
}
double traj_at::s_diff_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  const auto target = tvehAt0.s.after(T) + delta.s;
  const auto actual = s_coeffs.state_at(T);
  auto cost = logistic(fabs(actual.a0 - target.a0) / tg::sigma_s[0]);
  cost += logistic(fabs(actual.a1 - target.a1) / tg::sigma_s[1]);
  cost += logistic(fabs(actual.a2 - target.a2) / tg::sigma_s[2]);
  return tg::ptg_s_diff_cost * cost;
}
double traj_at::d_diff_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  const auto target = tvehAt0.d.after(T) + delta.d;
  const auto actual = d_coeffs.state_at(T);
  auto cost = logistic(fabs(actual.a0 - target.a0) / tg::sigma_d[0]);
  cost += logistic(fabs(actual.a1 - target.a1) / tg::sigma_d[1]);
  cost += logistic(fabs(actual.a2 - target.a2) / tg::sigma_d[2]);
  return tg::ptg_d_diff_cost * cost;
}
double traj_at::collision_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  const auto cost = (tvehAt0.nearest_approach_to_any_vehicle(vehicles) < 2 * tg::vehicle_radius ? 1.0 : 0.0);
  return tg::ptg_collision_cost * cost;
}
double traj_at::buffer_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  return tg::ptg_buffer_cost * logistic(2 * tg::vehicle_radius / tvehAt0.nearest_approach_to_any_vehicle(vehicles));
}
double traj_at::stays_on_road_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  return tg::ptg_stays_on_road_cost * 0;
}
double traj_at::exceeds_speed_limit_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  return tg::ptg_exceeds_speed_cost * 0;
}
double traj_at::efficiency_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  const auto avg_v = s_coeffs.f0(t) / t;
  const auto target = tvehAt0.s.after(t);
  const auto target_v = target.a0 / t;
  return tg::ptg_efficiency_cost * logistic(2 * (target_v - avg_v) / avg_v);
}
double traj_at::max_accel_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  auto max_acc = std::numeric_limits<double>::min();
  for (auto i = 0; i<100; i++) {
    const auto acc = s_coeffs.f2(i*T / 100.0);
    if (fabs(acc) > fabs(max_acc)) max_acc = acc;
  }
  return tg::ptg_max_accel_cost * (fabs(max_acc)> tg::max_accel ? 1 : 0);
}
double traj_at::total_accel_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  auto total_acc = 0.0;
  const auto dt = T / 100;
  for (auto i = 0; i<100; i++) {
    const auto acc = s_coeffs.f2(dt * i);
    total_acc += fabs(acc*dt);
  }
  const auto acc_per_second = total_acc / T;

  return tg::ptg_total_accel_cost * logistic(acc_per_second / tg::expected_acc_in_one_sec);
}
double traj_at::max_jerk_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  auto max_jerk = std::numeric_limits<double>::min();
  for (auto i = 0; i<100; i++) {
    const auto jerk = s_coeffs.f3(i*T / 100.0);
    if (fabs(jerk) > fabs(max_jerk)) max_jerk = jerk;
  }
  return tg::ptg_max_jerk_cost * (fabs(max_jerk)>max_jerk ? 1 : 0);
}
double traj_at::total_jerk_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  auto total_jerk = 0.0;
  const auto dt = T / 100;
  for (auto i = 0; i<100; i++) {
    const auto jerk = s_coeffs.f2(dt * i);
    total_jerk += fabs(jerk*dt);
  }
  const auto jerk_per_second = total_jerk / T;

  return tg::ptg_total_jerk_cost * logistic(jerk_per_second / tg::expected_jerk_in_one_sec);
}
double traj_at::calculate_cost(const state& tvehAt0, const state& delta, const double T, const vector<state>& vehicles) const {
  return time_diff_cost(tvehAt0, delta, T, vehicles) +
    s_diff_cost(tvehAt0, delta, T, vehicles) +
    d_diff_cost(tvehAt0, delta, T, vehicles) +
    collision_cost(tvehAt0, delta, T, vehicles) +
    buffer_cost(tvehAt0, delta, T, vehicles) +
    stays_on_road_cost(tvehAt0, delta, T, vehicles) +
    exceeds_speed_limit_cost(tvehAt0, delta, T, vehicles) +
    efficiency_cost(tvehAt0, delta, T, vehicles) +
    max_accel_cost(tvehAt0, delta, T, vehicles) +
    total_accel_cost(tvehAt0, delta, T, vehicles) +
    max_jerk_cost(tvehAt0, delta, T, vehicles) +
    total_jerk_cost(tvehAt0, delta, T, vehicles);
}

TrajectoryGenerator::TrajectoryGenerator(int lane_width_, int num_lanes_) : lane_width(lane_width_), num_lanes(num_lanes_),
  s_distribs{ nd{ 0.0, tg::sigma_s[0] }, nd{ 0.0, tg::sigma_s[1] }, nd{ 0.0, tg::sigma_s[2] } },
  d_distribs{ nd{ 0.0, tg::sigma_d[0] }, nd{ 0.0, tg::sigma_d[1] }, nd{ 0.0, tg::sigma_d[2] } }
{}

state TrajectoryGenerator::perturb_goal(const state& s) {
  default_random_engine gen;
  state pg{
    { s.s.a0 + s_distribs[0](gen), s.s.a1 + s_distribs[1](gen), s.s.a2 + s_distribs[2](gen) },
    { s.d.a0 + d_distribs[0](gen), s.d.a1 + d_distribs[1](gen), s.d.a2 + d_distribs[2](gen) }
  };
  // basic validations
  pg.s.a0 = max(pg.s.a0, s.s.a0);
  pg.d.a0 = min(max(pg.s.a0, 0.0), 3.0*4.0); // TODO: pass in num_lanes & lane_width
  return pg;
}

traj_at TrajectoryGenerator::PTG(const state& start, const state& targetAt0, const state& delta, const double T, const vector<state>& predictions) {
  //# generate alternative goals
  //  all_goals = []
  vector<tuple<state, double>> all_goals;
  //  timestep = 0.5
  const auto timestep = 0.5;
  //  t = T - 4 * timestep
  auto t = T - 4 * timestep;
  //  while t <= T + 4 * timestep:
  ;
  ;

  while (t <= T + 4 * timestep) {
    //    target_state = np.array(target.state_in(t)) + np.array(delta)
    auto target_state = targetAt0.after(t);
    //    goal_s = target_state[:3]
    //    goal_d = target_state[3:]
    //    goals = [(goal_s, goal_d, t)]
    vector<tuple<state, double>> goals;
    goals.push_back({ target_state, t });
    //    for _ in range(N_SAMPLES) :
    for (auto i = 0; i< tg::n_samples; i++) {
      //      perturbed = perturb_goal(goal_s, goal_d)
      //      goals.append((perturbed[0], perturbed[1], t))
      goals.push_back({ perturb_goal(target_state), t });
    }
    //      all_goals += goals
    all_goals.insert(all_goals.end(), goals.begin(), goals.end());
    //      t += timestep
    t += timestep;
  }

  //  # find best trajectory
  //  trajectories = []
  vector<traj_at> trajectories;
  auto best_cost = numeric_limits<double>::max();
  auto best_idx = -1;
  //  for goal in all_goals :
  for (auto i = 0; i < all_goals.size(); i++) {
    //    s_goal, d_goal, t = goal
    const auto& elt = all_goals[i];
    auto& goal = get<0>(elt);
    auto& t1 = get<1>(elt);
    //    s_coefficients = JMT(start_s, s_goal, t)
    const auto s_coefficients = JMT(start.s, goal.s, t1);
    //    d_coefficients = JMT(start_d, d_goal, t)
    const auto d_coefficients = JMT(start.d, goal.d, t1);
    //    trajectories.append(tuple([s_coefficients, d_coefficients, t]))
    traj_at traj{ s_coefficients, d_coefficients, t1 };
    trajectories.push_back(traj);
    const auto cost = traj.calculate_cost(targetAt0, delta, T, predictions);
    if (cost < best_cost) { best_idx = i; best_cost = cost; }
  }
  //  best = min(trajectories, key = lambda tr : calculate_cost(tr, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS))
  //  calculate_cost(best, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS, verbose = True)
  //  return best
  return trajectories[best_idx];
}
