#include "Road.h"
#include "Vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

/**
* Initializes Road
*/
Road::Road(string map_file) {
  this->camera_center = this->update_width / 2;

  // The max s value before wrapping around the track back to 0
  //double max_s = 6945.554;

  ifstream in_map_(map_file.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x, y, s, dx, dy;
    iss >> x >> y >> s >> dx >> dy;
    waypoints.push_back({ x, y, s, dx, dy });
  }
}

Road::~Road() {}

//void Road::updateNewPathState(const jason& prev_path_x_, const jason& prev_path_y_, double end_path_s_, double end_path_d_)
void Road::updateNewPathState(const vector<double>& prev_path_x_, const vector<double>&  prev_path_y_, double end_path_s_, double end_path_d_, double car_x_, double car_y_, double car_s_, double car_d_, double car_yaw_) {
  //prev_path_x.clear(); prev_path_y.clear();
  //auto sz = prev_path_x.size();
  //for (auto i=0; i < sz; i++) {
  //  prev_path_x.push_back(prev_path_x_[i]);
  //  prev_path_y.push_back(prev_path_y_[i]);
  //}
  prev_path_x = prev_path_x_; prev_path_y = prev_path_y_;
  end_path_s = end_path_s_; end_path_d = end_path_d_;
  car_x = car_x_; car_y = car_y_; car_s = car_s_; car_d = car_d_;
  car_yaw = car_yaw_;
}

void Road::updateCarPositions(json sensor_fusion) {
  veh_states.clear();
  vehicles.clear();
  for (const auto& oth : sensor_fusion) { //id,x,y,vx,vy,s,d
    auto oth_id = int(oth[0]);
    double oth_vx = oth[3], oth_vy = oth[4], oth_s = oth[5], oth_d = oth[6];
    auto oth_v = sqrt(oth_vx * oth_vx + oth_vy * oth_vy);
    auto lane = int(ceil(oth_d / lane_width));
    Vehicle veh{ lane, oth_s, oth_v, 0.0, oth_d };
    veh.x = oth[1]; veh.y = oth[2]; veh.vx = oth_vx; veh.vy = oth_vy;
    veh.behavior_state = "CS";
    vehicles.insert(pair<int, Vehicle>(oth_id, veh));
    veh_states.push_back({ {oth_s, oth_v, 0.0},{oth_d, 0.0, 0.0} });
  }
}

Vehicle Road::get_ego() {
  return this->vehicles.find(this->ego_key)->second;
}


tg_state Road::GetBehavarialGoal() {
  map<int, vector<lane_s>> predictions;
  auto it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    auto v_id = it->first;
    predictions[v_id] = it->second.generate_predictions(2 * PLANNING_HORIZON);
    ++it;
  }
  it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    auto v_id = it->first;
    if (v_id == ego_key) {
      it->second.update_state(predictions);
      it->second.realize_state(predictions);
    }
    it->second.increment(1);

    ++it;
  }
  auto ego = get_ego();
  return { {double(ego.goal.s), double(ego.goal.v), ego.goal.a}, {ego.goal.lane*lane_width - 2.0, 0.0, 0.0} };
}

tuple<vector<double>, vector<double>> Road::GetTrajectory(const traj_at& ptg_traj) {
  auto ego = get_ego();
  double ref_x = car_x, ref_y = car_y, ref_yaw = ego.yaw;
  vector<double> x_pts, y_pts;
  tie(x_pts, y_pts) = ptg_traj.trajectory(0.02); // 20 ms per point

  Vehicle ref{ 0, 0, 0, 0, 0 }; ref.x = ref_x; ref.y = ref_y; ref.yaw = ref_yaw;
  const auto prev_sz = prev_path_x.size();
  const auto traj_sz = x_pts.size();
  next_x_vals.clear(); next_y_vals.clear();

  for (auto i = 0; i < prev_sz; i++) {
    next_x_vals.push_back(prev_path_x[i]); next_y_vals.push_back(prev_path_y[i]);
    cout << "PTG p#" << i << ": (" << next_x_vals[i] << "," << next_x_vals[i] << ")" << endl;
  }

  for (auto traj_idx=prev_sz; traj_idx < traj_sz && traj_idx<50; traj_idx++) {
    const auto pt = ref.toGlobalCoordinates({ x_pts[traj_idx], y_pts[traj_idx] });
    next_x_vals.push_back(pt.x); next_y_vals.push_back(pt.y);
    cout << "PTG n#" << traj_idx << ": (" << pt.x << "," << pt.y << ")" << endl;
  }
  return { next_x_vals, next_y_vals };
}

tuple<vector<double>, vector<double>> Road::advance() {
  auto goal = GetBehavarialGoal();
  auto traj = ptg.PTG(ego_state, goal, { {-5,0,0}, {0,0,0} }, 10, veh_states);
  return GetTrajectory(traj);
}
void Road::add_ego(Vehicle ego) { /*, vector<int> config_data) {
  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;
    Vehicle v = it->second;
    if (v.lane == lane_num && v.s == s) {
      this->vehicles.erase(v_id);
    }
    it++;
  }
  Vehicle ego = Vehicle(lane_num, s, v, a);
  ego.configure(config_data);
*/
  ego.behavior_state = "KL";
  this->vehicles.insert(std::pair<int, Vehicle>(ego_key, ego));
  ego_state = { {ego.s, ego.v, ego.a},{ego.d, 0.0, 0.0} };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
Point Road::getXY(double s, double d)
{
  int prev_wp = -1;

  while (s > waypoints[prev_wp + 1].s && (prev_wp < (int)(waypoints.size() - 1)))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % waypoints.size();

  double heading = atan2((waypoints[wp2].y - waypoints[prev_wp].y), (waypoints[wp2].x - waypoints[prev_wp].x));
  // the x,y,s along the segment
  double seg_s = (s - waypoints[prev_wp].s);

  double seg_x = waypoints[prev_wp].x + seg_s*cos(heading);
  double seg_y = waypoints[prev_wp].y + seg_s*sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return { x,y };
}

tk::spline Road::calcSpline(const vector<double>& x_vals, const vector<double>& y_vals)
{
  tk::spline s;
  s.set_points(x_vals, y_vals);
  return s;
}

int Road::ClosestWaypoint(double x, double y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (size_t i = 0; i < waypoints.size(); i++)
  {
    double map_x = waypoints[i].x;
    double map_y = waypoints[i].y;
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int Road::NextWaypoint(double x, double y, double theta)
{
  int closestWaypoint = ClosestWaypoint(x, y);

  double map_x = waypoints[closestWaypoint].x;
  double map_y = waypoints[closestWaypoint].y;

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > pi() / 4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Point Road::getFrenet(double x, double y, double theta)
{
  int next_wp = NextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0)
  {
    prev_wp = waypoints.size() - 1;
  }

  double n_x = waypoints[next_wp].x - waypoints[prev_wp].x;
  double n_y = waypoints[next_wp].y - waypoints[prev_wp].y;
  double x_x = x - waypoints[prev_wp].x;
  double x_y = y - waypoints[prev_wp].y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - waypoints[prev_wp].x;
  double center_y = 2000 - waypoints[prev_wp].y;
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(waypoints[i].x, waypoints[i].y, waypoints[i + 1].x, waypoints[i + 1].y);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return { frenet_s,frenet_d };
}
