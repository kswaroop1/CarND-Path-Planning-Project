#define _USE_MATH_DEFINES
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "WayPoint.h"
#include "Road.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

tk::spline calcSpline(const vector<Point>& pts)
{
  vector<double> x, y;
  for (auto pt : pts) { x.push_back(pt.x); y.push_back(pt.y); }
  tk::spline s;
  s.set_points(x, y);
  return s;
}

vector<WayPoint> ReadWaypoints()
{
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  vector<WayPoint> ret;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x, y, s, dx, dy;
    iss >> x >> y >> s >> dx >> dy;
    ret.push_back({ x, y, s, dx, dy });
  }
  return ret;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Road road{"../data/highway_map.csv"};
  //auto waypoints = ReadWaypoints();
  cout << "#way points in map=" << road.waypoints.size() << endl;
  auto lane = 1;  auto ref_vel = 0.0; static const auto target_vel = 49.6;

  // h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
  h.onMessage([&](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        cout << "j=" << j << endl;
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"]; double car_s0 = car_s;
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto jprev_path_x = j[1]["previous_path_x"]; int prev_sz = jprev_path_x.size();
          auto jprev_path_y = j[1]["previous_path_y"];
          vector<double> prev_path_x, prev_path_y;
          for (double xx : jprev_path_x) prev_path_x.push_back(xx);
          for (double yy : jprev_path_y) prev_path_y.push_back(yy);
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          road.updateNewPathState(prev_path_x, prev_path_y, end_path_s, end_path_d, car_x, car_y, car_s, car_d, deg2rad(car_yaw));

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          road.updateCarPositions(sensor_fusion);

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          vector<Point> pts;
          double ref_x = car_x, ref_y = car_y, ref_yaw = deg2rad(car_yaw);
          Vehicle ego{ int(ceil(car_d / 4.0)), car_s, car_speed, 0, car_d };
          ego.d = car_d; ego.x = car_x; ego.y = car_y; ego.yaw = ref_yaw;
          ego.target_speed = tg::max_speed; ego.max_acceleration = tg::max_accel;
          road.add_ego(ego);
          //tie(next_x_vals, next_y_vals) = 
          road.advance();

          if (prev_sz > 0) car_s = end_path_s;
          auto too_close = false;

          for (const auto& oth : sensor_fusion) { //id,x,y,vx,vy,s,d
            double oth_d = oth[6];
            //if (oth_d<(2 + 4*lane - 1.8) || oth_d>(2 + 4*lane + 1.8)) continue;
            double oth_vx = oth[3], oth_vy = oth[4], oth_s0 = oth[5], oth_s = oth[5];
            auto oth_speed = sqrt(oth_vx*oth_vx + oth_vy*oth_vy);
            oth_s += prev_sz*.02*oth_speed; // project where oth car will be; sensor_fusion is current, whereas prev_path is future state
            cout << endl << "Oth Car: id=" << oth[0] << ", [oth,me] S=(" << oth[5] << "," << j[1]["s"] << "). D=(" << oth[6] << "," << car_d
              << "). PROJ_S=(" << oth_s << "," << car_s << "). V=(" << oth_speed << "," << car_speed / 2.24 << ").";
            if (oth_d<(2 + 4 * lane - 2) || oth_d>(2 + 4 * lane + 2)) continue;
            cout << " *** in same lane ***";
            too_close = ((oth_s > car_s0) && ((oth_s - car_s) < 30.0)); // other car < 30m ahead
            if (too_close) {
              cout << "**** CLOSE Car: id=" << oth[0] << ", s diff=" << (oth_s - car_s) << " ****** ";
              break;
            }
          }
          cout << endl;
          if (too_close) ref_vel -= .224;
          else if (ref_vel < target_vel) ref_vel += .224;
          cout << "too_close=" << too_close << ", ref_vel=" << ref_vel << endl;

          if (prev_sz < 2) { // if prev path is almost empty, use car as starting point
            pts.push_back({ car_x - cos(ref_yaw), car_y - sin(ref_yaw) }); // prev car point x, y project back in time
            pts.push_back({ car_x, car_y });
          }
          else { // use the previous path's end point as starting ref
            ref_x = prev_path_x[prev_sz - 1];             ref_y = prev_path_y[prev_sz - 1];
            double ref_x_prev = prev_path_x[prev_sz - 2], ref_y_prev = prev_path_y[prev_sz - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            pts.push_back({ ref_x_prev, ref_y_prev });
            pts.push_back({ ref_x,      ref_y });
          }

          Vehicle ref{ 0, 0, 0, 0, 0 }; ref.x = ref_x; ref.y = ref_y; ref.yaw = ref_yaw;

          // get evenly spaced points
          auto wp0 = road.getXY(car_s + 30, (2 + 4 * lane));
          auto wp1 = road.getXY(car_s + 60, (2 + 4 * lane));
          auto wp2 = road.getXY(car_s + 90, (2 + 4 * lane));
          pts.push_back(wp0);
          pts.push_back(wp1);
          pts.push_back(wp2);

          for (size_t i = 0; i < pts.size(); i++) { // translate to car's point of view
            pts[i] = ref.toVehicleCoordinates(pts[i]);
            cout << "to fit #" << i << ": (" << pts[i].x << "," << pts[i].y << ")" << endl;
          }

          for (auto i = 0; i < prev_sz; i++) {
            next_x_vals.push_back(prev_path_x[i]); next_y_vals.push_back(prev_path_y[i]);
            cout << "last pts #" << i << ": (" << next_x_vals[i] << "," << next_x_vals[i] << ")" << endl;
          }

          auto s = calcSpline(pts);
          auto target_x = 30.0;
          auto target_y = s(target_x);
          auto target_dist = sqrt(target_x*target_x + target_y*target_y);
          auto x_add_on = 0.0;

          for (auto i = 1; i <= 50 - prev_sz; i++) {
            auto N = target_dist / (0.02*ref_vel / 2.24); // 0.02 seconds apart, convert to m/s
            auto x_pt = x_add_on + target_x / N;
            auto y_pt = s(x_pt);
            x_add_on = x_pt;

            // translate back to global coords
            auto glb = ref.toGlobalCoordinates({ x_pt, y_pt });
            next_x_vals.push_back(glb.x); next_y_vals.push_back(glb.y);
            cout << "added pt #" << prev_sz + i << ": (" << glb.x << "," << glb.y << ")" << endl;
          }

          //next_x_vals.clear(); next_y_vals.clear();
          //double dist_inc = 0.425;
          //for (int i = 0; i < 50; i++)
          //{
          //  //lane follow #1: 
          //  auto ns = (i + 1)*dist_inc + car_s; auto nd = 6.0; auto xy = getXY(ns, nd, map_waypoints_s, map_waypoints_x, map_waypoints_y);next_x_vals.push_back(xy[0]);next_y_vals.push_back(xy[1]);
          //  //straight line: next_x_vals.push_back(car_x + (dist_inc*i)*cos(deg2rad(car_yaw))); next_y_vals.push_back(car_y + (dist_inc*i)*sin(deg2rad(car_yaw)));
          //}
          // END

          // SEND BACK
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          cout << msg << endl;
          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
                         char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
