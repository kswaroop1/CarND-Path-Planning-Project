#ifndef WAY_POINT_H
#define WAY_POINT_H
#include <vector>
#include <map>

struct Point { double x, y; };
struct WayPoint { double x, y, s, dx, dy; };

struct triple {
  double a0, a1, a2;  // represents factor x, its first derivative xd, and second derivative xdd

  triple operator+(const triple& o) const { return {a0+o.a0, a1+o.a1, a2+o.a2}; }
  triple after(double dt) const { return { a0 + a1*dt + a2*dt*dt / 2.0, a1 + a2*dt, a2 }; }
  double f0(double x)     const { return a0 + a1*x + a2*x*x; } // value
  double f1(double x)     const { return a1 + 2*a2*x; }        // first deriv
  double f2(double x)     const { return 2*a2; }               // second deriv
};

struct state {
  triple s, d;  // combined s and d triples for trajectory and JMT
  double t;

  state operator+(const state& o) const { return { s + o.s, d + o.d, t }; }
  state after(double dt)  const { return { s.after(dt), d.after(dt), t }; }
  double nearest_approach(const state& o, const double T) const;
  double nearest_approach_to_any_vehicle(const std::vector<state>& vehicles) const;
};

#endif // WAY_POINT_H
