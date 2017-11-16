#ifndef WAY_POINT_H
#define WAY_POINT_H
#include <vector>

static constexpr int lanes_available = 3;
static constexpr int lane_width = 4;

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

struct tg_state {
  triple s, d;  // combined s and d triples for trajectory and JMT
  double t;

  tg_state operator+(const tg_state& o) const { return { s + o.s, d + o.d, t }; }
  tg_state after(double dt)  const { return { s.after(dt), d.after(dt), t }; }
  double nearest_approach(const tg_state& o, const double T) const;
  double nearest_approach_to_any_vehicle(const std::vector<tg_state>& vehicles) const;
};

#endif // WAY_POINT_H
