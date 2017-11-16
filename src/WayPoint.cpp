#include "WayPoint.h"
#include "cost_functions.h"

double tg_state::nearest_approach(const tg_state& o, const double T) const {
  auto closest = 999999.0;
  for (auto i=0; i<100; i++) {
    auto t = T*i / 100.0;
    auto s_ = s.f0(t);
    auto d_ = d.f0(t);
    auto target = o.after(t);
    auto ss_ = s_ - target.s.a0;
    auto dd_ = d_ - target.d.a0;
    auto dist = sqrt(ss_*ss_ + dd_*dd_);
    closest = min(dist, closest);
  }
  return closest;
}
double tg_state::nearest_approach_to_any_vehicle(const std::vector<tg_state>& vehicles) const {
  auto closest = 999999.0;
  for (auto& v :vehicles) {
    auto dist = nearest_approach(v , t);
    closest = min(dist, closest);
  }
  return closest;
}
