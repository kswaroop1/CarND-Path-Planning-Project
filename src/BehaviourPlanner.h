#ifndef BEHAVIOUR_PLANNER_H
#define BEHAVIOUR_PLANNER_H
#include <vector>
#include <map>
#include <string>
#include "WayPoint.h"

struct collider {
  bool collision; // is there a collision?
  int  time; // time collision happens
};

#endif
