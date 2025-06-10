#pragma once
#include <vector>
struct Waypoint2D { double x, y; };
struct TimedPoint2D { double t, x, y; };

class TrapezoidalPlanner {
public:
  TrapezoidalPlanner(double v_cruise, double a_max);
  std::vector<TimedPoint2D> plan(const Waypoint2D& start, const Waypoint2D& end);
private:
  double v_cruise_, a_max_;
};