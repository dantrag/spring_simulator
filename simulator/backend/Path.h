#ifndef PATH_H
#define PATH_H

#include <vector>

#include "backend/Particle.h"

class Path {
 public:
  Path(const std::vector<Point>& points, bool is_cyclic)
      : points_(points), is_cyclic_(is_cyclic), n_(static_cast<int>(points.size())) {}

  double length();

  Point sampleFraction(double fraction);

 protected:
  std::vector<Point> points_ = {};
  bool is_cyclic_ = false;
  int n_ = 0;
};

#endif // PATH_H
