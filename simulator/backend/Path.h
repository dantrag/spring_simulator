#ifndef PATH_H
#define PATH_H

#include <vector>
#include <string>

#include "backend/Particle.h"

class Path {
 public:
  Path() {}
  Path(const std::vector<Point>& points, bool is_cyclic = false)
      : points_(points), is_cyclic_(is_cyclic), n_(static_cast<int>(points.size())) {}

  double length() const;
  const std::vector<Point> points() const { return points_; }

  Point sampleFraction(double fraction) const ;

  std::string toString() const;

 protected:
  std::vector<Point> points_ = {};
  bool is_cyclic_ = false;
  int n_ = 0;
};

#endif // PATH_H
