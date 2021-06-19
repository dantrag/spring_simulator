#ifndef SHAPE_H
#define SHAPE_H

#include <vector>

#include "backend/Particle.h"

class Shape {
 public:
  Shape(const std::vector<Point>& points);
  Shape(std::string filename);

  const std::vector<Point> points() const { return points_; }
  int n() const { return n_; }

  bool contains(const Point& point) const;

  double length() const;
  double area(bool oriented = false) const;
  Point centroid() const;
  bool clockwise() const;

  void moveTo(const Point& new_center);
  void scaleTo(double new_area);
  void scaleBy(double scale);

  Point sampleBoundary() const;
  Point sampleBoundary(double percentile) const;

  double distanceTo(const Shape& other, int samples = 100, bool hausdorff = false) const;

  Shape samplingContour(const std::vector<std::pair<double, double>>& vectors,
                        double margin) const;

  void saveToFile(std::string filename) const;

 private:
  std::vector<Point> points_;
  int n_;
};

#endif // SHAPE_H
