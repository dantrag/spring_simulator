#include "backend/Shape.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "backend/Path.h"

Shape::Shape(const std::vector<Point>& points)
  : points_(points), n_(static_cast<int>(points.size())) {}

Shape::Shape(const Shape& shape)
  : Shape(shape.points()) {}

Shape::Shape(std::string filename) {
  std::ifstream shape_file;
  shape_file.open(filename);
  std::string line;
  // read past the first line with column names
  std::getline(shape_file, line);
  while (std::getline(shape_file, line)) {
    std::stringstream line_stream(line);
    std::string substring;
    std::stringstream spaced_values;
    while (std::getline(line_stream, substring, ',')) {
      spaced_values << substring << " ";
    }
    double x = 0.0, y = 0.0;
    spaced_values >> x >> y;
    // avoid duplicates due to the rounding
    if (!points_.empty() && *points_.rbegin() == Point(x, y)) continue;
    points_.push_back(Point(x, y));
  }
  shape_file.close();
}

bool Shape::contains(const Point& point) const {
  int winding_number = 0;

  for (int i = 0; i < n_; i++) {
    const auto& current = points_[i];
    const auto& next = (i < n_ - 1) ? points_[i + 1]
                                    : points_[0];
    if (current.y <= point.y) {
      if (next.y > point.y) {
        if (sign(crossProduct(current, next, point)) > 0) winding_number++;
      }
    } else {
      if (next.y <= point.y) {
        if (sign(crossProduct(current, next, point)) < 0) winding_number--;
      }
    }
  }

  return winding_number != 0;
}

bool Shape::contains_ray_casting(const Point& point) const {
  int intersections = 0;
  if (n_ < 3) return false;

  double max_x = points_[0].x;
  double max_y = points_[0].y;
  for (const auto& shape_point : points_) {
    max_x = std::max(max_x, shape_point.x);
    max_y = std::max(max_y, shape_point.y);
  }
  // create a ray from the point, make sure it does not hit a vertex
  Point infinity(max_x + 111.1, max_y + 100.0);

  for (int i = 0; i < n_; ++i) {
    auto& p1 = points_[i];
    auto& p2 = points_[(i + 1) % n_];
    if (segmentsIntersect(p1, p2, point, infinity)) intersections++;
  }
  if (intersections > 2) {
    intersections -= 2;
  }
  return (intersections & 1);
}

double Shape::perimeter() const {
  Path boundary(points_, true);
  return boundary.length();
}

double Shape::diameter() const {
  double diameter = 0.0;
  for (int i = 0; i < n_ - 1; ++i) {
    for (int j = i + 1; j < n_; ++j) {
      diameter = std::max(diameter, distance(points_[i], points_[j]));
    }
  }
  return diameter;
}

double Shape::area(bool oriented) const {
  double a = 0.0;
  for (int i = 0; i < n_; ++i) {
    auto& p1 = points_[i];
    auto& p2 = points_[(i + 1) % n_];

    a += (p2.x - p1.x) *
         (p2.y + p1.y);
  }
  if (!oriented) a = std::abs(a);
  return a / 2;
}

Point Shape::centroid() const {
  Point center(0.0, 0.0);
  if (n_) {
    for (const auto& point : points_) {
      center.x += point.x;
      center.y += point.y;
    }
    center.x /= n_;
    center.y /= n_;
  }
  return center;
}

bool Shape::clockwise() const {
  auto a = this->area(true);
  return (a > 0);
}

void Shape::moveTo(const Point& new_center) {
  auto center = this->centroid();
  for (auto& point : points_) {
    point.x -= (center.x - new_center.x);
    point.y -= (center.y - new_center.y);
  }
}

void Shape::scaleTo(double new_area) {
  double area = this->area();
  if (area < 1e-5) return;

  double scale = std::sqrt(new_area / area);
  this->scaleBy(scale);
}

void Shape::scaleBy(double scale) {
  auto center = this->centroid();
  for (auto& point : points_) {
    point.x = center.x + (point.x - center.x) * scale;
    point.y = center.y + (point.y - center.y) * scale;
  }
}

void Shape::rotateBy(double angle) {
  auto center = this->centroid();
  for (auto& point : points_) {
    auto x = point.x;
    auto y = point.y;
    point.x = center.x + (x - center.x) * std::cos(angle) - (y - center.y) * std::sin(angle);
    point.y = center.y + (x - center.x) * std::sin(angle) + (y - center.y) * std::cos(angle);
  }
}

Point Shape::sampleBoundary(double fraction) const {
  Path boundary(points_, true);
  return boundary.sampleFraction(fraction);
}

Point Shape::sampleBoundary() const {
  double fraction = static_cast<double>(std::rand()) / RAND_MAX;
  return this->sampleBoundary(fraction);
}

double Shape::distanceTo(const Shape& other, int samples, bool hausdorff) const {
  const int sample_points1 = samples;
  std::vector<Point> sampled1;
  for (int i = 0; i < sample_points1; ++i) {
    sampled1.push_back(this->sampleBoundary(double(i) / sample_points1));
  }

  const int sample_points2 = hausdorff ? other.perimeter() / this->perimeter() * sample_points1
                                       : sample_points1;
  std::vector<Point> sampled2;
  for (int i = 0; i < sample_points2; ++i) {
    sampled2.push_back(other.sampleBoundary(double(i) / sample_points2));
  }

  // align two sets of points
  int closest_to_point0 = 0;
  for (int i = 1; i < static_cast<int>(sampled2.size()); ++i) {
    if (distance2(sampled1[0], sampled2[i]) < distance2(sampled1[0], sampled2[closest_to_point0]))
      closest_to_point0 = i;
  }
  std::rotate(sampled2.begin(), sampled2.begin() + closest_to_point0, sampled2.end());
  if (this->clockwise() != other.clockwise()) {
    std::reverse(sampled2.begin(), sampled2.end());
  }

  double diff = 0.0;
  if (hausdorff) {
    for (const auto& p1 : sampled1) {
      double dist = std::numeric_limits<double>::max();
      for (const auto& p2 : sampled2) {
        dist = std::min(dist, distance2(p1, p2));
      }
      diff = std::max(diff, dist);
    }
    for (const auto& p1 : sampled2) {
      double dist = std::numeric_limits<double>::max();
      for (const auto& p2 : sampled1) {
        dist = std::min(dist, distance2(p1, p2));
      }
      diff = std::max(diff, dist);
    }
  } else {
    for (int i = 0; i < static_cast<int>(sampled1.size()) &&
                    i < static_cast<int>(sampled2.size()); ++i) {
      diff += distance2(sampled1[i], sampled2[i]);
      //std::cout << "distance2 = " << distance2(sampled1[i], sampled2[i]) << std::endl;
    }
  }
  return std::sqrt(diff);
}

void Shape::saveToFile(std::string filename) const {
  // in CSV format
  std::ofstream output_file(filename);
  output_file << "X,Y" << std::endl;
  for (const auto& point : points_) {
    output_file << point.x << "," << point.y << std::endl;
  }
  output_file.close();
}

Shape Shape::samplingContour(const std::vector<std::pair<double, double>>& vectors,
                             double margin) const {
  std::vector<Point> sampling_contour;
  for (const auto& point : this->points()) {
    double separation_from_shape = -1.0;
    Point best_sampling_point(0.0, 0.0);
    for (const auto& radius : vectors) {
      Point sampling_point(point.x + radius.first * margin,
                           point.y + radius.second * margin);
      bool point_is_inside = this->contains(sampling_point);
      if ((margin > 0) == point_is_inside) continue;

      double separation = std::numeric_limits<double>::max();
      for (const auto& contour_point : this->points()) {
        separation = std::min(distance(sampling_point, contour_point), separation);
      }
      if (separation > separation_from_shape && separation != std::numeric_limits<double>::max()) {
        best_sampling_point = sampling_point;
        separation_from_shape = separation;
      }
    }
    if (separation_from_shape > 0) sampling_contour.push_back(best_sampling_point);
  }
  return Shape(sampling_contour);
}
