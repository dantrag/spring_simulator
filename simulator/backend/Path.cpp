#include "backend/Path.h"

#include <sstream>

int Path::size() const {
  return n_;
}

double Path::length() const {
  if (n_ == 0) return 0.0;

  double length = 0.0;
  for (int i = 0; i < n_ - 1; ++i) {
    auto& p1 = points_[i];
    auto& p2 = points_[i + 1];
    length += distance(p1, p2);
  }
  if (is_cyclic_) {
    length += distance(points_[n_ - 1], points_[0]);
  }
  return length;
}

Point Path::sampleFraction(double fraction) const {
  if (n_ == 0) return Point(0, 0);

  auto length = this->length();
  double cumulative_length = 0.0;
  for (int i = 0; i < is_cyclic_ ? n_ : n_ - 1; ++i) {
    const auto& p1 = points_[i];
    const auto& p2 = points_[(i + 1) % n_];
    auto segment_length = distance(p1, p2);
    cumulative_length += segment_length;
    if (fraction * length <= cumulative_length) {
      double segment_prefix = (fraction * length - (cumulative_length - segment_length)) / segment_length;
      return Point(p1.x + (p2.x - p1.x) * segment_prefix,
                   p1.y + (p2.y - p1.y) * segment_prefix);
    }
  }
  return *points_.rbegin();
}

std::string Path::toString() const {
  std::stringstream stringstream;
  stringstream.precision(1);
  for (const auto& point : points_) {
    stringstream << point.x << " " << point.y << " ";
  }
  auto string = stringstream.str();
  string.erase(string.size() - 1);
  return string;
}
