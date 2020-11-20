#include "Particle.h"

#include <cmath>

double distance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double distance(const Point& p, double x, double y) {
  return std::sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
}

double distance(const Point& p1, const Point& p2) {
  return distance(p1.x, p1.y, p2.x, p2.y);
}

double distance(const Particle* p, double x, double y) {
  return distance(p->point(), x, y);
}

double distance(const Particle* p1, const Particle* p2) {
  return distance(p1->point(), p2->point());
}

void Particle::removeString(Spring* spring) {
  for (size_t i = 0; i < springs_.size(); ++i) {
    if (springs_[i] == spring) {
      springs_[i] = nullptr;
    }
    if (springs_[i] == nullptr && i + 1 < springs_.size()) {
      springs_[i] = springs_[i + 1];
      springs_[i + 1] = nullptr;
    }
  }

  // if the spring was deleted, last item appears empty
  if (*springs_.rbegin() == nullptr) springs_.pop_back();
}
