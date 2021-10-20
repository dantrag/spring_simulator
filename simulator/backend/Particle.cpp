#include "Particle.h"

#include <cmath>
#include <queue>
#include <unordered_map>
#include <iostream>

#include "Spring.h"

double distance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double distance2(double x1, double y1, double x2, double y2) {
  return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

double distance(const Point& p, double x, double y) {
  return std::sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
}

double distance(const Point& p1, const Point& p2) {
  return distance(p1.x, p1.y, p2.x, p2.y);
}

double distance2(const Point& p1, const Point& p2) {
  return distance2(p1.x, p1.y, p2.x, p2.y);
}

double distance(const Particle* p, double x, double y) {
  return distance(p->point(), x, y);
}

double distance(const Particle* p1, const Particle* p2) {
  return distance(p1->point(), p2->point());
}

double distance(const Particle* p1, const Particle* p2, const Particle* p3, bool segment) {
  return distance(p1->point(), p2->point(), p3->point(), segment);
}

double distance2(const Particle* p1, const Particle* p2, const Particle* p3, bool segment) {
  return distance2(p1->point(), p2->point(), p3->point(), segment);
}

double distance(const Point& p1, const Point& p2, const Point& p3, bool segment) {
  if (distance(p2, p3) < 1e-5) return distance(p1, p2);
  if (segment) {
    // calculate distance to segment
    auto segment_length_squared = distance2(p2, p3);
    auto dot_product = ((p1.x - p2.x) * (p3.x - p2.x) + (p1.y - p2.y) * (p3.y - p2.y));
    auto projection_fraction = std::max(0.0, std::min(1.0,
                                                      dot_product / segment_length_squared));
    Point projection(p2.x + projection_fraction * (p3.x - p2.x),
                     p2.y + projection_fraction * (p3.y - p2.y));
    return distance(p1, projection);
  } else {
    // calculate distance to line
    Line l(p2, p3);
    double distance_to_line = std::abs(l.a * p1.x + l.b * p1.y + l.c) /
                              std::sqrt(l.a * l.a + l.b * l.b + l.c * l.c);
    return distance_to_line;
  }
}

double distance2(const Point& p1, const Point& p2, const Point& p3, bool segment) {
  if (distance2(p2, p3) < 1e-10) return distance2(p1, p2);
  if (segment) {
    // calculate distance to segment
    auto segment_length_squared = distance2(p2, p3);
    auto dot_product = ((p1.x - p2.x) * (p3.x - p2.x) + (p1.y - p2.y) * (p3.y - p2.y));
    auto projection_fraction = std::max(0.0, std::min(1.0,
                                                      dot_product / segment_length_squared));
    Point projection(p2.x + projection_fraction * (p3.x - p2.x),
                     p2.y + projection_fraction * (p3.y - p2.y));
    return distance2(p1, projection);
  } else {
    // calculate distance to line
    Line l(p2, p3);
    auto line_evaluation = l.a * p1.x + l.b * p1.y + l.c;
    double distance_to_line_squared = line_evaluation * line_evaluation /
                                      (l.a * l.a + l.b * l.b + l.c * l.c);
    return distance_to_line_squared;
  }
}

double crossProduct(const Point& p1, const Point& p2, const Point& p3) {
  return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
}

inline double area(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool intersect_1(double a, double b, double c, double d) {
  if (a > b) std::swap(a, b);
  if (c > d) std::swap(c, d);
  return std::max(a, c) <= std::min(b, d);
}

int sign(double x) {
  if (std::abs(x) <= 1e-5) return 0;
  if (x < 0) return -1; else return 1;
}

bool segmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
  return intersect_1(p1.x, p2.x, p3.x, p4.x)
    && intersect_1 (p1.y, p2.y, p3.y, p4.y)
    && sign(area(p1, p2, p3)) * sign(area(p1, p2, p4)) <= 0
    && sign(area(p3, p4, p1)) * sign(area(p3, p4, p2)) <= 0;

  // not accounting for the three-on-a-line cases
  /*
   bool o1 = crossProduct(p1, p2, p3) > 0.0;
   bool o2 = crossProduct(p1, p2, p4) > 0.0;
   bool o3 = crossProduct(p3, p4, p1) > 0.0;
   bool o4 = crossProduct(p3, p4, p2) > 0.0;

   return (o1 != o2) && (o3 != o4);*/
   Line l1(p1, p2), l2(p3, p4);

   if (fabs(l1.a * l2.b - l2.a * l1.b) < 1e-9) {
     // parallel
     if (fabs(l1.b) < 1e-9) {
       // vertical
       return (std::min(std::max(p1.y, p2.y), std::max(p3.y, p4.y)) >=
               std::max(std::min(p1.y, p2.y), std::min(p3.y, p4.y)));
     }
     if (fabs(l2.c / l2.b - l1.c / l1.b) >= 1e-9) {
       return false;
     } else {
       return (std::min(std::max(p1.x, p2.x), std::max(p3.x, p4.x)) >=
               std::max(std::min(p1.x, p2.x), std::min(p3.x, p4.x)));
     }
   } else {
     // intersection point
     double D = l1.a * l2.b - l1.b * l2.a;
     double Dx = l1.b * l2.c - l1.c * l2.b;
     double Dy = l1.c * l2.a - l1.a * l2.c;
     double x = Dx / D;
     double y = Dy / D;
     bool in_first = false;
     bool in_second = false;
     if (fabs(l1.b) < 1e-9) {
       // l1 is vertical
       in_first = (y - p1.y) * (y - p2.y) < 0;
     } else {
       in_first = (x - p1.x) * (x - p2.x) < 0;
     }
     if (fabs(l2.b) < 1e-9) {
       // l2 is vertical
       in_second = (y - p3.y) * (y - p4.y) < 0;
     } else {
       in_second = (x - p3.x) * (x - p4.x) < 0;
     }
     return (in_first && in_second);
   }
}

Particle::Particle(const Particle* particle) : settings_(particle->settings_) {
  x_ = particle->x();
  y_ = particle->y();
  molten_ = particle->isMolten();
  movable_ = particle->isMovable();
  melting_timeout_ = particle->meltingTimeout();
}

void Particle::removeSpring(Spring* spring) {
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

std::pair<double, double> Particle::netForce() const {
  double force_x = 0.0;
  double force_y = 0.0;

  for (const auto spring : springs_) {
    auto dx = spring->otherEnd(this)->x() - this->x();
    auto dy = spring->otherEnd(this)->y() - this->y();
    auto length = std::sqrt(dx * dx + dy * dy);
    if (length < 1e-5) continue;
    dx /= length;
    dy /= length;
    force_x -= dx * spring->force();
    force_y -= dy * spring->force();
  }

  return std::make_pair(force_x, force_y);
}

Particle::~Particle() {
  for (auto s : springs_) {
    s->otherEnd(this)->removeSpring(s);
    delete s;
  }
}

void particleBFS(Particle* start, int minimum_depth, int maximum_depth,
                 std::unordered_set<Particle*>& neighbourhood) {
  std::queue<Particle*> bfs_queue = {};
  std::unordered_map<Particle*, int> depth = {};
  bfs_queue.push(start);
  depth[start] = 0;
  while (!bfs_queue.empty()) {
    auto p = bfs_queue.front();
    bfs_queue.pop();
    if (minimum_depth <= depth[p] && depth[p] <= maximum_depth) {
      neighbourhood.insert(p);
    }
    if (depth[p] > maximum_depth) break;
    for (auto s : p->springs()) {
      auto next = s->otherEnd(p);
      if (!depth.count(next)) {
        bfs_queue.push(next);
        depth[next] = depth[p] + 1;
      }
    }
  }
}
