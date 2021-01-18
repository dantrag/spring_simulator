#include "Particle.h"

#include <cmath>

#include "Spring.h"

struct Line {
  Line() {}
  Line(const Point& p1, const Point& p2) {
    a = 1;
    if (p1.x == p2.x) {
      b = 0;
      c = -p1.x;
    } else {
      b = - a * (p2.x - p1.x) / (p2.y - p1.y);
      c = -a * p1.x - b * p1.y;
    }
  }

  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
};

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

double distance(const Particle* p1, const Particle* p2, const Particle* p3) {
  return distance(p1->point(), p2->point(), p3->point());
}

double distance(const Point& p1, const Point& p2, const Point& p3) {
  Line l(p2, p3);
  double distance_to_line = std::abs(l.a * p1.x + l.b * p1.y + l.c) /
                            std::sqrt(l.a * l.a + l.b * l.b + l.c * l.c);
  return std::min(distance_to_line, std::min(distance(p1, p2), distance(p1, p3)));
}

double crossProduct(const Point& p1, const Point& p2, const Point& p3) {
  return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
}

bool segmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
  // not accounting for the three-on-a-line cases
  /*
   bool o1 = crossProduct(p1, p2, p3) > 0.0;
   bool o2 = crossProduct(p1, p2, p4) > 0.0;
   bool o3 = crossProduct(p3, p4, p1) > 0.0;
   bool o4 = crossProduct(p3, p4, p2) > 0.0;

   return (o1 != o2) && (o3 != o4);*/
   Line l1(p1, p2), l2(p3, p4);

   if (fabs(l1.a * l2.b - l2.a * l1.b) < 1e-5) {
     // parallel
     if (fabs(l1.b) < 1e-5) {
       // vertical
       return (std::min(std::max(p1.y, p2.y), std::max(p3.y, p4.y)) >
               std::max(std::min(p1.y, p2.y), std::min(p3.y, p4.y)));
     }
     if (fabs(l2.c / l2.b - l1.c / l1.b) >= 1e-5) {
       return false;
     } else {
       return (std::min(std::max(p1.x, p2.x), std::max(p3.y, p4.x)) >
               std::max(std::min(p1.x, p2.x), std::min(p3.y, p4.x)));
     }
   } else {
     // intersection point
     double D = l1.a * l2.b - l1.b * l2.a;
     double Dx = l1.b * l2.c - l1.c * l2.b;
     double Dy = l1.c * l2.a - l1.a * l2.c;
     double x = Dx / D;
     double y = Dy / D;
     if (fabs(l1.b) < 1e-5) {
       // l1 is vertical
       return (y - p1.y) * (y - p2.y) < 0;
     } else {
       return (x - p1.x) * (x - p2.x) < 0;
     }
   }
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

Particle::~Particle() {
  for (auto s : springs_) {
    s->otherEnd(this)->removeString(s);
    delete s;
  }
}
