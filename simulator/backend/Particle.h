#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <unordered_set>

#include "backend/SimulatorSettings.h"

class Spring;

struct Point {
  Point(double X, double Y) : x(X), y(Y) {}
  bool operator== (const Point& other) {
    return x == other.x && y == other.y;
  }

  double x = 0.0;
  double y = 0.0;
};

struct Line {
  Line() {}
  Line(const Point& p1, const Point& p2) {
    a = 1;
    if (p1.x == p2.x) {
      b = 0;
      c = -a * p1.x;
    } else {
      b = - a * (p2.x - p1.x) / (p2.y - p1.y);
      c = -a * p1.x - b * p1.y;
    }
  }

  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
};

class Particle {
 public:
  Particle() {}
  Particle(double x, double y, SimulatorSettings* settings) : x_(x), y_(y), settings_(settings) {}
  Particle(const Particle* particle);
  ~Particle();

  Point point() const { return Point(x_, y_); }
  double x() const { return x_; }
  double y() const { return y_; }
  double radius() const { return molten_ ? settings_->moltenParticleDefaultRadius()
                                         : settings_->particleDefaultRadius(); }

  void setDisplacement(const Point& displacement) { displacement_ = displacement; }
  void applyDisplacement() { x_ += displacement_.x; y_ += displacement_.y; }

  // molten implies larger radius; mobility is set separately
  bool isMolten() const { return molten_; }
  void setMolten(bool molten) { molten_ = molten; if (!molten_) melting_timeout_ = -1; }
  int meltingTimeout() const { return melting_timeout_; }
  void setMeltingTimeout(int timeout) { melting_timeout_ = timeout; }

  // mark that this particle is allowed to relax its new state (move)
  bool isMovable() const { return movable_; }
  void setMovable(bool movable) { movable_ = movable; }

  void addSpring(Spring* spring) { springs_.push_back(spring); }
  const std::vector<Spring*>& springs() const { return springs_; }
  void removeString(Spring* spring);

  void setSettings(SimulatorSettings* settings) { settings_ = settings; }
  const SimulatorSettings* settings() const { return settings_; }

 protected:
  double x_ = 0;
  double y_ = 0;
  Point displacement_ = Point(0.0, 0.0);

  bool molten_ = false;
  int melting_timeout_ = -1;

  bool movable_ = false;

  std::vector<Spring*> springs_ = {};

  const SimulatorSettings* settings_ = nullptr;
};

double distance(double x1, double y1, double x2, double y2);
double distance2(double x1, double y1, double x2, double y2);
double distance(const Point& p, double x, double y);
double distance(const Point& p1, const Point& p2);
double distance2(const Point& p1, const Point& p2);
double distance(const Particle* p, double x, double y);
double distance(const Particle* p1, const Particle* p2);
double distance(const Particle* p1, const Particle* p2, const Particle* p3);
double distance(const Point& p1, const Point& p2, const Point& p3, bool segment = true);
double crossProduct(const Point& p1, const Point& p2, const Point& p3);
bool segmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4);

void particleBFS(Particle* start, int minimum_depth, int maximum_depth,
                 std::unordered_set<Particle*>& neighbourhood);

#endif // PARTICLE_H
