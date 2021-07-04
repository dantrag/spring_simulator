#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <vector>

#include "backend/Particle.h"
#include "backend/Path.h"

class Actuator {
 public:
  Actuator() {}
  virtual ~Actuator() {}

  bool enabled() const { return on_; }
  virtual void enable() { on_ = true; }
  virtual void disable() { on_ = false; }

  double speed() const { return speed_; }
  void setSpeed(double speed) { speed_ = speed; }
  void setTime(int time) { time_ = time; }

  Point position() const { return position_; }
  void setPathAdvancement(double cumulative_length);

  const Path& path() { return path_; }
  void setPath(Path path) { path_ = path; setPathAdvancement(0.0); }

  virtual void preprocessParticle(Particle* particle) = 0;
  virtual void processParticle(Particle* particle) = 0;
  virtual void postprocessParticle(Particle* particle) = 0;

  virtual void resetParticles(std::vector<Particle*>& particles) = 0;

 protected:
  bool on_ = false;
  double speed_ = 1.0;
  int time_ = 0;
  Point position_ = Point(0.0, 0.0);
  Path path_;
};

#endif // ACTUATOR_H
