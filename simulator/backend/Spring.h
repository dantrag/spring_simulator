#ifndef SPRING_H
#define SPRING_H

#include "backend/Particle.h"
#include "backend/SimulatorSettings.h"

class Spring {
 public:
  Spring(Particle* p1, Particle* p2, double length, SimulatorSettings* settings)
      : length_(length), ends_(std::make_pair(p1, p2)), settings_(settings) {
    p1->addSpring(this);
    p2->addSpring(this);
  }

  Particle* particle1() const { return ends_.first; }
  Particle* particle2() const { return ends_.second; }
  Particle* otherEnd(const Particle* one_end) {
    if (ends_.first == one_end) return ends_.second;
    if (ends_.second == one_end) return ends_.first;
    return nullptr;
  }

  double length() const { return length_; }
  double actualLength() const {
    return distance(ends_.first, ends_.second) - ends_.first->radius() - ends_.second->radius();
  }

  double force() const { return force_; }
  void updateForce();

  void setSettings(SimulatorSettings* settings) { settings_ = settings; }
  SimulatorSettings* settings() { return settings_; }

 private:
  // equilibrium length
  double length_ = 0.0;
  double force_ = 0.0;

  std::pair<Particle*, Particle*> ends_ = {};

  SimulatorSettings* settings_ = nullptr;
};

#endif // SPRING_H
