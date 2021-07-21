#ifndef SPRING_H
#define SPRING_H

#include "backend/Particle.h"
#include "backend/SimulatorSettings.h"

class Spring {
 public:
  Spring(Particle* p1, Particle* p2, double length, double force_constant)
      : length_(length), force_constant_(force_constant), ends_(std::make_pair(p1, p2)) {
    p1->addSpring(this);
    p2->addSpring(this);
  }

  inline Particle* particle1() const { return ends_.first; }
  inline Particle* particle2() const { return ends_.second; }
  inline Particle* otherEnd(const Particle* one_end) {
    if (ends_.first == one_end) return ends_.second;
    if (ends_.second == one_end) return ends_.first;
    return nullptr;
  }

  inline double length() const { return length_; }
  void setLength(double length) { length_ = length; }
  inline double actualLength() const {
    return distance(ends_.first, ends_.second) - ends_.first->radius() - ends_.second->radius();
  }

  inline double forceConstant() const { return force_constant_; }
  void setForceConstant(double force_constant) { force_constant_ = force_constant; }

  inline double force() const { return force_; }
  void updateForce();

 private:
  // equilibrium length
  double length_ = 0.0;
  double force_constant_ = 0.0;
  double force_ = 0.0;

  std::pair<Particle*, Particle*> ends_ = {};
};

#endif // SPRING_H
