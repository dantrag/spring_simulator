#ifndef SPRINGSTATE_H
#define SPRINGSTATE_H

#include "backend/Particle.h"
#include "backend/ParticleState.h"

class SpringState {
 public:
  SpringState(const ParticleState* p1, const ParticleState* p2,
              double equilibrium_length, double force_constant);

  const ParticleState* particle1() const { return particle1_; }
  const ParticleState* particle2() const { return particle2_; }
  double equilibriumLength() const { return equilibrium_length_; }
  double actualLength() const { return distance(particle1_->point(), particle2_->point()); }
  double forceConstant() const { return force_constant_; }
  double stretch() const { return actualLength() / equilibrium_length_; }

 private:
  const ParticleState* particle1_ = nullptr;
  const ParticleState* particle2_ = nullptr;
  double equilibrium_length_ = 0.0;
  double force_constant_ = 0.0;
};

#endif // SPRINGSTATE_H
