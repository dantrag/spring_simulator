#ifndef SPRINGSTATE_H
#define SPRINGSTATE_H

#include "backend/Particle.h"
#include "backend/ParticleState.h"

class SpringState {
 public:
  SpringState(const ParticleState* p1, const ParticleState* p2,
              double actual_length, double equilibrium_length);

  const ParticleState* particle1() const { return particle1_; }
  const ParticleState* particle2() const { return particle2_; }
  double actualLength() const { return actual_length_; }
  double equilibriumLength() const { return equilibrium_length_; }
  double stretch() const { return actual_length_ / equilibrium_length_; }

 private:
  const ParticleState* particle1_ = nullptr;
  const ParticleState* particle2_ = nullptr;
  double actual_length_ = 0.0;
  double equilibrium_length_ = 0.0;
};

#endif // SPRINGSTATE_H
