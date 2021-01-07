#include "backend/ParticleState.h"

#include "backend/Particle.h"

ParticleState::ParticleState(const Particle *particle) {
  x_ = particle->x();
  y_ = particle->y();
  radius_ = particle->radius();
}
