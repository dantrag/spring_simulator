#include "backend/ParticleState.h"

ParticleState::ParticleState(double x, double y, double radius) {
  x_ = x;
  y_ = y;
  radius_ = radius;
}

ParticleState::ParticleState(const Particle *particle)
  : ParticleState(particle->x(), particle->y(), particle->radius()) {}
