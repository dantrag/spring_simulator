#include "backend/SpringState.h"

SpringState::SpringState(const ParticleState* p1, const ParticleState* p2,
                         double actual_length, double equilibrium_length)
    : particle1_(p1), particle2_(p2),
      actual_length_(actual_length), equilibrium_length_(equilibrium_length) {}
