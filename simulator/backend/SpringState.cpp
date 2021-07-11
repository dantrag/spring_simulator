#include "backend/SpringState.h"

SpringState::SpringState(const ParticleState* p1, const ParticleState* p2,
                         double equilibrium_length, double force_constant)
    : particle1_(p1), particle2_(p2),
      equilibrium_length_(equilibrium_length),
      force_constant_(force_constant) {}
