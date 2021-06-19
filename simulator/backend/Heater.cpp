#include "backend/Heater.h"

void Heater::preprocessParticle(Particle* particle) {
  // cool a timed out particle
  if (particle->meltingTimeout() > 0 && particle->meltingTimeout() <= time_) {
    particle->setMolten(false);
    particle->setMovable(true);
  }
}

void Heater::processParticle(Particle* particle) {
  // heat around x, y
  if (distance(particle->point(), position_) <= size_) {
    particle->setMolten(true);
    particle->setMeltingTimeout(time_ + particle->settings()->moltenParticleCooldownTime());
    particle->setMovable(true);
  }
}

void Heater::postprocessParticle(Particle* particle) {
  // stop movement of just-frozen particles
  if (!particle->isMolten()) particle->setMovable(false);
}

void Heater::resetParticles(std::vector<Particle*>& particles) {
  // all molten particles after the last movement should now be allowed to cool
  // NOTE: this is only valid if there is only one heater
  // NOTE: this is only valid if particles do not accumulate heat
  for (auto& p : particles) {
    if (p->isMolten()) {
      p->setMolten(false);
      p->setMovable(true);
    }
  }
}
