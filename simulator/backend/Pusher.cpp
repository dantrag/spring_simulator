#include "backend/Pusher.h"

Pusher::Pusher()
    : Actuator() {
  capture_particle_ = std::move([&](const Particle* particle) {
    return distance(position_, particle->point()) < 2;
  });
}

void Pusher::preprocessParticle(Particle* particle) {
  particle->setMovable(true);
}

void Pusher::processParticle(Particle* particle) {
  // move particles, captured by the pusher, and freeze them
  if (capture_particle_(particle)) {
    particle->setDisplacement(Point(position_.x - particle->x(),
                                    position_.y - particle->y()));
    particle->applyDisplacement();
    particle->setMovable(false);
  }
}

void Pusher::postprocessParticle(Particle* particle) {
  particle->setMovable(false);
}

void Pusher::resetParticles(std::vector<Particle*>& particles) {
  // optionally - release pusher grip and let all particles move
  for (auto& p : particles) {
    p->setMovable(false);
  }
}
