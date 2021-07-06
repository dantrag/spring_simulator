#include "backend/SpringSimulatorState.h"

#include <map>

#include "backend/Spring.h"
#include "backend/SpringSimulator.h"

SpringSimulatorState::SpringSimulatorState(const SpringSimulator* simulator, int id) {
  id_ = id;
  std::map<Particle*, ParticleState*> states = {};

  for (const auto p : simulator->particles()) {
    particles_.push_back(new ParticleState(p));
    states[p] = *particles_.rbegin();
  }

  for (const auto p : simulator->particles()) {
    for (const auto s : p->springs()) {
      // iterate only once each string
      if (p < s->otherEnd(p)) {
        springs_.push_back(new SpringState(states[p], states[s->otherEnd(p)],
                                           s->actualLength(), s->length()));
      }
    }
  }
}

Shape SpringSimulatorState::fieldContour() {
  std::vector<Particle*> particles;
  std::unordered_map<const ParticleState*, Particle*> state_to_particle;
  for (auto p : particles_) {
    auto particle = new Particle(p->x(), p->y(), nullptr);
    state_to_particle[p] = particle;
    particles.push_back(particle);
  }

  for (auto s : springs_) {
    new Spring(state_to_particle[s->particle1()],
               state_to_particle[s->particle2()],
               s->equilibriumLength(),
               nullptr);
  }
  auto shape = particlesContour(particles);
  for (auto& p : particles) {
    delete p;
  }
  return shape;
}
