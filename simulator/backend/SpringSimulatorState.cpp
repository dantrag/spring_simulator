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
