#ifndef SPRINGSIMULATORSTATE_H
#define SPRINGSIMULATORSTATE_H

#include <vector>

#include "backend/ParticleState.h"
#include "backend/SpringState.h"

class SpringSimulator;

class SpringSimulatorState {
 public:
  SpringSimulatorState(const SpringSimulator* simulator, int id = -1);

  const std::vector<ParticleState*>& particles() const { return particles_; }
  const std::vector<SpringState*>& springs() const { return springs_; }
  int id() const { return id_; }

 private:
  std::vector<ParticleState*> particles_;
  std::vector<SpringState*> springs_;
  int id_ = -1;
};

#endif // SPRINGSIMULATORSTATE_H
