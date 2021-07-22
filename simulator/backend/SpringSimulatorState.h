#ifndef SPRINGSIMULATORSTATE_H
#define SPRINGSIMULATORSTATE_H

#include <vector>
#include <unordered_map>

#include "backend/XMLIO.h"
#include "backend/ParticleState.h"
#include "backend/SpringState.h"
#include "backend/Shape.h"

class SpringSimulator;

// Contains only crucial geometric information - postions of particles
// and connectivity; type of simulator etc. is not available
class SpringSimulatorState : public XMLIO {
 public:
  SpringSimulatorState() {}
  SpringSimulatorState(const SpringSimulator* simulator, int id = -1);
  SpringSimulatorState(std::string xml_file);
  ~SpringSimulatorState();

  const std::vector<ParticleState*>& particles() const { return particles_; }
  const std::vector<SpringState*>& springs() const { return springs_; }
  int id() const { return id_; }

  ParticleState* getItemState(Particle* particle) const;
  SpringState* getItemState(Spring* spring) const;

  Shape fieldContour();

  bool loadFromXMLNode(pugi::xml_node root) override;
  bool loadFromXML(std::string xml_file) override;
  pugi::xml_document toXML() const override;

 protected:
  void clear();

  std::unordered_map<Particle*, ParticleState*> particle_state_mapping_;
  std::unordered_map<Spring*, SpringState*> spring_state_mapping_;

  std::vector<ParticleState*> particles_;
  std::vector<SpringState*> springs_;
  int id_ = -1;
};

#endif // SPRINGSIMULATORSTATE_H
