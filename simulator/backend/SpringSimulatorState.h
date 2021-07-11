#ifndef SPRINGSIMULATORSTATE_H
#define SPRINGSIMULATORSTATE_H

#include <vector>

#include "pugixml/pugixml.hpp"

#include "backend/ParticleState.h"
#include "backend/SpringState.h"
#include "backend/Shape.h"

class SpringSimulator;

class SpringSimulatorState {
 public:
  SpringSimulatorState(const SpringSimulator* simulator, int id = -1);
  SpringSimulatorState(std::string xml_file);

  const std::vector<ParticleState*>& particles() const { return particles_; }
  const std::vector<SpringState*>& springs() const { return springs_; }
  int id() const { return id_; }

  Shape fieldContour();

  bool loadFromXML(std::string xml_file);
  void saveToXML(std::string filename) const;
  std::string toString() const;

 protected:
  pugi::xml_document toXML() const;
  void clear();

  std::vector<ParticleState*> particles_;
  std::vector<SpringState*> springs_;
  int id_ = -1;
};

#endif // SPRINGSIMULATORSTATE_H
