#ifndef INELASTICSIMULATOR_H
#define INELASTICSIMULATOR_H

#include "backend/SpringSimulator.h"

class InelasticSimulator : public SpringSimulator {
 public:
  using SpringSimulator::SpringSimulator;

 protected:
  void updateConnectivity() override;
};

#endif // INELASTICSIMULATOR_H
