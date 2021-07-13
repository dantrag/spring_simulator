#ifndef INELASTICSIMULATOR_H
#define INELASTICSIMULATOR_H

#include "backend/SpringSimulator.h"

class InelasticSimulator : public SpringSimulator {
 public:
  using SpringSimulator::SpringSimulator;

  std::string generic_name() const override { return "Inelastic-simulator"; }

 protected:
  void updateConnectivity() override;

  virtual std::vector<std::string> compatible_names() { return {"wax-simulator", "inelastic-simulator"}; }
};

#endif // INELASTICSIMULATOR_H
