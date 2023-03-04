#ifndef HEATER_H
#define HEATER_H

#include "backend/Actuator.h"

class Heater : public Actuator {
 public:
  Heater() {}

  void preprocessParticle(Particle* particle) override;
  void processParticle(Particle* particle) override;
  void postprocessParticle(Particle* particle) override;

  void resetParticles(std::vector<Particle*>& particles) override;

  double radius() { return radius_; }
  void setRadius(double radius) { radius_ = radius; }

  std::string generic_name() const override { return "Heater"; }

 protected:
  virtual std::vector<std::string> compatible_names() override { return {"heater", "laser"}; }

 private:
  double radius_ = 0.0;
};

#endif // HEATER_H
