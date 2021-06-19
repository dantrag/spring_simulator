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

  double size() { return size_; }
  void setSize(double size) { size_ = size; }

 private:
  double size_ = 0.0;
};

#endif // HEATER_H
