#ifndef PUSHER_H
#define PUSHER_H

#include <functional>
#include <set>

#include "backend/Actuator.h"

class Pusher : public Actuator {
 public:
  void enable() override;

  void preprocessParticle(Particle* particle) override;
  void processParticle(Particle* particle) override;
  void postprocessParticle(Particle* particle) override;

  void resetParticles(std::vector<Particle*>& particles) override;

  bool isSpringCrossingAllowed() { return spring_crossing_allowed_; }
  void setSpringCrossing(bool allowed) { spring_crossing_allowed_ = allowed; }

  // means that the particle(s) captured in the first move is/are going to be the one(s)
  // being pushed until resetParticles() is called
  bool isFirmGrip() { return firm_grip_; }
  void setFirmGrip(bool firm) { firm_grip_ = firm; }

  std::string generic_name() const override { return "Pusher"; }

 private:
  bool isParticleCaptured(Particle* particle);

  bool spring_crossing_allowed_ = false;
  bool firm_grip_ = true;
  bool gripping_in_progress_ = false;
  // transparent comparator std::less<> to allow searching for const pointers
  std::set<Particle*, std::less<>> current_grip;
};

#endif // PUSHER_H
