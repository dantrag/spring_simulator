#ifndef PUSHER_H
#define PUSHER_H

#include <functional>
#include <set>

#include "backend/Actuator.h"

class Pusher : public Actuator {
 public:
  Pusher() { firm_grip_ = true; }
  void enable() override;

  void preprocessParticle(Particle* particle) override;
  void processParticle(Particle* particle) override;
  void postprocessParticle(Particle* particle) override;

  void resetParticles(std::vector<Particle*>& particles) override;

  bool isSpringCrossingApplicable() const override { return true; };
  bool isFirmGripApplicable() const override { return true; }
  bool isFinalReleaseApplicable() const override { return true; }

  std::string generic_name() const override { return "Pusher"; }

 protected:
  virtual std::vector<std::string> compatible_names() override { return {"pusher", "picker", "gripper"}; }

 private:
  bool isParticleCaptured(Particle* particle);

  bool gripping_in_progress_ = false;
  // transparent comparator std::less<> to allow searching for const pointers
  std::set<Particle*, std::less<>> current_grip;
};

#endif // PUSHER_H
