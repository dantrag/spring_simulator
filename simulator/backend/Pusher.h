#ifndef PUSHER_H
#define PUSHER_H

#include <functional>

#include "backend/Actuator.h"

typedef std::function<bool(const Particle*)> CaptureFunction;

class Pusher : public Actuator {
 public:
  Pusher();

  void preprocessParticle(Particle* particle) override;
  void processParticle(Particle* particle) override;
  void postprocessParticle(Particle* particle) override;

  void resetParticles(std::vector<Particle*>& particles) override;

  void setCapture(CaptureFunction function) { capture_particle_ = std::move(function); }

 private:
  CaptureFunction capture_particle_;
};

#endif // PUSHER_H
