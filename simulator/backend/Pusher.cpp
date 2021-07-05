#include "backend/Pusher.h"

#include "backend/Spring.h"

bool Pusher::isParticleCaptured(Particle* particle) {
  if (isFirmGrip() && !gripping_in_progress_) {
    return current_grip.count(particle) > 0;
  } else {
    return capture_particle_check_(particle);
  }
}

void Pusher::enable() {
  Actuator::enable();
  gripping_in_progress_ = true;
}

void Pusher::preprocessParticle(Particle* particle) {
  particle->setMovable(true);
}

void Pusher::processParticle(Particle* particle) {
  // move particles, captured by the pusher, and freeze them
  if (isParticleCaptured(particle)) {
    if (isFirmGrip() && gripping_in_progress_) {
      // if this is the first time after pusher was enabled,
      // grip this particle firmly (until it is released)
      current_grip.insert(particle);
    }

    auto displacement = Point(position_.x - particle->x(),
                              position_.y - particle->y());

    if (!isSpringCrossingAllowed()) {
      // if spring crossing is not allowed, check that the displacement
      // will not cross any of the springs in the neighbourhood (distance 2)
      std::unordered_set<Particle*> neighbourhood;
      particleBFS(particle, 1, 1, neighbourhood);
      Point new_point(particle->x() + displacement.x,
                      particle->y() + displacement.y);
      // initialize clearance radius as double a displacement norm
      double clearance_radius = distance(displacement, Point(0, 0)) * 2;
      if (clearance_radius > 1e-5) {
        for (auto neighbour : neighbourhood) {
          for (auto spring : neighbour->springs()) {
            if (spring->otherEnd(neighbour) == particle) continue;

            if (segmentsIntersect(particle->point(), new_point,
                                  neighbour->point(), spring->otherEnd(neighbour)->point())) {
              clearance_radius = std::min(clearance_radius,
                                          distance(particle->point(),
                                                   neighbour->point(),
                                                   spring->otherEnd(neighbour)->point()));
            }
          }
        }
        if (clearance_radius < particle->settings()->relaxationConvergenceLimit()) clearance_radius = 0.0;
        // allow only a displacement of half of the clearance radius
        double original_displacement_norm = distance(displacement, Point(0, 0));
        double scale_factor = clearance_radius / 2 / original_displacement_norm;
        displacement.x *= scale_factor;
        displacement.y *= scale_factor;
      }
    }
    particle->setDisplacement(displacement);
    particle->applyDisplacement();
    particle->setMovable(false);
  }
}

void Pusher::postprocessParticle(Particle* particle) {
  particle->setMovable(false);
  gripping_in_progress_ = false;
}

void Pusher::resetParticles(std::vector<Particle*>& particles) {
  // optionally - release pusher grip and let all particles move
  for (auto& p : particles) {
    p->setMovable(true);
  }
  current_grip.clear();
}
