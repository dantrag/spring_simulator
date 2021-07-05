#include "backend/Actuator.h"

Actuator::Actuator() {
  setShape(Shape({Point(0.0, 0.0),
                  Point(1.0, 0.0),
                  Point(1.0, 1.0),
                  Point(0.0, 1.0)}));
}

void Actuator::setPathAdvancement(double cumulative_length) {
  position_ = path_.sampleFraction(cumulative_length / path_.length());
}

void Actuator::setShape(Shape shape) {
  shape_ = shape;
  capture_particle_check_ = std::move([&](const Particle* particle) {
    Shape oriented_shape(shape);
    oriented_shape.rotateBy(orientation_);
    return oriented_shape.contains(particle->point());
  });
}
