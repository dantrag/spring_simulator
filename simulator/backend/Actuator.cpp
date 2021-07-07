#include "backend/Actuator.h"

Actuator::Actuator() {
  setShape(Shape({Point(0.0, 0.0),
                  Point(1.0, 0.0),
                  Point(1.0, 1.0),
                  Point(0.0, 1.0)}));
}

void Actuator::setEnabled(bool enabled) {
  if (on_ != enabled) {
    if (enabled)
      enable();
    else
      disable();
  }
}

void Actuator::setPathAdvancement(double cumulative_length) {
  last_position_ = position_;
  if (path_.length() > 1e-5) {
    position_ = path_.sampleFraction(cumulative_length / path_.length());
    path_advancement_ = cumulative_length;
  } else {
    if (path_.size() > 0)
      position_ = path_.points()[0];
    else
      position_ = Point(0.0, 0.0);
    path_advancement_ = 0.0;
  }
}

void Actuator::setShape(Shape shape) {
  shape_ = shape;
  capture_particle_check_ = std::move([shape, this](const Particle* particle) {
    Shape oriented_shape(shape);
    oriented_shape.rotateBy(orientation_);
    oriented_shape.moveTo(position_);
    return oriented_shape.contains(particle->point());
  });
}

ActuatorState Actuator::saveState() const {
  return ActuatorState(this);
}

void Actuator::loadState(const ActuatorState& state) {
  setEnabled(state.enabled);
  setTime(state.time);
  setPath(state.path);
  setPathAdvancement(state.path_advancement);
}

ActuatorState::ActuatorState(const Actuator* actuator) {
  enabled = actuator->enabled();
  path_advancement = actuator->pathAdvancement();
  time = actuator->time();
  path = actuator->path();
}
