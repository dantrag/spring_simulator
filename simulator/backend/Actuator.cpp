#include "backend/Actuator.h"

void Actuator::setPathAdvancement(double cumulative_length) {
  position_ = path_.sampleFraction(cumulative_length / path_.length());
}
