#include "Spring.h"

void Spring::updateForce() {
  auto actual_length = actualLength();
  if (actual_length < length_) {
    force_ = (1.0 / actual_length - 1.0 / length_) * force_constant_ * length_ * length_ / 2;
  } else {
    force_ = force_constant_ * (length_ - actual_length);
  }
}
