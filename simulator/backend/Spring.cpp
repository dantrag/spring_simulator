#include "Spring.h"

void Spring::updateForce() {
  if (actualLength() < length_) {
    force_ = (1.0 / actualLength() - 1.0 / length_) * force_constant_ * length_ * length_ / 2;
  } else {
    force_ = force_constant_ * (length_ - actualLength());
  }
}
