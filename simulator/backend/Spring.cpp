#include "Spring.h"

void Spring::updateForce() {
  if (actualLength() < length_) {
    force_ = (1.0 / actualLength() - 1.0 / length_) * settings_->springDefaultStiffness() * length_ * length_ / 2;
  } else {
    force_ = settings_->springDefaultStiffness() * (length_ - actualLength());
  }
}
