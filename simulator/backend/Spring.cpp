#include "Spring.h"

void Spring::updateForce() {
  if (actualLength() < settings_->springDefaultLength() / 2) {
    // if the spring becomes too short, make the force larger
    force_ = settings_->springDefaultStiffness() * length_;
  } else
  force_ = settings_->springDefaultStiffness() * (length_ - actualLength());
}
