#include "Spring.h"

void Spring::updateForce() {
  force_ = settings_->springDefaultStiffness() *
      (length_ - (distance(ends_.first, ends_.second) - ends_.first->radius() - ends_.second->radius()));
}
