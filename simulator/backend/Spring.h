#ifndef SPRING_H
#define SPRING_H

#include "backend/Particle.h"
#include "backend/SimulatorSettings.h"

class Spring {
 public:
  Spring(Particle* p1, Particle* p2, double length, SimulatorSettings* settings)
      : length_(length), ends_(std::make_pair(p1, p2)), settings_(settings) {
    p1->addSpring(this);
    p2->addSpring(this);
  }

  Particle* particle1() const { return ends_.first; }
  Particle* particle2() const { return ends_.second; }
  Particle* otherEnd(const Particle* one_end) {
    if (ends_.first == one_end) return ends_.second;
    if (ends_.second == one_end) return ends_.first;
    return nullptr;
  }

  double length() const { return length_; }
  double actualLength() const {
    return distance(ends_.first, ends_.second) - ends_.first->radius() - ends_.second->radius();
  }

  double force() const { return force_; }
  void updateForce();

  void setSettings(SimulatorSettings* settings) { settings_ = settings; }

 private:
  // equilibrium length
  double length_ = 0.0;
  double force_ = 0.0;

  std::pair<Particle*, Particle*> ends_ = {};

  SimulatorSettings* settings_ = nullptr;
};

/*
struct Spring {
  std::pair<Particle*, Particle*> ends;
  double length = 0.0;
  double force = 0.0;
  QGraphicsLineItem* ui = nullptr;
  Spring(Particle* p1, Particle* p2, double equilibrium_length)
      : ends(std::make_pair(p1, p2)), length(equilibrium_length) {
    p1->springs.push_back(this);
    p2->springs.push_back(this);
  }
  ~Spring() {
    ui->scene()->removeItem(ui);
    delete ui;
    ui = nullptr;
  }
  Particle* otherEnd(Particle* one_end) {
    if (ends.first == one_end) return ends.second;
    if (ends.second == one_end) return ends.first;
    return nullptr;
  }
  void render(QGraphicsScene* scene) {
    if (ui) {
      ui->setLine(this->ends.first->x, this->ends.first->y,
                  this->ends.second->x, this->ends.second->y);
    } else {
      ui = scene->addLine(this->ends.first->x, this->ends.first->y,
                          this->ends.second->x, this->ends.second->y,
                          QPen(Qt::darkGreen));
      ui->setZValue(10);
    }
  }

  // currently not used
  void resetLength() {
    length = distance(ends.first, ends.second) - ends.first->radius() - ends.second->radius();
    //length = kDefaultSpringLength;
  }
  void updateForce() {
    force = kStiffness * (length - (distance(ends.first, ends.second) - ends.first->radius() - ends.second->radius()));
  }
};
*/
#endif // SPRING_H
