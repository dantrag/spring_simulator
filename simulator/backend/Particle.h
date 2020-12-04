#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>

#include "backend/SimulatorSettings.h"

class Spring;

struct Point {
  Point(double X, double Y) : x(X), y(Y) {}

  double x = 0.0;
  double y = 0.0;
};

class Particle {
 public:
  Particle(int x, int y, SimulatorSettings* settings) : x_(x), y_(y), settings_(settings) {}
  ~Particle();

  Point point() const { return Point(x_, y_); }
  double x() const { return x_; }
  double y() const { return y_; }
  double radius() const { return molten_ ? settings_->moltenParticleDefaultRadius()
                                         : settings_->particleDefaultRadius(); }

  void setDisplacement(const Point& displacement) { displacement_ = displacement; }
  void applyDisplacement() { x_ += displacement_.x; y_ += displacement_.y; }

  // molten implies larger radius; mobility is set separately
  bool isMolten() const { return molten_; }
  void setMolten(bool molten) { molten_ = molten; if (!molten_) melting_timeout_ = -1; }
  int meltingTimeout() const { return melting_timeout_; }
  void setMeltingTimeout(int timeout) { melting_timeout_ = timeout; }

  // mark that this particle is allowed to relax its new state (move)
  bool isMovable() const { return movable_; }
  void setMovable(bool movable) { movable_ = movable; }

  void addSpring(Spring* spring) { springs_.push_back(spring); }
  const std::vector<Spring*>& springs() const { return springs_; }
  void removeString(Spring* spring);

  void setSettings(SimulatorSettings* settings) { settings_ = settings; }

 protected:
  double x_ = 0;
  double y_ = 0;
  Point displacement_ = Point(0.0, 0.0);

  bool molten_ = false;
  int melting_timeout_ = -1;

  bool movable_ = false;

  std::vector<Spring*> springs_ = {};

  const SimulatorSettings* settings_ = nullptr;
};
/*
struct Particle2 {
  QGraphicsEllipseItem* ui = nullptr;

  void render(QGraphicsScene* scene) {
    if (ui) {
      ui->setRect(kRenderCenterX + this->x - radius() * kBlobScale, kRenderCenterY + this->y - radius() * kBlobScale,
                  radius() * 2 * kBlobScale, radius() * 2 * kBlobScale);
    } else {
      ui = scene->addEllipse(kRenderCenterX + this->x - radius() * kBlobScale, kRenderCenterY + this->y - radius() * kBlobScale,
                             radius() * 2 * kBlobScale, radius() * 2 * kBlobScale,
                             QPen(Qt::darkRed), QBrush(Qt::darkRed));
      ui->setZValue(20);
    }
  }
  void setMolten(bool ismolten, int timeout = -1) {
    molten = ismolten;
    molten_timeout = timeout;
    ui->setRect(kRenderCenterX + this->x - radius(), kRenderCenterY + this->y - radius(),
                radius() * 2, radius() * 2);
  }
};
*/


double distance(double x1, double y1, double x2, double y2);
double distance(const Point& p, double x, double y);
double distance(const Point& p1, const Point& p2);
double distance(const Particle* p, double x, double y);
double distance(const Particle* p1, const Particle* p2);
double crossProduct(const Point& p1, const Point& p2, const Point& p3);
bool segmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4);

#endif // PARTICLE_H
