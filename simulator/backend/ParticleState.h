#ifndef PARTICLESTATE_H
#define PARTICLESTATE_H

class Particle;

class ParticleState {
 public:
  ParticleState(const Particle* particle);

  double x() const { return x_; }
  double y() const { return y_; }
  double radius() const { return radius_; }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double radius_ = 0.0;
};

#endif // PARTICLESTATE_H
