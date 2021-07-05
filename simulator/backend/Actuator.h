#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <vector>

#include "backend/Particle.h"
#include "backend/Path.h"
#include "backend/Shape.h"

typedef std::function<bool(const Particle*)> ActuatorCaptureFunction;

struct ActuatorState;

class Actuator {
 public:
  Actuator();
  virtual ~Actuator() {}

  bool enabled() const { return on_; }
  virtual void enable() { on_ = true; }
  virtual void disable() { on_ = false; }
  void setEnabled(bool enabled);

  double speed() const { return speed_; }
  void setSpeed(double speed) { speed_ = speed; }
  int time() const { return time_; }
  void setTime(int time) { time_ = time; }

  Point position() const { return position_; }
  double pathAdvancement() const { return path_advancement_; }
  void setPathAdvancement(double cumulative_length);

  // overrides the capture based on Shape! use carefully
  void setCaptureFunction(ActuatorCaptureFunction function) { capture_particle_check_ = std::move(function); }

  const Path& path() const { return path_; }
  void setPath(Path path) { path_ = path; setPathAdvancement(0.0); }

  const Shape& shape() { return shape_; }
  void setShape(Shape shape);

  double orientation() { return orientation_; }
  void setOrientation(double orientation) { orientation_ = orientation; }

  virtual void preprocessParticle(Particle* particle) = 0;
  virtual void processParticle(Particle* particle) = 0;
  virtual void postprocessParticle(Particle* particle) = 0;

  virtual void resetParticles(std::vector<Particle*>& particles) = 0;

  virtual std::string generic_name() const { return "Actuator"; }
  std::string name() const { return name_; }
  void setName(std::string new_name) { name_ = new_name; }

  virtual ActuatorState saveState() const;
  virtual void loadState(const ActuatorState& state);

 protected:
  std::string name_ = "";

  bool on_ = false;
  double speed_ = 1.0;
  int time_ = 0;

  Point position_ = Point(0.0, 0.0);
  Path path_;
  double path_advancement_ = 0.0;

  double orientation_ = 0.0;
  Shape shape_ = Shape({Point(0.0, 0.0)});
  ActuatorCaptureFunction capture_particle_check_;
};

// contains parameters that can be changed during the simulator operation,
// so that they can be reset after trying something with it
struct ActuatorState {
  ActuatorState(const Actuator* actuator);
  bool enabled;
  double path_advancement;
  int time;
  Path path;
};

#endif // ACTUATOR_H
