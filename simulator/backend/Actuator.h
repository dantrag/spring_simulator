#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <vector>

#include "backend/XMLIO.h"
#include "backend/Particle.h"
#include "backend/Path.h"
#include "backend/Shape.h"

typedef std::function<bool(const Particle*)> ActuatorCaptureFunction;

struct ActuatorState;

class Actuator : public XMLIO, public TypeReadable {
 public:
  Actuator();
  Actuator(std::string xml_file);
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

  const Shape& shape() const { return shape_; }
  void setShape(Shape shape);

  double orientation() const { return orientation_; }
  void setOrientation(double orientation) { orientation_ = orientation; }

  virtual void preprocessParticle(Particle* particle) = 0;
  virtual void processParticle(Particle* particle) = 0;
  virtual void postprocessParticle(Particle* particle) = 0;

  virtual void resetParticles(std::vector<Particle*>& particles) = 0;

  // means that the inward motion allows material to self-penetrate;
  // sometimes it allows to emulate folding
  virtual bool isSpringCrossingApplicable() const { return false; };
  bool isSpringCrossingAllowed() const { return spring_crossing_allowed_; }
  void setSpringCrossing(bool allowed) { spring_crossing_allowed_ = allowed; }

  // means that the particle(s) captured in the first move is/are going to be the one(s)
  // being pushed until resetParticles() is called
  virtual bool isFirmGripApplicable() const { return false; }
  bool isFirmGrip() const { return firm_grip_; }
  void setFirmGrip(bool firm) { firm_grip_ = firm; }

  // means that the actuator releases grip (if applicable) after the run
  // and lets the particles to reach equilibrium under no external forces
  virtual bool isFinalReleaseApplicable() const { return false; }
  bool isFinalRelease() const { return final_release_; }
  void setFinalRelease(bool release) { final_release_ = release; }

  virtual std::string generic_name() const { return "Actuator"; }
  std::string name() const { return name_; }
  void setName(std::string new_name) { name_ = new_name; }

  virtual ActuatorState saveState() const;
  virtual void loadState(const ActuatorState& state);

  //static std::string loadGenericNameFromXML(std::string xml_file);

  bool loadFromXMLNode(pugi::xml_node root) override;
  bool loadFromXML(std::string xml_file) override;
  pugi::xml_document toXML() const override;

 protected:
  std::string name_ = "";

  bool on_ = false;
  double speed_ = 1.0;
  int time_ = 0;
  bool spring_crossing_allowed_ = false;
  bool firm_grip_ = true;
  bool final_release_ = false;

  Point position_ = Point(0.0, 0.0);
  Point last_position_ = Point(0.0, 0.0);
  Path path_;
  double path_advancement_ = 0.0;

  double orientation_ = 0.0;
  Shape shape_ = Shape({Point(0.0, 0.0)});
  ActuatorCaptureFunction capture_particle_check_;
};

template<class ActuatorClass> Actuator* tryLoadingActuatorFromFile(std::string filename) {
  auto actuator = new ActuatorClass();
  if (actuator->loadFromXML(filename)) return actuator;
  delete actuator;
  return nullptr;
}

template<class ActuatorClass> Actuator* tryLoadingActuatorFromXMLNode(pugi::xml_node root) {
  auto actuator = new ActuatorClass();
  if (actuator->loadFromXMLNode(root)) return actuator;
  delete actuator;
  return nullptr;
}

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
