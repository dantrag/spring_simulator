#ifndef SPRINGSIMULATOR_H
#define SPRINGSIMULATOR_H

#include <string>
#include <sstream>
#include <set>
#include <vector>
#include <functional>
#include <algorithm>
#include <chrono>

#include "backend/XMLIO.h"
#include "backend/SimulatorSettings.h"
#include "backend/SpringSimulatorState.h"
#include "backend/Particle.h"
#include "backend/Path.h"
#include "backend/Shape.h"
#include "backend/Actuator.h"

class SpringSimulator : public XMLIO, public TypeReadable {
 public:
  // Initialization
  SpringSimulator();
  SpringSimulator(const SpringSimulator* simulator);
  SpringSimulator(SimulatorSettings* settings);  
  SpringSimulator(std::string settings_file);
  virtual ~SpringSimulator();

  enum class InitializationGrid {
    kHexagonal = 0,
    kSquare,
  };

  bool loadFromXMLNode(pugi::xml_node root) override;
  bool loadFromXML(std::string xml_file) override;
  void restoreState(const SpringSimulatorState* state);

  void initializeCircle(Point center, double radius,
                        InitializationGrid mode = InitializationGrid::kHexagonal);
  void initializeRectangle(Point lefttop, Point rightbottom,
                           InitializationGrid mode = InitializationGrid::kHexagonal);
  void initializeFromPixelArray(const std::vector<std::vector<int>>& rgb_array, double scale,
                                std::function<bool(int)> add_pixel,
                                InitializationGrid mode = InitializationGrid::kHexagonal);
  void initializeFromShape(const Shape& shape, double scale = 1.0,
                           InitializationGrid mode = InitializationGrid::kHexagonal);
  #ifdef QT_CORE_LIB
  void initializeFromImage();
  #endif

  // Publicly accessible variables
  virtual std::string generic_name() const { return "Simulator"; }
  std::string log() const { return log_.str(); }
  SimulatorSettings* settings() const { return settings_; }
  void setSettings(SimulatorSettings* settings);

  const std::vector<Particle*>& particles() const { return particles_; }
  const std::vector<Actuator*>& actuators() const { return actuators_; }

  double scale() const { return scale_; }

  int getTime() const { return time_; }
  void incrementTime();

  void addActuator(Actuator* actuator) { actuators_.push_back(actuator); }
  void removeActuator(Actuator* actuator) { std::remove(actuators_.begin(), actuators_.end(), actuator); }
  void removeAllActuators() { actuators_.clear(); }

  const std::set<Spring*>& recentlyAddedSprings() const { return recently_added_springs_; }
  const std::set<Spring*>& recentlyDeletedSprings() const { return recently_deleted_springs_; }
  void clearRecent() { recently_added_springs_.clear(); recently_deleted_springs_.clear(); }

  Shape fieldContour() const;

  // Operation
  void runLinearPasses();
  void relax(bool extra_long_relaxation = false);
  void clear();
  Path predictMoves(Shape target, Actuator* actuator,
                    double entry_margin, double exit_margin,
                    int samples = 10, int repeats = 1, int angular_resolution = 60);

 protected:
  double defaultInitializationInterval() const;
  void initializeField(InitializationGrid mode, Point center, double width, double height, double interval,
                       std::function<bool(double, double)> valid_point);

  Spring* checkAndAddSpring(Particle* p1, Particle* p2);
  virtual void updateConnectivity() {}

  pugi::xml_document toXML() const override;

  virtual std::vector<std::string> compatible_names() override { return {"simulator", "elastic-simulator"}; }

  std::vector<Particle*> particles_;
  std::vector<Particle*> movable_particles_;
  std::set<Spring*> recently_added_springs_;
  std::set<Spring*> recently_deleted_springs_;
  std::vector<Actuator*> actuators_;
  int time_ = 0;
  double scale_ = 1.0;
  SimulatorSettings* settings_;
  std::stringstream log_;
};

template<class SimulatorClass> SpringSimulator* tryLoadingSimulatorFromFile(std::string filename) {
  auto simulator = new SimulatorClass();
  if (simulator->loadFromXML(filename)) return simulator;
  delete simulator;
  return nullptr;
}

Shape particlesContour(const std::vector<Particle*>& particles_);

double stopwatch(std::chrono::time_point<std::chrono::steady_clock> start);

#endif // SPRINGSIMULATOR_H
