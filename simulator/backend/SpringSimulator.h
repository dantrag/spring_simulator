#ifndef SPRINGSIMULATOR_H
#define SPRINGSIMULATOR_H

#include <string>
#include <sstream>
#include <set>
#include <vector>
#include <functional>
#include <chrono>

#include "backend/SimulatorSettings.h"
#include "backend/Particle.h"
#include "backend/Path.h"
#include "backend/Shape.h"
#include "backend/Actuator.h"

class SpringSimulator {
 public:
  // Initialization
  SpringSimulator();
  SpringSimulator(const SpringSimulator* simulator);
  SpringSimulator(SimulatorSettings* settings);
  #ifdef QT_CORE_LIB
  SpringSimulator(QString settings_file);
  #endif
  virtual ~SpringSimulator();

  enum class InitializationGrid {
    kHexagonal = 0,
    kSquare,
  };

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

  // Publicly accesible variables
  SimulatorSettings* settings() const { return settings_; }
  std::string log() const { return log_.str(); }

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
  void relax();
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

Shape particlesContour(const std::vector<Particle*>& particles_);

double stopwatch(std::chrono::time_point<std::chrono::steady_clock> start);

#endif // SPRINGSIMULATOR_H
