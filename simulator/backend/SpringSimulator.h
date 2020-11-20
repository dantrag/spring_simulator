#ifndef SPRINGSIMULATOR_H
#define SPRINGSIMULATOR_H

#include <string>
#include <set>
#include <vector>

#include "backend/SimulatorSettings.h"
#include "backend/Particle.h"

class SpringSimulator {
 public:
  SpringSimulator();
  #ifdef QT_CORE_LIB
  SpringSimulator(QString settings_file);
  #endif

  SimulatorSettings* settings() { return settings_; }

  enum class InitializationGrid {
    kHexagonal = 0,
    kSquare,
  };

  void initializeCircle(Point center, double radius, InitializationGrid mode = InitializationGrid::kHexagonal);
  void initializeFromPixelArray();
  #ifdef QT_CORE_LIB
  void initializeFromImage();
  #endif
  void clear();

  int getTime() const { return time_; }
  void incrementTime() { time_++; }

  // heating/cooling operations
  void runLinearPasses(const std::vector<Point>& points);
  void relaxHeat();

  const std::set<Spring*>& recentlyAddedSprings() const { return recently_added_springs_; }
  const std::set<Spring*>& recentlyDeletedSprings() const { return recently_deleted_springs_; }
  void clearRecent() { recently_added_springs_.clear(); recently_deleted_springs_.clear(); }

  std::vector<Particle*> particles_;

 protected:
  void runLinearPass(const Point& start, const Point& finish);

  Spring* checkAndAddSpring(Particle* p1, Particle* p2);

  std::set<Spring*> recently_added_springs_;
  std::set<Spring*> recently_deleted_springs_;
  int time_ = 0;
  SimulatorSettings* settings_;
};

#endif // SPRINGSIMULATOR_H
