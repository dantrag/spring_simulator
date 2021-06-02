#ifndef SPRINGSIMULATOR_H
#define SPRINGSIMULATOR_H

#include <string>
#include <sstream>
#include <set>
#include <vector>
#include <functional>

#include "backend/SimulatorSettings.h"
#include "backend/Particle.h"
#include "backend/Shape.h"

class SpringSimulator {
 public:
  SpringSimulator();
  SpringSimulator(const SpringSimulator* simulator);
  #ifdef QT_CORE_LIB
  SpringSimulator(QString settings_file);
  #endif

  SimulatorSettings* settings() const { return settings_; }

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
  void clear();
  std::string log() const { return log_.str(); };

  int getTime() const { return time_; }
  void incrementTime() { time_++; }
  double scale() const { return scale_; }

  // heating/cooling operations
  void runLinearPasses(const std::vector<Point>& points);
  void relaxHeat();

  const std::set<Spring*>& recentlyAddedSprings() const { return recently_added_springs_; }
  const std::set<Spring*>& recentlyDeletedSprings() const { return recently_deleted_springs_; }
  void clearRecent() { recently_added_springs_.clear(); recently_deleted_springs_.clear(); }

  Shape fieldContour() const;

  const std::vector<Particle*>& particles() const { return particles_; }

 protected:
  double defaultInitializationInterval() const;
  void initializeField(InitializationGrid mode, Point center, double width, double height, double interval,
                       std::function<bool(double, double)> valid_point);
  void runLinearPass(const Point& start, const Point& finish);

  Spring* checkAndAddSpring(Particle* p1, Particle* p2);

  std::vector<Particle*> particles_;
  std::set<Spring*> recently_added_springs_;
  std::set<Spring*> recently_deleted_springs_;
  int time_ = 0;
  double scale_ = 1.0;
  SimulatorSettings* settings_;
  std::stringstream log_;
};

Shape samplingContour(const Shape& contour,
                      const std::vector<std::pair<double, double>>& vectors,
                      double margin);

std::vector<Point> predictMoves(const SpringSimulator* simulator, Shape target,
                                double entry_margin, double exit_margin,
                                int samples = 10, int repeats = 1, int angular_resolution = 60);

#endif // SPRINGSIMULATOR_H
