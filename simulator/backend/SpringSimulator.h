#ifndef SPRINGSIMULATOR_H
#define SPRINGSIMULATOR_H

#include <string>
#include <sstream>
#include <set>
#include <vector>
#include <functional>

#include "backend/SimulatorSettings.h"
#include "backend/Particle.h"

class SpringSimulator
{
public:
  SpringSimulator();
#ifdef QT_CORE_LIB
  SpringSimulator(QString settings_file);
#endif

  SimulatorSettings *settings()
  {
    return settings_;
  }

  enum class InitializationGrid
  {
    kHexagonal = 0,
    kSquare,
  };

  void initializeCircle(Point center, double radius,
                        InitializationGrid mode = InitializationGrid::kHexagonal);
  void initializeRectangle(Point lefttop, Point rightbottom,
                           InitializationGrid mode = InitializationGrid::kHexagonal);
  void initializeFromPixelArray(const std::vector<std::vector<int>> &rgb_array, double scale,
                                std::function<bool(int)> add_pixel,
                                InitializationGrid mode = InitializationGrid::kHexagonal);
  void initializeFromFileName(const std::string fileName);

#ifdef QT_CORE_LIB
  void initializeFromImage();
#endif
  void clear();
  std::string log() const { return log_.str(); };

  int getTime() const { return time_; }
  void incrementTime() { time_++; }

  // heating/cooling operations
  void runLinearPasses(const std::vector<Point> &points);
  void runLinearPass(const Point &start, const Point &finish);

  void relaxHeat();

  const std::set<Spring *> &recentlyAddedSprings() const { return recently_added_springs_; }
  const std::set<Spring *> &recentlyDeletedSprings() const { return recently_deleted_springs_; }
  void clearRecent()
  {
    recently_added_springs_.clear();
    recently_deleted_springs_.clear();
  }

  const std::vector<Point> fieldContour() const;

  const std::vector<Particle *> &particles() const { return particles_; }

  // Stuff i added
  // std::vector<std::tuple<std::tuple<double, double>, std::tuple<double, double>>> getGraph(std::vector<Particle*> particles);

  void getGraph(std::vector<Particle *> particles, std::string fileName);

  void DSFUtil(Particle *particle, std::string filename, std::set<Particle *> &visited);

  void saveParticleState(Point point1, Point point2, std::string filename);
  void saveAction(Point point1, Point point2, Point midpoint, std::string filename);
  // void saveParticleState(std::vector<std::tuple<std::tuple<double, double>, std::tuple<double, double>>> tupleList, string fileName);
  //void saveImageState(string fileName);

protected:
  double defaultInitializationInterval() const;
  void initializeField(InitializationGrid mode, Point center, double width, double height, double interval,
                       std::function<bool(double, double)> valid_point);

  Spring *checkAndAddSpring(Particle *p1, Particle *p2);

  std::vector<Particle *> particles_;
  std::set<Spring *> recently_added_springs_;
  std::set<Spring *> recently_deleted_springs_;
  int time_ = 0;
  SimulatorSettings *settings_;
  std::stringstream log_;
};

#endif // SPRINGSIMULATOR_H
