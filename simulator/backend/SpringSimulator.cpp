#include "backend/SpringSimulator.h"

#include <cmath>

#include "backend/Spring.h"

SpringSimulator::SpringSimulator() {
  settings_ = new SimulatorSettings();
}

#ifdef QT_CORE_LIB
SpringSimulator::SpringSimulator(QString settings_file) {
  settings_ = new SimulatorSettings();
  settings_->loadFromFile(settings_file);
}
#endif

Spring* SpringSimulator::checkAndAddSpring(Particle *p1, Particle *p2) {
  if (p1 && p2) {
    for (auto s : p1->springs())
      if (s->otherEnd(p1) == p2) return nullptr;

    return new Spring(p1, p2, settings_->springDefaultLength(), settings_);
  } else return nullptr;
}

void SpringSimulator::initializeField(InitializationGrid mode, Point center, double width, double height,
                                      double interval, std::function<bool(double, double)> valid_point) {
  clear();

  constexpr double hex_scale = std::sqrt(3) / 2;
  double y_interval = interval;
  switch (mode) {
    case InitializationGrid::kHexagonal :
      y_interval = interval * hex_scale; break;
    case InitializationGrid::kSquare :
      break;
  }

  auto size_x = int((width / 2 - interval / 2) / interval);
  auto size_y = int((height / 2 - interval / 2) / y_interval);
  if (size_x <= 0) return;
  if (size_y <= 0) return;

  std::vector<std::vector<Particle*>> particles(2 * size_y + 1, std::vector<Particle*>(2 * size_x + 1));
  for (int i = -size_y; i <= size_y; ++i) {
    for (int j = -size_x; j <= size_x; ++j) {
      double x = center.x + j * interval;
      if (mode == InitializationGrid::kHexagonal && (i & 1)) x -= interval / 2;
      double y = center.y + i * y_interval;
      if (valid_point(x, y))
        particles[i + size_y][j + size_x] = new Particle(x, y, settings_);
    }
  }

  for (int i = 0; i <= 2 * size_y; ++i) {
    for (int j = 0; j <= 2 * size_x; ++j) {
      if (particles[i][j]) {
        if (j > 0) checkAndAddSpring(particles[i][j], particles[i][j - 1]);
        if (i > 0) checkAndAddSpring(particles[i][j], particles[i - 1][j]);
        if (mode == InitializationGrid::kHexagonal && (i > 0)) {
          if ((i - size_y) & 1) {
            if (j > 0) checkAndAddSpring(particles[i][j], particles[i - 1][j - 1]);
          } else {
            if (j < 2 * size_x) checkAndAddSpring(particles[i][j], particles[i - 1][j + 1]);
          }
        }
        particles_.push_back(particles[i][j]);
      }
    }
  }
}

double SpringSimulator::defaultInitializationInterval() const {
  return settings_->particleDefaultRadius() * 2 + settings_->springDefaultLength();
}

void SpringSimulator::initializeCircle(Point center, double radius, InitializationGrid mode) {
  auto interval = defaultInitializationInterval();
  initializeField(mode, center, radius * 2, radius * 2, interval, [&](double x, double y) {
    return distance(center, x, y) + interval / 2 <= radius + 1e-5;
  });
}

void SpringSimulator::initializeRectangle(Point lefttop, Point rightbottom, InitializationGrid mode) {
  auto left = std::min(lefttop.x, rightbottom.x);
  auto right = std::max(lefttop.x, rightbottom.x);
  auto top = std::min(lefttop.y, rightbottom.y);
  auto bottom = std::max(lefttop.y, rightbottom.y);
  auto interval = defaultInitializationInterval();
  initializeField(mode, Point((left + right) / 2, (top + bottom) / 2), right - left, bottom - top,
                  interval, [&](double x, double y) {
    return (x - interval / 2 >= left - 1e-5) && (x + interval / 2 <= right + 1e-5) &&
           (y - interval / 2 >= top - 1e-5) && (y + interval / 2 <= bottom + 1e-5);
  });
}

void SpringSimulator::initializeFromPixelArray(const std::vector<std::vector<int>>& rgb_array, double scale,
                                               std::function<bool(int)> add_pixel, InitializationGrid mode) {
  int height = static_cast<int>(rgb_array.size());
  int width = 0;
  std::vector<std::vector<bool>> include;
  for (const auto& line : rgb_array) {
    include.push_back(std::vector<bool>());
    for (auto pixel : line) include.rbegin()->push_back(add_pixel(pixel));
    width = std::max(width, static_cast<int>(line.size()));
  }
  auto interval = defaultInitializationInterval();
  initializeField(mode, Point(width * scale / 2, height * scale / 2), width * scale, height * scale, interval,
                  [&](double x, double y) {
    auto pixel_y = static_cast<int>(round(y / scale));
    if (pixel_y >= height) return false;
    auto pixel_x = static_cast<int>(round(x / scale));
    if (pixel_x >= static_cast<int>(include[pixel_y].size())) return false;
    return static_cast<bool>(include[pixel_y][pixel_x]);
  });
}

void particleDFS(Particle* current, int current_depth, int minimum_depth, int maximum_depth,
                 std::set<Particle*>& visited,
                 std::set<Particle*>& neighbourhood) {
  visited.insert(current);
  if (current_depth > maximum_depth) return;
  if (current_depth >= minimum_depth) neighbourhood.insert(current);
  for (auto s : current->springs()) {
    if (!visited.count(s->otherEnd(current)))
      particleDFS(s->otherEnd(current), current_depth + 1, minimum_depth, maximum_depth, visited, neighbourhood);
  }
}

void SpringSimulator::relaxHeat() {
  double max_displacement = 0;
  int iteration_count = 0;
  std::vector<Particle*> movable_particles;
  for (auto p : particles_)
    if (p->isMovable()) movable_particles.push_back(p);

  do {
    max_displacement = 0;
    for (auto p : movable_particles) {
      double x_displacement = 0.0;
      double y_displacement = 0.0;
      for (auto s : p->springs()) {
        double delta_x = s->otherEnd(p)->x() - p->x();
        double delta_y = s->otherEnd(p)->y() - p->y();
        if (s->force() > 0) delta_x = -delta_x, delta_y = -delta_y;
        double delta_module = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        // make sure not to divide by [close to] zero
        if (delta_module < 1e-5) continue;
        delta_x /= delta_module;
        delta_y /= delta_module;
        delta_x *= std::fabs(s->force());
        delta_y *= std::fabs(s->force());
        x_displacement += delta_x;
        y_displacement += delta_y;
      }
      max_displacement = std::max(max_displacement, std::sqrt(x_displacement * x_displacement + y_displacement * y_displacement));
      p->setDisplacement(Point(x_displacement, y_displacement));
    }

    for (auto p : movable_particles) {
      p->applyDisplacement();
      for (auto s : p->springs()) s->updateForce();
    }

    // delete too long springs
    if (iteration_count % 50 == 0)
    for (auto p : movable_particles) {
      for (auto s : p->springs()) {
        if (s == nullptr) continue;
        if (s->actualLength() > s->length() * settings_->springDisconnectionThreshold()) {
          p->removeString(s);
          s->otherEnd(p)->removeString(s);
          if (recently_added_springs_.count(s))
            recently_added_springs_.erase(s);
          else
            recently_deleted_springs_.insert(s);
          delete s;
        }
      }
    }

    // create new springs between close particles
    if (iteration_count % 50 == 0)
    for (auto p : movable_particles) {
      std::set<Particle*> neighbours, visited;
      particleDFS(p, 0, 2, 4, visited, neighbours);
      neighbours.erase(p);
      for (auto neighbour : neighbours) {
        if (distance(p, neighbour) - p->radius() - neighbour->radius() <
            settings_->springDefaultLength() * settings_->springConnectionThreshold()) {
          auto spring = checkAndAddSpring(p, neighbour);
          if (spring) recently_added_springs_.insert(spring);
        }
      }
    }

    iteration_count++;
  } while (max_displacement > settings_->relaxationConvergenceLimit() &&
           iteration_count < settings_->relaxationIterationLimit());

  for (auto p : movable_particles) {
    if (!p->isMolten()) p->setMovable(false);
  }
}

void SpringSimulator::runLinearPass(const Point& start, const Point& finish) {
  double path = distance(start, finish);
  int ticks = path / settings_->heaterSpeed() + 1;
  for (int i = 0; i < ticks; ++i) {
    double x = start.x + (finish.x - start.x) / path * settings_->heaterSpeed() * i;
    double y = start.y + (finish.y - start.y) / path * settings_->heaterSpeed() * i;
    // cool timed out particles
    for (auto p : particles_) {
      if (p->meltingTimeout() > 0 && p->meltingTimeout() <= time_) {
        p->setMolten(false);
        p->setMovable(true);
      }
    }
    // heat around x, y
    for (auto p : particles_) {
      if (distance(p, x, y) <= settings_->heaterSize()) {
        p->setMolten(true);
        p->setMeltingTimeout(time_ + settings_->moltenParticleCooldownTime());
        p->setMovable(true);
      }
    }

    for (auto p : particles_) {
      for (auto s : p->springs()) s->updateForce();
    }
    relaxHeat();

    for (auto p : particles_) {
      if (!p->isMolten()) p->setMovable(false);
    }

    incrementTime();
  }
}


void SpringSimulator::runLinearPasses(const std::vector<Point>& points) {
  for (size_t i = 0; i < points.size() - 1; ++i)
    runLinearPass(points[i], points[i + 1]);

  // after-pass cooldown
  for (auto p : particles_) {
    if (p->isMolten()) {
      p->setMolten(false);
      p->setMovable(true);
    }
  }
  for (auto p : particles_) {
    for (auto s : p->springs()) s->updateForce();
  }
  relaxHeat();
}

void SpringSimulator::clear() {
  time_ = 0;
  for (auto p : particles_) {
    delete p;
  }
  particles_.clear();
}
