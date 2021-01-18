#include "backend/SpringSimulator.h"

#include <cmath>
#include <unordered_map>
#include <queue>

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

void particleBFS(Particle* start, int minimum_depth, int maximum_depth,
                 std::set<Particle*>& neighbourhood) {
  std::queue<Particle*> bfs_queue = {};
  std::unordered_map<Particle*, int> depth = {};
  bfs_queue.push(start);
  depth[start] = 0;
  while (!bfs_queue.empty()) {
    auto p = bfs_queue.front();
    bfs_queue.pop();
    if (minimum_depth <= depth[p] && depth[p] <= maximum_depth) {
      neighbourhood.insert(p);
    }
    if (depth[p] > maximum_depth) break;
    for (auto s : p->springs()) {
      auto next = s->otherEnd(p);
      if (!depth.count(next)) {
        bfs_queue.push(next);
        depth[next] = depth[p] + 1;
      }
    }
  }
}

// check if removal of the spring will create a long cycle (potential void)
bool checkSpringRemovalAllowance(Spring* s, int min_cycle_length, int max_cycle_length,
                                 std::vector<Particle*>& cycle, bool& fixable) {
  std::set<Spring*> forbidden_springs = {s};
  std::queue<Particle*> bfs_queue;
  bfs_queue.push(s->particle1());
  std::map<Particle*, Spring*> link_to_previous = {};
  link_to_previous[s->particle1()] = nullptr;
  std::map<Particle*, int> depth = {};
  depth[s->particle1()] = 0;

  while (!bfs_queue.empty()) {
    auto current = bfs_queue.front();
    bfs_queue.pop();
    for (auto adjacent_spring : current->springs()) {
      if (!forbidden_springs.count(adjacent_spring)) {
        auto next = adjacent_spring->otherEnd(current);
        if (!link_to_previous.count(next)) {
          bfs_queue.push(next);
          link_to_previous[next] = adjacent_spring;
          depth[next] = depth[current] + 1;
          if (next == s->particle2() || depth[next] > max_cycle_length / 2) {
            while (!bfs_queue.empty()) bfs_queue.pop();
            break;
          }
        }
      }
    }
  }

  if (!depth.count(s->particle2())) {
    // particles became disjointed or too long cycle forms, so removal of the spring is not possible
    fixable = false;
    return false;
  }

  auto current = s->particle2();
  while (current != s->particle1()) {
    cycle.push_back(current);
    forbidden_springs.insert(link_to_previous[current]);
    current = link_to_previous[current]->otherEnd(current);
  }
  std::reverse(cycle.begin(), cycle.end());
  int half_cycle_size = depth[s->particle2()];

  bfs_queue.push(s->particle1());
  link_to_previous.clear();
  depth.clear();
  depth[s->particle1()] = 0;
  link_to_previous[s->particle1()] = nullptr;

  while (!bfs_queue.empty()) {
    auto current = bfs_queue.front();
    bfs_queue.pop();
    for (auto adjacent_spring : current->springs()) {
      if (!forbidden_springs.count(adjacent_spring)) {
        auto next = adjacent_spring->otherEnd(current);
        if (!link_to_previous.count(next)) {
          bfs_queue.push(next);
          link_to_previous[next] = adjacent_spring;
          depth[next] = depth[current] + 1;
          if (next == s->particle2() || depth[next] + half_cycle_size > max_cycle_length) {
            while (!bfs_queue.empty()) bfs_queue.pop();
            break;
          }
        }
      }
    }
  }

  if (!depth.count(s->particle2())) {
    // only one path between the ends of the spring, no cycle formed
    // or (very unlikely) the second path is too long ("creating a crack")
    fixable = false;
    if (half_cycle_size <= max_cycle_length / 2 && s->actualLength() / s->length() > 1.6)
      return true;
    else
      return false;
  }

  // otherwise we have found a long cycle! but is it minimal?

  current = s->particle2();
  while (current != s->particle1()) {
    forbidden_springs.insert(link_to_previous[current]);
    current = link_to_previous[current]->otherEnd(current);
    cycle.push_back(current);
  }

  fixable = true;

  // try to shrink the found cycle - there can be at most one edge between two "sides" of the cycle,
  // according to our design where we add 1 edge that might intersect s

  for (int i = 0; i < half_cycle_size - 1; ++i) {
    for (int j = half_cycle_size; j < static_cast<int>(cycle.size()) - 1; ++j) {
      for (auto s : cycle[i]->springs()) {
        if (s->otherEnd(cycle[i]) == cycle[j]) {
          // we can split the cycle
          int sub_cycle_size1 = j - i + 1;
          int sub_cycle_size2 = static_cast<int>(cycle.size()) - sub_cycle_size1 + 2;
          if (sub_cycle_size1 < min_cycle_length && sub_cycle_size2 < min_cycle_length) {
            return true;
          }
        }
      }
    }
  }

  return (depth[s->particle2()] + half_cycle_size < min_cycle_length);
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
    }

    // delete too long springs
    if (iteration_count % 50 == 0) {
      auto spring_comparator = [](Spring* s1, Spring* s2) { return s1->actualLength() / s1->length() >
                                                                   s2->actualLength() / s2->length(); };
      std::set<Spring*, decltype(spring_comparator)> sorted_springs(spring_comparator);
      for (auto p : movable_particles) {
        for (auto s : p->springs()) {
          if (s != nullptr && s->actualLength() / s->length() > settings_->springDisconnectionThreshold())
            sorted_springs.insert(s);
        }
      }

      const int min_cycle_length = 4;
      const int max_cycle_length = 4;
      for (auto s : sorted_springs) {
        // check if removal of the spring will create no leaves/isolated nodes
        if (s->particle1()->springs().size() <= 2) continue;
        if (s->particle2()->springs().size() <= 2) continue;

        if (s->actualLength() / s->length() > 1.4) {
          s->updateForce();
        }

        // check if removal of the spring will create no long cycles (potential voids)
        std::vector<Particle*> cycle = {}, temp_cycle = {};
        bool can_fix = false;
        bool can_remove = checkSpringRemovalAllowance(s, min_cycle_length, max_cycle_length, cycle, can_fix);
        if (!can_remove && can_fix) {
          // if a long cycle is created, can it be fixed with adding a shorter spring?
          Spring* new_spring = nullptr;
          for (auto p1 : cycle) {
            for (auto p2 : cycle) {
              if (p1 >= p2) continue;
              if (p1 == s->particle1() && p2 == s->particle2()) continue;
              if (p1 == s->particle2() && p2 == s->particle1()) continue;
              new_spring = checkAndAddSpring(p1, p2);
              if (new_spring) {
                double nal = new_spring->actualLength();
                double nl = new_spring->length();
                double sal = s->actualLength();
                double sl = s->length();
                if (new_spring->actualLength() / new_spring->length() < s->actualLength() / s->length() &&
                    checkSpringRemovalAllowance(s, min_cycle_length, max_cycle_length, temp_cycle, can_fix)) {
                  // can eliminate the formed long cycle with a shorter spring, keep it
                  recently_added_springs_.insert(new_spring);
                  break;
                } else {
                  p1->removeString(new_spring);
                  p2->removeString(new_spring);
                  delete new_spring;
                  new_spring = nullptr;
                }
                temp_cycle.clear();
              }
            }
            if (new_spring != nullptr) break;
          }
          if (new_spring != nullptr) can_remove = true;
        }

        if (can_remove) {
          s->particle1()->removeString(s);
          s->particle2()->removeString(s);
          if (recently_added_springs_.count(s))
            recently_added_springs_.erase(s);
          else
            recently_deleted_springs_.insert(s);
          delete s;
        }
      }
    }

    // create new springs between close particles - but make sure there are no overlaps
    if (iteration_count % 50 == 0)
    for (auto p : movable_particles) {
      std::set<Particle*> new_partners = {};
      particleBFS(p, 2, 4, new_partners);
      std::set<Particle*> neighbourhood = new_partners;
      particleBFS(p, 1, 1, neighbourhood);
      for (auto partner : new_partners) {
        if (distance(p, partner) - p->radius() - partner->radius() <
            settings_->springDefaultLength() * settings_->springConnectionThreshold()) {
          // check if new spring will intersect with some other
          bool intersect = false;
          for (auto other : neighbourhood) {
            if (other != partner) {
              for (auto spring : other->springs()) {
                if (spring->otherEnd(other) != partner &&
                    spring->otherEnd(other) != p) {
                  // check intersection
                  if (segmentsIntersect(p->point(), partner->point(),
                                        other->point(), spring->otherEnd(other)->point())) {
                    intersect = true;
                    break;
                  }
                }
              }
            }
            if (intersect) break;
          }
          if (!intersect) {
            auto spring = checkAndAddSpring(p, partner);
            if (spring) recently_added_springs_.insert(spring);
          }
        }
      }
    }

    for (auto p : movable_particles) {
      for (auto s : p->springs()) s->updateForce();
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

template<class VertexType>
bool findCycle(VertexType* current, VertexType* parent,
               std::unordered_map<VertexType*, std::vector<VertexType*>>& graph,
               std::set<VertexType*>& visited,
               std::vector<VertexType*>& current_path, std::vector<VertexType*>& cycle) {
  visited.insert(current);
  current_path.push_back(current);
  for (auto next : graph[current]) {
    if (!visited.count(next)) {
      if (findCycle(next, current, graph, visited, current_path, cycle)) return true;
    } else {
      if (next != parent) {
        for (auto particle = current_path.rbegin();
             particle != current_path.rend() && *particle != next;
             particle++)
          cycle.push_back(*particle);
        cycle.push_back(next);
        return true;
      }
    }
  }
  current_path.pop_back();
  return false;
}

const std::vector<Point> SpringSimulator::fieldContour() const {
  // threshold of what is considered to be a cycle lying inside the material
  const int max_inner_cycle_size = 4;

  // pair of boolean - whether left side/right side of the spring is inside
  std::unordered_map<Spring*, std::pair<bool, bool>> spring_sides_inside = {};

  for (auto p : particles_) {
    for (auto s : p->springs()) if (p < s->otherEnd(p)) {
      // make sure to check every spring only once
      auto other = s->otherEnd(p);
      std::queue<Particle*> bfs_queue = {};
      std::set<Particle*> visited = {p, other};
      std::vector<Particle*> cycle_ends = {};
      std::unordered_map<Particle*, std::pair<int, Particle*>> depth = {}; // depth and previous vertex
      depth[other] = std::make_pair(-1, nullptr);
      depth[p] = std::make_pair(0, other);
      bfs_queue.push(p);
      while (!bfs_queue.empty()) {
        auto v = bfs_queue.front();
        bfs_queue.pop();
        for (auto s : v->springs()) {
          auto next = s->otherEnd(v);
          if (!visited.count(next) && depth[v].first < max_inner_cycle_size - 2) {
            bfs_queue.push(next);
            depth[next] = std::make_pair(depth[v].first + 1, v);
            visited.insert(next);
          }
          if (next == other && next != depth[v].second)
            cycle_ends.push_back(v);
        }
      }

      for (auto v : cycle_ends) {
        // register cycle as an inside cycle
        // assume it is convex => geometric centre lies inside
        double centre_x = 0.0, centre_y = 0.0;
        auto current = v;
        int size = 0;
        while (current != nullptr) {
          centre_x += current->point().x;
          centre_y += current->point().y;
          current = depth[current].second;
          size++;
        }
        // TODO: check that (depth[v].first + 2) == size !
        centre_x /= size;
        centre_y /= size;
        auto centre = Point(centre_x, centre_y);

        std::vector<Spring*> cycle_springs = {};
        current = v;
        while (current != nullptr) {
          auto parent = depth[current].second;
          for (auto spring : current->springs()) {
            if (spring->otherEnd(current) == parent) {
              cycle_springs.push_back(spring);
              break;
            }
          }
          current = parent;
        }
        for (auto spring : v->springs()) {
          if (spring->otherEnd(v) == other) {
            cycle_springs.push_back(spring);
            break;
          }
        }

        for (auto spring : cycle_springs) {
          if (!spring_sides_inside.count(spring) ||
              !(spring_sides_inside[spring].first & spring_sides_inside[spring].second)) {
            bool is_left = crossProduct(spring->particle1()->point(),
                                        spring->particle2()->point(),
                                        centre) >= 0.0;
            if (is_left)
              spring_sides_inside[spring].first = true;
            else
              spring_sides_inside[spring].second = true;
          }
        }
      }
    }
  }

  std::unordered_map<Particle*, std::vector<Particle*>> graph = {};
  for (auto s : spring_sides_inside) {
    if (!(s.second.first & s.second.second)) {
      graph[s.first->particle1()].push_back(s.first->particle2());
      graph[s.first->particle2()].push_back(s.first->particle1());
    }
  }

  std::vector<Particle*> largest_cycle = {};
  for (const auto& v : graph) {
    std::set<Particle*> visited = {};
    std::vector<Particle*> path = {}, cycle = {};
    if (findCycle<Particle>(v.first, nullptr, graph, visited, path, cycle)) {
      if (cycle.size() > largest_cycle.size()) {
        largest_cycle = cycle;
      }
    }
  }

  std::vector<Point> contour = {};
  for (auto p : largest_cycle) contour.push_back(p->point());
  return contour;
}

void SpringSimulator::clear() {
  time_ = 0;
  for (auto p : particles_) {
    delete p;
  }
  particles_.clear();
}
