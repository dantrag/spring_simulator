#include "backend/SpringSimulator.h"

#include <cmath>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <queue>

//temp
#include <iostream>
#include <sstream>

#include "backend/Spring.h"
#include "backend/SpringSimulatorState.h"
#include "backend/Heater.h"
#include "backend/Pusher.h"

SpringSimulator::SpringSimulator() {
  settings_ = new SimulatorSettings();
}

SpringSimulator::SpringSimulator(const SpringSimulator* simulator)
    : SpringSimulator() {
  std::unordered_map<Particle*, Particle*> particle_mapping;
  for (const auto& particle : simulator->particles()) {
    auto new_particle = new Particle(particle);
    particle_mapping[particle] = new_particle;
    particles_.push_back(new_particle);
  }

  std::unordered_map<Spring*, Spring*> spring_mapping;
  for (const auto& particle : simulator->particles()) {
    for (const auto& spring : particle->springs()) {
      if (!spring_mapping.count(spring)) {
        spring_mapping[spring] = new Spring(particle_mapping[particle],
                                            particle_mapping[spring->otherEnd(particle)],
                                            spring->length(),
                                            spring->forceConstant());
      }
    }
  }

  time_ = simulator->getTime();
  log_ << simulator->log();
  settings_ = simulator->settings();
}

SpringSimulator::SpringSimulator(std::string settings_file) {
  settings_ = new SimulatorSettings(settings_file);
}

SpringSimulator::SpringSimulator(SimulatorSettings* settings)
    : settings_(settings) {}

void SpringSimulator::setSettings(SimulatorSettings* settings) {
  settings_ = settings;
  for (auto particle : particles_) {
    particle->setSettings(settings_);
  }
}

void SpringSimulator::restoreState(const SpringSimulatorState* state) {
  clear();

  std::unordered_map<const ParticleState*, Particle*> states;
  for (const auto particle_state : state->particles()) {
    auto particle = new Particle(particle_state->x(), particle_state->y(), settings_);
    if (std::abs(particle_state->radius() - settings_->moltenParticleDefaultRadius()) < 1e-5)
      particle->setMolten(true);
    states[particle_state] = particle;
    particles_.push_back(particle);
  }

  for (const auto spring_state : state->springs()) {
    new Spring(states[spring_state->particle1()],
               states[spring_state->particle2()],
               spring_state->equilibriumLength(),
               spring_state->forceConstant());
  }
}

bool SpringSimulator::loadFromXMLNode(pugi::xml_node root) {
  std::string type(root.attribute("type").as_string());
  if (!checkType(type)) return false;

  time_ = root.attribute("time").as_int();
  scale_ = root.attribute("scale").as_double(1.0);

  auto state_node = root.child("state");
  if (state_node.empty()) return false;

  auto state = new SpringSimulatorState();
  if (!state->loadFromXMLNode(state_node)) return false;

  restoreState(state);

  auto actuators_node = root.child("actuators");
  for (auto actuator_node = actuators_node.child("actuator");
       actuator_node;
       actuator_node = actuator_node.next_sibling("actuator")) {
    Actuator* actuator = nullptr;
    if (actuator == nullptr) actuator = tryLoadingActuatorFromXMLNode<Heater>(actuator_node);
    if (actuator == nullptr) actuator = tryLoadingActuatorFromXMLNode<Pusher>(actuator_node);
    if (actuator != nullptr) addActuator(actuator);
  }

  return true;
}

bool SpringSimulator::loadFromXML(std::string xml_file) {
  clear();

  pugi::xml_document xml;

  if (xml.load_file_or_string(xml_file)) {
    auto simulator_node = xml.child("simulator");
    if (simulator_node.empty()) return false;
    return loadFromXMLNode(simulator_node);
  } else return false;
}

pugi::xml_document SpringSimulator::toXML() const {
  pugi::xml_document xml;

  auto simulator_node = xml.append_child("simulator");
  simulator_node.append_attribute("type") = generic_name().c_str();
  simulator_node.append_attribute("time") = time_;
  simulator_node.append_attribute("scale") = scale_;

  auto current_state = new SpringSimulatorState(this);

  auto state_xml = current_state->toXML();
  simulator_node.append_copy(state_xml.root().first_child());

  auto actuators_node = simulator_node.append_child("actuators");
  for (auto actuator : actuators_) {
    auto actuator_xml = actuator->toXML();
    actuators_node.append_copy(actuator_xml.root().first_child());
  }
  return xml;
}

void SpringSimulator::incrementTime() {
  time_++;
  for (auto& actuator : actuators_) actuator->setTime(time_);
}

Spring* SpringSimulator::checkAndAddSpring(Particle *p1, Particle *p2) {
  if (p1 && p2) {
    for (auto s : p1->springs())
      if (s->otherEnd(p1) == p2) return nullptr;

    return new Spring(p1, p2, settings_->springDefaultLength(), settings_->springDefaultStiffness());
  } else return nullptr;
}

void SpringSimulator::initializeField(InitializationGrid mode, Point center, double width, double height,
                                      double interval, std::function<bool(double, double)> valid_point) {
  log_ << "Initializing field";
  auto timer = std::chrono::steady_clock::now();

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

  log_ << " (" << particles_.size() << " particles) " << stopwatch(timer) << " ms\n";
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
  scale_ = scale;
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

void SpringSimulator::initializeFromShape(const Shape& shape, double scale, InitializationGrid mode) {
  scale_ = scale;
  auto bounding_rect = shape.bounding_rectangle();
  double min_x = bounding_rect.first.x;
  double min_y = bounding_rect.first.y;
  double max_x = bounding_rect.second.x;
  double max_y = bounding_rect.second.y;
  auto new_shape = shape.clone();

  // ensure that the shape is in the positive quadrant
  if (min_x < 0 || min_y < 0) {
    Point center = shape.centroid();
    if (min_x < 0) center.x -= min_x;
    if (min_y < 0) center.y -= min_y;
    new_shape->moveTo(center);
  }
  auto interval = defaultInitializationInterval();
  initializeField(mode, Point(max_x * scale / 2,
                              max_y * scale / 2),
                        max_x * scale,
                        max_y * scale, interval,
                        [&](double x, double y) {
    return new_shape->contains(Point(x / scale, y / scale));
  });
  delete new_shape;
}

// this algorithm relies on a complete triangulation
template<typename T>
void calculateOpposingSprings(const std::vector<Particle*>& particles,
                              T& opposing_springs) {
  std::vector<Spring*> springs;
  for (auto p : particles)
    for (auto s : p->springs()) springs.push_back(s);

  for (auto spring : springs) {
    std::map<Particle*, int> neighbour_count;
    auto p1 = spring->particle1();
    auto p2 = spring->particle2();
    for (auto s : p1->springs()) neighbour_count[s->otherEnd(p1)]++;
    for (auto s : p2->springs()) neighbour_count[s->otherEnd(p2)]++;
    for (auto neighbour : neighbour_count) {
      if (neighbour.second > 1) {
        // common neighbour of p1 and p2
        opposing_springs[neighbour.first].push_back(spring);
      }
    }
  }
}

void SpringSimulator::relax(bool extra_long_relaxation) {
  double max_displacement = 0;
  int iteration_count = 0;
  movable_particles_.clear();
  for (auto p : particles_)
    if (p->isMovable()) movable_particles_.push_back(p);

  log_ << movable_particles_.size() << " movable particles\n";
  auto timer = std::chrono::steady_clock::now();

  //std::map<Particle*, std::vector<Spring*>> opposing_springs;
  //calculateOpposingSprings(movable_particles_, opposing_springs);

  bool is_spring_crossing_allowed = false;
  for (auto actuator : actuators_) is_spring_crossing_allowed |= actuator->isSpringCrossingAllowed();

  do {
    max_displacement = 0;
    for (auto p : movable_particles_) {
      double x_displacement = 0.0;
      double y_displacement = 0.0;
      //std::set<Particle*> neighbours = {};
      auto max_allowable_displacement = std::numeric_limits<double>::max();
      for (auto s : p->springs()) {
        double delta_x = s->otherEnd(p)->x() - p->x();
        double delta_y = s->otherEnd(p)->y() - p->y();
        if (s->force() > 0) delta_x = -delta_x, delta_y = -delta_y;
        double delta_module = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        //neighbours.insert(s->otherEnd(p));
        // make sure not to divide by [close to] zero
        if (delta_module < 1e-5) continue;
        delta_x /= delta_module;
        delta_y /= delta_module;
        delta_x *= std::fabs(s->force());
        delta_y *= std::fabs(s->force());
        x_displacement += delta_x;
        y_displacement += delta_y;

        if (!is_spring_crossing_allowed)
          max_allowable_displacement = std::min(max_allowable_displacement, s->actualLength() / 4);
      }
      /*
      for (auto neighbour : neighbours) {
        for (auto neighbour_spring : neighbour->springs()) {
          if (neighbours.count(neighbour_spring->otherEnd(neighbour)) &&
              neighbour < neighbour_spring->otherEnd(neighbour)) {
            double distance_to_spring_squared = distance2(p,
                                                          neighbour,
                                                          neighbour_spring->otherEnd(neighbour));
            if (max_allowable_displacement * max_allowable_displacement * 4 > distance_to_spring_squared)
              max_allowable_displacement = std::sqrt(distance_to_spring_squared) / 2;
          }
        }
      }
      */
      /*
      if (!is_spring_crossing_allowed)
      for (auto s : opposing_springs[p]) {
        if (segmentsIntersect(p->point(), Point(p->x() + x_displacement,
                                                p->y() + y_displacement),
                              s->particle1()->point(), s->particle2()->point())) {
          double distance_to_spring_squared = distance2(p,
                                                        s->particle1(),
                                                        s->particle2(), false);
          if (max_allowable_displacement * max_allowable_displacement * 4 > distance_to_spring_squared)
            max_allowable_displacement = std::sqrt(distance_to_spring_squared) / 2;
        }
      }
      */

      double total_displacement = distance(Point(0, 0), Point(x_displacement, y_displacement));
      if (total_displacement > max_allowable_displacement) {
        x_displacement /= (total_displacement / max_allowable_displacement);
        y_displacement /= (total_displacement / max_allowable_displacement);
      }
      max_displacement = std::max(max_displacement, total_displacement);
      p->setDisplacement(Point(x_displacement, y_displacement));
    }

    for (auto p : movable_particles_) {
      p->applyDisplacement();
    }

    if (iteration_count % 50 == 0) {
      updateConnectivity();
      //calculateOpposingSprings(movable_particles_, opposing_springs);
    }

    for (auto p : movable_particles_) {
      for (auto s : p->springs()) s->updateForce();
    }

    iteration_count++;
  } while (max_displacement > settings_->relaxationConvergenceLimit() * (extra_long_relaxation ? 0.1 : 1) &&
           iteration_count < settings_->relaxationIterationLimit() * (extra_long_relaxation ? 10 : 1));

  log_ << iteration_count << " steps in " << stopwatch(timer) << " ms\n";
}

void SpringSimulator::runLinearPasses() {
  auto total_timer = std::chrono::steady_clock::now();

  std::unordered_map<Actuator*, int> ticks;
  int total_ticks = 0;
  for (auto actuator : actuators_) {
    if (actuator->enabled()) {
      if (actuator->speed() < 1e-5)
        ticks[actuator] = 0;
      else
        ticks[actuator] = static_cast<int>(std::floor(actuator->path().length() / actuator->speed())) + 1;
      total_ticks = std::max(total_ticks, ticks[actuator]);
    }
  }

  for (int tick = 0; tick <= total_ticks; ++tick) {
    for (auto actuator : actuators_) {
      if (actuator->enabled()) {
        actuator->setPathAdvancement(std::min(actuator->speed() * tick, actuator->path().length()));
      }
    }

    for (auto p : particles_) {
      for (auto a : actuators_) {
        if (a->enabled()) a->preprocessParticle(p);
      }
    }

    for (auto p : particles_) {
      for (auto a : actuators_) {
        if (a->enabled()) a->processParticle(p);
      }
    }

    for (auto p : particles_) {
      for (auto s : p->springs()) s->updateForce();
    }

    relax(tick == total_ticks);

    for (auto p : particles_) {
      for (auto a : actuators_) {
        if (a->enabled()) a->postprocessParticle(p);
      }
    }

    incrementTime();
  }


  // reset particles' states changed during these passes
  for (const auto& a : actuators_) {
    if (a->enabled()) {
      a->resetParticles(particles_);
    }
  }

  for (auto p : particles_) {
    for (auto s : p->springs()) s->updateForce();
  }

  relax();

  for (auto actuator : actuators_) actuator->disable();

  std::cout << "Total time: " << stopwatch(total_timer) << " ms\n";
  log_ << "Total time: " << stopwatch(total_timer) << " ms\n";
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

Polygon SpringSimulator::fieldContour() const {
  return particlesContour(particles_);
}

Polygon particlesContour(const std::vector<Particle*>& particles_) {
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
  return Polygon(contour);
}

void SpringSimulator::clear() {
  for (auto p : particles_) {
    delete p;
  }
  particles_.clear();
}

Path SpringSimulator::predictMoves(Polygon target, Actuator* actuator,
                                   double entry_margin, double exit_margin,
                                   int samples, int repeats, int angular_resolution) {
  const auto current_shape = fieldContour();
  auto current_area = current_shape.area();
  auto current_center = current_shape.centroid();
  target.moveTo(current_center);
  target.scaleTo(current_area);

  std::vector<std::pair<double, double>> vectors;
  for (int i = 0; i < angular_resolution; ++i) {
    vectors.push_back(std::make_pair(std::cos(2 * M_PI / angular_resolution * i),
                                     std::sin(2 * M_PI / angular_resolution * i)));
  }

  auto sampling_entry_contour = current_shape.samplingContour(vectors, entry_margin);
  auto sampling_exit_contour = current_shape.samplingContour(vectors, exit_margin);

  std::vector<Path> passes;
  while (samples--) {
    auto entry = sampling_entry_contour.sampleBoundary();
    int tries = 100;
    auto exit = sampling_exit_contour.sampleBoundary();
    while (tries--) {
      if (distance2(entry, exit) > (entry_margin + exit_margin) * (entry_margin + exit_margin)) break;
    }

    std::vector<Point> pass;
    pass.push_back(entry);
    pass.push_back(current_center);
    pass.push_back(exit);
    passes.push_back(Path(pass));
  }

  auto best_pass = *passes.begin();
  double best_distance = std::numeric_limits<double>::max();

  for (const auto pass : passes) {
    std::cout << "Pass " << pass.toString();
    SpringSimulator test_simulator(this);
    test_simulator.removeAllActuators();
    test_simulator.addActuator(actuator);
    actuator->setPath(pass);
    actuator->enable();
    for (int i = 0; i < repeats; ++i) {
      test_simulator.runLinearPasses();
    }
    auto new_shape = test_simulator.fieldContour();
    std::stringstream ss; ss << pass.toString() << "_result.csv";
    new_shape.saveToFile(ss.str());
    double distance = new_shape.distanceTo(target);
    std::cout << "distance is " << distance << std::endl;
    if (distance < best_distance) {
      best_distance = distance;
      best_pass = pass;
    }
  }
  std::cout << "Best pass is " << best_pass.toString() << std::endl;
  return best_pass;
}

SpringSimulator::~SpringSimulator() {
  clear();
}

double stopwatch(std::chrono::time_point<std::chrono::steady_clock> start) {
  std::chrono::duration<double> duration = std::chrono::steady_clock::now() - start;
  return duration.count() * 1000;
}
