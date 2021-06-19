#include "backend/InelasticSimulator.h"

#include <cmath>
#include <queue>
#include <unordered_map>
#include <map>
#include <algorithm>

//temp
#include <iostream>
#include <sstream>

#include "backend/Spring.h"

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

void InelasticSimulator::updateConnectivity() {
  SpringSimulator::updateConnectivity();

  auto spring_comparator = [](Spring* s1, Spring* s2) { return s1->actualLength() / s1->length() >
                                                               s2->actualLength() / s2->length(); };
  std::set<Spring*, decltype(spring_comparator)> sorted_springs(spring_comparator);
  for (auto p : movable_particles_) {
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

  // create new springs between close particles - but make sure there are no overlaps
  for (auto p : movable_particles_) {
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
}

