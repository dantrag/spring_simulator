#include "backend/SpringSimulatorState.h"

#include <unordered_map>

#include "backend/Spring.h"
#include "backend/SpringSimulator.h"

SpringSimulatorState::SpringSimulatorState(const SpringSimulator* simulator, int id) {
  id_ = id;

  for (const auto p : simulator->particles()) {
    particles_.push_back(new ParticleState(p));
    particle_state_mapping_[p] = *particles_.rbegin();
  }

  for (const auto p : simulator->particles()) {
    for (const auto s : p->springs()) {
      // iterate only once each string
      if (p < s->otherEnd(p)) {
        springs_.push_back(new SpringState(particle_state_mapping_[p],
                                           particle_state_mapping_[s->otherEnd(p)],
                                           s->length(), s->forceConstant()));
        spring_state_mapping_[s] = *springs_.rbegin();
      }
    }
  }
}

ParticleState* SpringSimulatorState::getItemState(Particle* particle) const {
  if (particle) {
    if (particle_state_mapping_.find(particle) != particle_state_mapping_.end())
      return particle_state_mapping_.at(particle);
    else
      return nullptr;
  }
  return nullptr;
}

SpringState* SpringSimulatorState::getItemState(Spring* spring) const {
  if (spring) {
    if (spring_state_mapping_.find(spring) != spring_state_mapping_.end())
      return spring_state_mapping_.at(spring);
    else
      return nullptr;
  }
  return nullptr;
}

SpringSimulatorState::SpringSimulatorState(std::string xml_file) {
  loadFromXML(xml_file);
}

Shape SpringSimulatorState::fieldContour() {
  std::vector<Particle*> particles;
  std::unordered_map<const ParticleState*, Particle*> state_to_particle;
  for (auto p : particles_) {
    auto particle = new Particle(p->x(), p->y(), nullptr);
    state_to_particle[p] = particle;
    particles.push_back(particle);
  }

  for (auto s : springs_) {
    new Spring(state_to_particle[s->particle1()],
               state_to_particle[s->particle2()],
               s->equilibriumLength(),
               s->forceConstant());
  }
  auto shape = particlesContour(particles);
  for (auto& p : particles) {
    delete p;
  }
  return shape;
}

bool SpringSimulatorState::loadFromXMLNode(pugi::xml_node root) {
  id_ = root.attribute("id").as_int(-1);

  auto particles_node = root.child("particles");
  if (particles_node.empty()) return false;

  std::unordered_map<int, ParticleState*> states;
  for (auto particle_node = particles_node.child("particle");
       particle_node;
       particle_node = particle_node.next_sibling("particle")) {
    auto id = particle_node.attribute("id").as_int(-1);
    if (id == -1) {
      clear();
      return false;
    }
    states[id] = new ParticleState(particle_node.attribute("x").as_double(),
                                   particle_node.attribute("y").as_double(),
                                   particle_node.attribute("radius").as_double());
    particles_.push_back(states[id]);
  }

  auto springs_node = root.child("springs");
  for (auto spring_node = springs_node.child("spring");
       spring_node;
       spring_node = spring_node.next_sibling("spring")) {
    auto id1 = spring_node.attribute("particle1id").as_int(-1);
    auto id2 = spring_node.attribute("particle2id").as_int(-1);
    if ((states.find(id1) == states.end()) ||
        (states.find(id2) == states.end())) {
      clear();
      return false;
    }
    springs_.push_back(new SpringState(states[id1], states[id2],
                                       spring_node.attribute("equilibrium_length").as_double(),
                                       spring_node.attribute("force_constant").as_double()));
  }

  return !particles_.empty();
}

bool SpringSimulatorState::loadFromXML(std::string xml_file) {
  clear();

  pugi::xml_document xml;

  if (xml.load_file_or_string(xml_file)) {
    auto state_node = xml.child("state");
    if (state_node.empty()) return false;
    return loadFromXMLNode(state_node);
  } else return false;
}

pugi::xml_document SpringSimulatorState::toXML() const {
  pugi::xml_document xml;
  auto state_node = xml.append_child("state");
  state_node.append_attribute("id") = id_;

  auto particles_node = state_node.append_child("particles");
  std::unordered_map<const ParticleState*, int> particle_id;
  for (int i = 0; i < static_cast<int>(particles_.size()); ++i) {
    particle_id[particles_[i]] = i;
    auto particle_node = particles_node.append_child("particle");
    particle_node.append_attribute("x") = particles_[i]->x();
    particle_node.append_attribute("y") = particles_[i]->y();
    particle_node.append_attribute("radius") = particles_[i]->radius();
    particle_node.append_attribute("id") = i;
  }

  auto springs_node = state_node.append_child("springs");
  for (const auto spring_state : springs_) {
    auto spring_node = springs_node.append_child("spring");
    spring_node.append_attribute("particle1id") = particle_id[spring_state->particle1()];
    spring_node.append_attribute("particle2id") = particle_id[spring_state->particle2()];
    spring_node.append_attribute("equilibrium_length") = spring_state->equilibriumLength();
    spring_node.append_attribute("force_constant") = spring_state->forceConstant();
  }

  return xml;
}

void SpringSimulatorState::clear() {
  for (auto particle : particles_) delete particle;
  for (auto spring : springs_) delete spring;
  particles_.clear();
  springs_.clear();
}

SpringSimulatorState::~SpringSimulatorState() {
  clear();
}
