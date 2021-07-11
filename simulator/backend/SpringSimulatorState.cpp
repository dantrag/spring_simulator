#include "backend/SpringSimulatorState.h"

#include <unordered_map>

#include "backend/Spring.h"
#include "backend/SpringSimulator.h"

SpringSimulatorState::SpringSimulatorState(const SpringSimulator* simulator, int id) {
  id_ = id;
  std::unordered_map<Particle*, ParticleState*> states = {};

  for (const auto p : simulator->particles()) {
    particles_.push_back(new ParticleState(p));
    states[p] = *particles_.rbegin();
  }

  for (const auto p : simulator->particles()) {
    for (const auto s : p->springs()) {
      // iterate only once each string
      if (p < s->otherEnd(p)) {
        springs_.push_back(new SpringState(states[p], states[s->otherEnd(p)],
                                           s->actualLength(), s->length()));
      }
    }
  }
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

bool SpringSimulatorState::loadFromXML(std::string xml_file) {
  clear();

  pugi::xml_document xml;
  auto extension = xml_file.substr(xml_file.find_last_of(".") + 1, std::string::npos);
  std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

  auto status = pugi::xml_parse_status::status_file_not_found;
  if (extension == "xml")
    status = xml.load_file(xml_file.c_str()).status;
  else
    status = xml.load_string(xml_file.c_str()).status;

  if (status == pugi::xml_parse_status::status_ok) {
    auto simulator_node = xml.child("simulator");
    if (simulator_node.empty()) return false;
    id_ = simulator_node.attribute("id").as_int(-1);

    auto particles_node = simulator_node.child("particles");
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

    auto springs_node = simulator_node.child("springs");
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
  } else return false;

  return !particles_.empty();
}

pugi::xml_document SpringSimulatorState::toXML() const {
  pugi::xml_document xml;
  auto simulator_node = xml.append_child("simulator");
  simulator_node.append_attribute("id") = id_;

  auto particles_node = simulator_node.append_child("particles");
  std::unordered_map<const ParticleState*, int> particle_id;
  for (int i = 0; i < static_cast<int>(particles_.size()); ++i) {
    particle_id[particles_[i]] = i;
    auto particle_node = particles_node.append_child("particle");
    particle_node.append_attribute("x") = particles_[i]->x();
    particle_node.append_attribute("y") = particles_[i]->y();
    particle_node.append_attribute("radius") = particles_[i]->radius();
    particle_node.append_attribute("id") = i;
  }

  auto springs_node = simulator_node.append_child("springs");
  for (const auto spring_state : springs_) {
    auto spring_node = springs_node.append_child("spring");
    spring_node.append_attribute("particle1id") = particle_id[spring_state->particle1()];
    spring_node.append_attribute("particle2id") = particle_id[spring_state->particle2()];
    spring_node.append_attribute("equilibrium_length") = spring_state->equilibriumLength();
    spring_node.append_attribute("force_constant") = spring_state->forceConstant();
  }

  return xml;
}

void SpringSimulatorState::saveToXML(std::string filename) const {
  toXML().save_file(filename.c_str());
}

std::string SpringSimulatorState::toString() const {
  pugi::xml_writer_string writer;
  toXML().save(writer);
  return writer.result;
}

void SpringSimulatorState::clear() {
  for (auto particle : particles_) delete particle;
  for (auto spring : springs_) delete spring;
  particles_.clear();
  springs_.clear();
}
