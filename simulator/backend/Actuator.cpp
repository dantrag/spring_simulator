#include "backend/Actuator.h"

#include <cmath>

Actuator::Actuator() {
  setShape(Shape({Point(0.0, 0.0),
                  Point(1.0, 0.0),
                  Point(1.0, 1.0),
                  Point(0.0, 1.0)}));
}

Actuator::Actuator(std::string xml_file)
    : Actuator() {
  loadFromXML(xml_file);
}

void Actuator::setEnabled(bool enabled) {
  if (on_ != enabled) {
    if (enabled)
      enable();
    else
      disable();
  }
}

void Actuator::setPathAdvancement(double cumulative_length) {
  last_position_ = position_;
  if (path_.length() > 1e-5) {
    position_ = path_.sampleFraction(cumulative_length / path_.length());
    path_advancement_ = cumulative_length;
  } else {
    if (path_.size() > 0)
      position_ = path_.points()[0];
    else
      position_ = Point(0.0, 0.0);
    path_advancement_ = 0.0;
  }
}

void Actuator::setShape(Shape shape) {
  shape_ = shape;
  capture_particle_check_ = std::move([shape, this](const Particle* particle) {
    Shape oriented_shape(shape);
    oriented_shape.rotateBy(orientation_);
    oriented_shape.moveTo(position_);
    return oriented_shape.contains(particle->point());
  });
}

void Actuator::loadFromXML(std::string xml_file) {
  pugi::xml_document xml;
  auto extension = xml_file.substr(xml_file.find_last_of(".") + 1, std::string::npos);
  std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

  auto status = pugi::xml_parse_status::status_file_not_found;
  if (extension == "xml")
    status = xml.load_file(xml_file.c_str()).status;
  else
    status = xml.load_string(xml_file.c_str()).status;

  if (status == pugi::xml_parse_status::status_ok) {
    auto actuator_node = xml.child("actuator");

    setName(actuator_node.attribute("name").as_string());
    setSpeed(actuator_node.attribute("speed").as_double());
    setSpringCrossing(actuator_node.attribute("spring-crossing-allowed").as_bool());
    setFirmGrip(actuator_node.attribute("firm-grip").as_bool());
    setFinalRelease(actuator_node.attribute("final-release").as_bool());

    auto shape_node = actuator_node.child("shape");
    std::vector<Point> points = {};
    for (auto point_node = shape_node.child("point");
         point_node;
         point_node = point_node.next_sibling("point")) {
      points.push_back(Point(point_node.attribute("x").as_double(),
                             point_node.attribute("y").as_double()));
    }
    if (!points.empty()) setShape(Shape(points));
    setOrientation(actuator_node.attribute("orientation").as_double() / 180 * M_PI);

    points.clear();
    auto path_node = actuator_node.child("path");
    for (auto point_node = path_node.child("point");
         point_node;
         point_node = point_node.next_sibling("point")) {
      points.push_back(Point(point_node.attribute("x").as_double(),
                             point_node.attribute("y").as_double()));
    }
    setPath(Path(points));

    setPathAdvancement(actuator_node.attribute("path-advancement").as_double());
  }
}

pugi::xml_document Actuator::toXML() const {
  pugi::xml_document xml;
  auto actuator_node = xml.append_child("actuator");
  actuator_node.append_attribute("name") = name_.c_str();
  actuator_node.append_attribute("speed") = speed_;
  actuator_node.append_attribute("path-advancement") = path_advancement_;
  actuator_node.append_attribute("orientation") = orientation_ / M_PI * 180;
  actuator_node.append_attribute("spring-crossing-allowed") = spring_crossing_allowed_;
  actuator_node.append_attribute("firm-grip") = firm_grip_;
  actuator_node.append_attribute("final-release") = final_release_;

  auto shape_node = actuator_node.append_child("shape");
  for (const auto point : shape_.points()) {
    auto point_node = shape_node.append_child("point");
    point_node.append_attribute("x") = point.x;
    point_node.append_attribute("y") = point.y;
  }

  auto path_node = actuator_node.append_child("path");
  for (const auto point : path_.points()) {
    auto point_node = path_node.append_child("point");
    point_node.append_attribute("x") = point.x;
    point_node.append_attribute("y") = point.y;
  }

  return xml;
}

void Actuator::saveToXML(std::string filename) const {
  toXML().save_file(filename.c_str());
}

std::string Actuator::toString() const {
  pugi::xml_writer_string writer;
  toXML().save(writer);
  return writer.result;
}

ActuatorState Actuator::saveState() const {
  return ActuatorState(this);
}

void Actuator::loadState(const ActuatorState& state) {
  setEnabled(state.enabled);
  setTime(state.time);
  setPath(state.path);
  setPathAdvancement(state.path_advancement);
}

ActuatorState::ActuatorState(const Actuator* actuator) {
  enabled = actuator->enabled();
  path_advancement = actuator->pathAdvancement();
  time = actuator->time();
  path = actuator->path();
}
