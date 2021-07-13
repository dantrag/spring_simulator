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

bool Actuator::loadFromXMLNode(pugi::xml_node root) {
  std::string type(root.attribute("type").as_string());
  if (!checkType(type)) return false;

  setName(root.attribute("name").as_string());
  setSpeed(root.attribute("speed").as_double());
  setEnabled(root.attribute("enabled").as_bool());
  setSpringCrossing(root.attribute("spring-crossing-allowed").as_bool());
  setFirmGrip(root.attribute("firm-grip").as_bool());
  setFinalRelease(root.attribute("final-release").as_bool());

  auto shape_node = root.child("shape");
  std::vector<Point> points = {};
  for (auto point_node = shape_node.child("point");
       point_node;
       point_node = point_node.next_sibling("point")) {
    points.push_back(Point(point_node.attribute("x").as_double(),
                           point_node.attribute("y").as_double()));
  }
  if (!points.empty()) setShape(Shape(points));
  setOrientation(root.attribute("orientation").as_double() / 180 * M_PI);

  points.clear();
  auto path_node = root.child("path");
  for (auto point_node = path_node.child("point");
       point_node;
       point_node = point_node.next_sibling("point")) {
    points.push_back(Point(point_node.attribute("x").as_double(),
                           point_node.attribute("y").as_double()));
  }
  setPath(Path(points));

  setPathAdvancement(root.attribute("path-advancement").as_double());

  return true;
}

bool Actuator::loadFromXML(std::string xml_file) {
  pugi::xml_document xml;

  if (xml.load_file_or_string(xml_file)) {
    auto actuator_node = xml.child("actuator");
    if (actuator_node.empty()) return false;
    return loadFromXMLNode(actuator_node);
  } else return false;
}

pugi::xml_document Actuator::toXML() const {
  pugi::xml_document xml;
  auto actuator_node = xml.append_child("actuator");
  actuator_node.append_attribute("type") = generic_name().c_str();
  actuator_node.append_attribute("name") = name_.c_str();
  actuator_node.append_attribute("enabled") = on_;
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

/*
std::string Actuator::loadGenericNameFromXML(std::string xml_file) {
  pugi::xml_document xml;
  auto extension = xml_file.substr(xml_file.find_last_of(".") + 1, std::string::npos);
  std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

  if (extension == "xml")
    xml.load_file(xml_file.c_str());
  else
    xml.load_string(xml_file.c_str());

  return std::string(xml.child("actuator").attribute("type").as_string());
}*/

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
