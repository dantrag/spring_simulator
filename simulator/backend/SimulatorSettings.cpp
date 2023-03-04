#include "backend/SimulatorSettings.h"

#include "mini/ini.h"

template <typename T>
T convert(std::string& value, T default_result) {
  T result = default_result;
  std::stringstream stream;
  stream << value;
  if (stream >> result) {
    return result;
  } else {
    stream.str(std::string());
    stream << default_result;
    value = stream.str();
    return default_result;
  }
}

template <typename T>
std::string convert(T value) {
  std::stringstream stream;
  stream << value;
  auto result = stream.str();
  return result;
}

void SimulatorSettings::loadFromFile(std::string filename) {
  mINI::INIFile settings_file(filename);
  mINI::INIStructure ini;

  settings_file.read(ini);

  particle_default_radius_ = convert(ini["Particle"]["DefaultRadius"], 1.0);
  molten_particle_default_radius_ = convert(ini["Particle"]["MoltenDefaultRadius"], 2.0);
  molten_particle_cooldown_time_ = convert(ini["Particle"]["CooldownTime"], 20);

  spring_default_stiffness_ = convert(ini["Spring"]["DefaultStiffness"], 0.01);
  spring_default_length_ = convert(ini["Spring"]["DefaultLength"], 5.5);
  spring_connection_threshold_ = convert(ini["Spring"]["ConnectionThreshold"], 1.0);
  spring_disconnection_threshold_ = convert(ini["Spring"]["DisconnectionThreshold"], 1.3);

  relaxation_iteration_limit_ = convert(ini["Relaxation"]["IterationLimit"], 2000);
  relaxation_convergence_limit_ = convert(ini["Relaxation"]["ConvergenceLimit"], 0.001);

  actuator_speed_ = convert(ini["Actuator"]["Speed"], 2.0);
  heater_radius_ = convert(ini["Heater"]["Radius"], 20.0);
}

void SimulatorSettings::saveToFile(std::string filename) {
  mINI::INIFile settings_file(filename);
  mINI::INIStructure ini;

  ini["Particle"]["DefaultRadius"] = convert(particle_default_radius_);
  ini["Particle"]["MoltenDefaultRadius"] = convert(molten_particle_default_radius_);
  ini["Particle"]["CooldownTime"] = convert(molten_particle_cooldown_time_);

  ini["Spring"]["DefaultStiffness"] = convert(spring_default_stiffness_);
  ini["Spring"]["DefaultLength"] = convert(spring_default_length_);
  ini["Spring"]["ConnectionThreshold"] = convert(spring_connection_threshold_);
  ini["Spring"]["DisconnectionThreshold"] = convert(spring_disconnection_threshold_);

  ini["Relaxation"]["IterationLimit"] = convert(relaxation_iteration_limit_);
  ini["Relaxation"]["ConvergenceLimit"] = convert(relaxation_convergence_limit_);

  ini["Actuator"]["Speed"] = convert(actuator_speed_);
  ini["Heater"]["Radius"] = convert(heater_radius_);

  settings_file.write(ini);
}
