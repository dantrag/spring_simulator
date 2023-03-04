#ifndef SIMULATORSETTINGS_H
#define SIMULATORSETTINGS_H

#include <string>

class SimulatorSettings {
 public:
  SimulatorSettings() {}
  SimulatorSettings(std::string settings_file) { loadFromFile(settings_file); }

  double particleDefaultRadius() const { return particle_default_radius_; }
  void setParticleDefaultRadius(double radius) { particle_default_radius_ = radius; }
  double moltenParticleDefaultRadius() const { return molten_particle_default_radius_; }
  void setMoltenParticleDefaultRadius(double radius) { molten_particle_default_radius_ = radius; }
  int moltenParticleCooldownTime() const { return molten_particle_cooldown_time_; }
  void setMoltenParticleCooldownTime(int time) { molten_particle_cooldown_time_ = time; }

  double springDefaultStiffness() const { return spring_default_stiffness_; }
  void setSpringDefaultStiffness(double stiffness) { spring_default_stiffness_ = stiffness; }
  double springDefaultLength() const { return spring_default_length_; }
  void setSpringDefaultLength(double length) { spring_default_length_ = length; }
  double springConnectionThreshold() const { return spring_connection_threshold_; }
  void setSpringConnectionThreshold(double threshold) { spring_connection_threshold_ = threshold; }
  double springDisconnectionThreshold() const { return spring_disconnection_threshold_; }
  void setSpringDisconnectionThreshold(double threshold) { spring_disconnection_threshold_ = threshold; }

  int relaxationIterationLimit() const { return relaxation_iteration_limit_; }
  void setRelaxationIterationLimit(int iterations) { relaxation_iteration_limit_ = iterations; }
  double relaxationConvergenceLimit() const { return relaxation_convergence_limit_; }
  void setRelaxationConvergenceLimit(double limit) { relaxation_convergence_limit_ = limit; }

  double actuatorSpeed() const { return actuator_speed_; }
  void setActuatorSpeed(double speed) { actuator_speed_ = speed; }
  double heaterRadius() const { return heater_radius_; }
  void setHeaterRadius(double radius) { heater_radius_ = radius; }

  void loadFromFile(std::string filename);
  void saveToFile(std::string filename);

 private:
  double particle_default_radius_ = 1.0;
  double molten_particle_default_radius_ = 2.0;
  int molten_particle_cooldown_time_ = 20;

  double spring_default_stiffness_ = 0.01;
  double spring_default_length_ = 5.5;
  double spring_connection_threshold_ = 1.0;
  double spring_disconnection_threshold_ = 1.3;

  int relaxation_iteration_limit_ = 2000;
  double relaxation_convergence_limit_ = 0.001;

  double actuator_speed_ = 2.0;
  double heater_radius_ = 20.0;
};

#endif // SIMULATORSETTINGS_H
