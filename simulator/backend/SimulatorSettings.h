#ifndef SIMULATORSETTINGS_H
#define SIMULATORSETTINGS_H

#include <string>

#ifdef QT_CORE_LIB
#include <QString>
#include <QSettings>
#endif

class SimulatorSettings {
 public:
  SimulatorSettings() {}
  #ifdef QT_CORE_LIB
  SimulatorSettings(QString settings_file) { loadFromFile(settings_file); }
  #endif

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

  double heaterSpeed() const { return heater_speed_; }
  void setHeaterSpeed(double speed) { heater_speed_ = speed; }
  double heaterSize() const { return heater_size_; }
  void setHeaterSize(double size) { heater_size_ = size; }

  double pusherSpeed() const { return pusher_speed_; }
  void setPusherSpeed(double speed) { pusher_speed_ = speed; }

  #ifdef QT_CORE_LIB
  void loadFromFile(QString filename);
  void saveToFile(QString filename);
  #endif

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

  double heater_speed_ = 2.0;
  double heater_size_ = 20.0;

  double pusher_speed_ = 2.0;
};

#endif // SIMULATORSETTINGS_H
