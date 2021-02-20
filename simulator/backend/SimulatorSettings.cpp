#include "SimulatorSettings.h"

#ifdef QT_CORE_LIB
void SimulatorSettings::loadFromFile(QString filename) {
  QSettings file_settings(filename, QSettings::IniFormat);
  particle_default_radius_ = file_settings.value("Particle/DefaultRadius", 1.0).toDouble();
  molten_particle_default_radius_ = file_settings.value("Particle/MoltenDefaultRadius", 2.0).toDouble();
  molten_particle_cooldown_time_ = file_settings.value("Particle/CooldownTime", 20).toInt();

  spring_default_stiffness_ = file_settings.value("Spring/DeafultStiffness", 0.01).toDouble();
  spring_default_length_ = file_settings.value("Spring/DefaultLength", 5.5).toDouble();
  spring_connection_threshold_ = file_settings.value("Spring/ConnectionThreshold", 1.0).toDouble();
  spring_disconnection_threshold_ = file_settings.value("Spring/DisconnectionThreshold", 1.3).toDouble();

  relaxation_iteration_limit_ = file_settings.value("Relaxation/IterationLimit", 2000).toInt();
  relaxation_convergence_limit_ = file_settings.value("Relaxation/ConvergenceLimit", 0.001).toDouble();

  heater_speed_ = file_settings.value("Heater/Speed", 2.0).toDouble();
  heater_size_ = file_settings.value("Heater/Size", 20.0).toDouble();
}

void SimulatorSettings::saveToFile(QString filename) {
  QSettings file_settings(filename, QSettings::IniFormat);
  file_settings.setValue("Particle/DefaultRadius", particle_default_radius_);
  file_settings.setValue("Particle/MoltenDefaultRadius", molten_particle_default_radius_);
  file_settings.setValue("Particle/CooldownTime", molten_particle_cooldown_time_);

  file_settings.setValue("Spring/DeafultStiffness", spring_default_stiffness_);
  file_settings.setValue("Spring/DefaultLength", spring_default_length_);
  file_settings.setValue("Spring/ConnectionThreshold", spring_connection_threshold_);
  file_settings.setValue("Spring/DisconnectionThreshold", spring_disconnection_threshold_);

  file_settings.setValue("Relaxation/IterationLimit", relaxation_iteration_limit_);
  file_settings.setValue("Relaxation/ConvergenceLimit", relaxation_convergence_limit_);

  file_settings.setValue("Heater/Speed", heater_speed_);
  file_settings.setValue("Heater/Size", heater_size_);
}
#endif
