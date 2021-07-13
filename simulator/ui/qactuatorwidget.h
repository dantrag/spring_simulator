#ifndef QACTUATORWIDGET_H
#define QACTUATORWIDGET_H

#include <QWidget>
#include <QGraphicsItem>
#include <QGraphicsScene>

#include <backend/Shape.h>
#include <backend/Path.h>
#include <backend/Actuator.h>

namespace Ui {
  class QActuatorWidget;
}

class QActuatorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit QActuatorWidget(QWidget* parent, const Actuator* actuator);
  ~QActuatorWidget();

  void setActuatorEnabled(bool enabled);
  bool isActuatorEnabled();

  Shape getShape();
  double getOrientation();
  double getSpeed();

  void addPass(const Path& path);
  Path getPasses();

  void updatePreview();

 signals:
  void actuatorGeometryChanged();
  void actuatorPathChanged();
  void actuatorSpeedChanged();
  void actuatorEnabledChanged(bool enabled);
  void actuatorSpringCrossingChanged(bool allowed);
  void actuatorFirmGripChanged(bool allowed);
  void actuatorFinalReleaseChanged(bool allowed);
  void saveActuatorToFile(QString filename);

 protected:
  void saveToFile();

 private:
  Ui::QActuatorWidget* ui_;
};

#endif // QACTUATORWIDGET_H
