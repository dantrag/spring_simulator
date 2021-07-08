#ifndef QACTUATORWIDGET_H
#define QACTUATORWIDGET_H

#include <QWidget>
#include <QGraphicsItem>
#include <QGraphicsScene>

#include <backend/Shape.h>
#include <backend/Path.h>

namespace Ui {
  class QActuatorWidget;
}

class QActuatorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit QActuatorWidget(QWidget* parent,
                           double speed,
                           bool enabled,
                           bool show_spring_crossing_option,
                           bool show_firm_grip_option,
                           bool show_release_option,
                           bool spring_crossing_allowed = false,
                           bool firm_grip_allowed = false,
                           bool release_allowed = false);
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

 private:
  Ui::QActuatorWidget* ui_;
};

#endif // QACTUATORWIDGET_H
