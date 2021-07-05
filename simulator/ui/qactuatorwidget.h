#ifndef QACTUATORWIDGET_H
#define QACTUATORWIDGET_H

#include <QWidget>

#include <backend/Shape.h>

namespace Ui {
  class QActuatorWidget;
}

class QActuatorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit QActuatorWidget(QWidget* parent,
                           double speed,
                           bool show_spring_crossing_option = false);
  ~QActuatorWidget();

  bool isActuatorEnabled();
  Shape getShape();
  double getOrientation();
  double getSpeed();

  void updatePreview();

 signals:
  void actuatorGeometryChanged();
  void actuatorSpeedChanged();

 private:
  Ui::QActuatorWidget* ui_;
};

#endif // QACTUATORWIDGET_H
