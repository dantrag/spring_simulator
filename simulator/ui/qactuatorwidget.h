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
                           bool show_spring_crossing_option = false);
  ~QActuatorWidget();

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

 private:
  Ui::QActuatorWidget* ui_;
};

#endif // QACTUATORWIDGET_H
