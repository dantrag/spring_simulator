#include "ui/qactuatorwidget.h"
#include "ui_qactuatorwidget.h"

#include <sstream>
#include <cmath>

#include <QFileDialog>

#include <backend/Particle.h>

QActuatorWidget::QActuatorWidget(QWidget* parent, const Actuator* actuator)
    : QWidget(parent), ui_(new Ui::QActuatorWidget) {
  ui_->setupUi(this);

  ui_->allow_spring_crossing_checkbox->setVisible(actuator->isSpringCrossingApplicable());
  ui_->allow_spring_crossing_checkbox->setChecked(actuator->isSpringCrossingAllowed());
  ui_->firm_grip_checkbox->setVisible(actuator->isFirmGripApplicable());
  ui_->firm_grip_checkbox->setChecked(actuator->isFirmGrip());
  ui_->release_grip_checkbox->setVisible(actuator->isFinalReleaseApplicable());
  ui_->release_grip_checkbox->setChecked(actuator->isFinalRelease());

  ui_->speed_spinbox->setValue(actuator->speed());
  ui_->orientation_spinbox->setValue(actuator->orientation() / M_PI);
  ui_->limit_force_checkbox->setChecked(actuator->forceRestrictionEnabled());
  ui_->force_spinbox->setValue(actuator->forceLimit());
  ui_->force_spinbox->setEnabled(ui_->limit_force_checkbox->isChecked());
  ui_->enabled_checkbox->setChecked(actuator->enabled());
  for (auto point : actuator->shape().points()) {
    ui_->coordinates_line_edit->insert(QString("%1 %2 ").arg(point.x).arg(point.y));
  }
  ui_->coordinates_line_edit->backspace();
  for (auto point : actuator->path().points()) {
    ui_->passes_text_edit->insertPlainText(QString("%1 %2 ").arg(point.x).arg(point.y));
  }

  ui_->actuator_preview->setScene(new QGraphicsScene(ui_->actuator_preview));

  connect(ui_->coordinates_line_edit, &QLineEdit::textChanged, this, &QActuatorWidget::updatePreview);
  connect(ui_->orientation_spinbox, QDoubleSpinBoxChanged, this, &QActuatorWidget::updatePreview);
  connect(ui_->speed_spinbox, QDoubleSpinBoxChanged, this, &QActuatorWidget::actuatorSpeedChanged);
  connect(ui_->passes_text_edit, &QPlainTextEdit::textChanged, this, &QActuatorWidget::actuatorPathChanged);
  connect(ui_->enabled_checkbox, &QCheckBox::clicked, this, &QActuatorWidget::actuatorEnabledChanged);
  connect(this, &QActuatorWidget::actuatorEnabledChanged, this, &QActuatorWidget::actuatorForceRestrictionChanged);
  connect(ui_->limit_force_checkbox, &QCheckBox::clicked, this, &QActuatorWidget::actuatorForceRestrictionChanged);
  connect(this, &QActuatorWidget::actuatorForceRestrictionChanged, [&](bool enabled) { ui_->force_spinbox->setEnabled(enabled); });
  connect(ui_->force_spinbox, QDoubleSpinBoxChanged, this, &QActuatorWidget::actuatorForceChanged);
  connect(ui_->allow_spring_crossing_checkbox, &QCheckBox::clicked, this, &QActuatorWidget::actuatorSpringCrossingChanged);
  connect(ui_->firm_grip_checkbox, &QCheckBox::clicked, this, &QActuatorWidget::actuatorFirmGripChanged);
  connect(ui_->release_grip_checkbox, &QCheckBox::clicked, this, &QActuatorWidget::actuatorFinalReleaseChanged);
  connect(ui_->clear_button, &QToolButton::clicked, [&]() { ui_->passes_text_edit->clear(); });
  connect(ui_->save_button, &QPushButton::clicked, this, &QActuatorWidget::saveToFile);

  updatePreview();
}

void QActuatorWidget::setActuatorEnabled(bool enabled) {
  ui_->enabled_checkbox->setChecked(enabled);
}

bool QActuatorWidget::isActuatorEnabled() {
  return ui_->enabled_checkbox->isChecked();
}

bool QActuatorWidget::isForceRestrictionEnabled() {
  return ui_->limit_force_checkbox->isChecked();
}

Shape QActuatorWidget::getShape() {
  auto string = ui_->coordinates_line_edit->text().toStdString();
  std::stringstream stream;
  stream << string;
  double x = 0.0;
  double y = 0.0;
  std::vector<Point> points;
  while (stream >> x >> y) {
    points.push_back(Point(x, y));
  }
  return Shape(points);
}

double QActuatorWidget::getOrientation() {
  return M_PI * ui_->orientation_spinbox->value();
}

double QActuatorWidget::getSpeed() {
  return ui_->speed_spinbox->value();
}

double QActuatorWidget::getForce() {
  return ui_->force_spinbox->value();
}

void QActuatorWidget::addPass(const Path& path) {
  QString string;
  for (const auto& point : path.points()) {
    string.append(QString("%1 %2 ").arg(point.x, 0, 'f', 0)
                                   .arg(point.y, 0, 'f', 0));
  }
  string.remove(string.length() - 1, 1);
  if (ui_->replace_toggle_button->isChecked())
    ui_->passes_text_edit->setPlainText(string);
  else
    ui_->passes_text_edit->appendPlainText(string);
}

Path QActuatorWidget::getPasses() {
  auto string = ui_->passes_text_edit->toPlainText().toStdString();
  std::stringstream stream;
  stream << string;
  double x = 0.0;
  double y = 0.0;
  std::vector<Point> points;
  while (stream >> x >> y) {
    points.push_back(Point(x, y));
  }
  return Path(points);
}

void QActuatorWidget::updatePreview() {
  ui_->actuator_preview->scene()->clear();

  auto shape = this->getShape();
  auto origin = shape.centroid();
  auto view_size = shape.diameter() * 2;

  QPolygonF polygon;
  for (auto point : shape.points()) {
    polygon << QPointF(point.x, point.y);
  }
  polygon = QTransform().translate(origin.x, origin.y)
                        .rotateRadians(getOrientation())
                        .translate(-origin.x, -origin.y)
                        .map(polygon);
  ui_->actuator_preview->scene()->addPolygon(polygon,
                        QPen(QBrush(QColor::fromRgb(128, 37, 190)), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
                        QBrush(QColor::fromRgb(192, 146, 223)));
  ui_->actuator_preview->setSceneRect(origin.x - view_size / 2, origin.y - view_size / 2,
                                      view_size, view_size);
  ui_->actuator_preview->centerOn(QPointF(origin.x, origin.y));
  ui_->actuator_preview->fitInView(ui_->actuator_preview->sceneRect(), Qt::KeepAspectRatio);
  emit actuatorGeometryChanged();
}

void QActuatorWidget::saveToFile() {
  auto filename = QFileDialog::getSaveFileName(this, "Save actuator to file",
                                               QApplication::applicationDirPath(),
                                               "Extensible Markup Language (*.xml);;"
                                               "All Files (*)");
  if (!filename.isEmpty()) emit saveActuatorToFile(filename);
}

QActuatorWidget::~QActuatorWidget() {
  delete ui_;
}
