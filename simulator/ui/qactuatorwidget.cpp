#include "ui/qactuatorwidget.h"
#include "ui_qactuatorwidget.h"

#include <sstream>
#include <cmath>

#include <backend/Particle.h>

QActuatorWidget::QActuatorWidget(QWidget* parent,
                                 double speed,
                                 bool show_spring_crossing_option)
    : QWidget(parent), ui_(new Ui::QActuatorWidget) {
  ui_->setupUi(this);

  ui_->allow_spring_crossing_checkbox->setVisible(show_spring_crossing_option);
  ui_->speed_spinbox->setValue(speed);

  ui_->actuator_preview->setScene(new QGraphicsScene(ui_->actuator_preview));

  connect(ui_->coordinates_line_edit, &QLineEdit::textChanged, this, &QActuatorWidget::updatePreview);
  connect(ui_->orientation_spinbox, &QDoubleSpinBox::textChanged, this, &QActuatorWidget::updatePreview);
  connect(ui_->speed_spinbox, &QDoubleSpinBox::textChanged, this, &QActuatorWidget::actuatorSpeedChanged);
  connect(ui_->clear_button, &QToolButton::clicked, [&]() { ui_->passes_text_edit->clear(); });

  updatePreview();
}

bool QActuatorWidget::isActuatorEnabled() {
  return ui_->enabled_checkbox->isChecked();
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

void QActuatorWidget::addPath(const Path& path) {
  if (ui_->replace_toggle_button->isChecked()) {
    ui_->passes_text_edit->clear();
  }

  QString string;
  for (const auto& point : path.points()) {
    string.append(QString("%1 %2 ").arg(point.x, 0, 'f', 0)
                                   .arg(point.y, 0, 'f', 0));
  }
  string.remove(string.length() - 1, 1);
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
  auto points = shape.points();
  QPolygonF polygon;
  for (auto point : points) {
    polygon << QPointF(point.x, point.y);
  }
  auto origin = shape.centroid();
  polygon = QTransform().translate(origin.x, origin.y)
                        .rotateRadians(getOrientation())
                        .translate(-origin.x, -origin.y)
                        .map(polygon);
  ui_->actuator_preview->scene()->addPolygon(polygon,
                        QPen(QBrush(QColor::fromRgb(128, 37, 190)), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
                        QBrush(QColor::fromRgb(192, 146, 223)));
  auto view_size = shape.diameter() * 2;
  ui_->actuator_preview->setSceneRect(origin.x - view_size / 2, origin.y - view_size / 2,
                                      view_size, view_size);
  ui_->actuator_preview->centerOn(QPointF(origin.x, origin.y));
  ui_->actuator_preview->fitInView(ui_->actuator_preview->sceneRect(), Qt::KeepAspectRatio);
  emit actuatorGeometryChanged();
}

QActuatorWidget::~QActuatorWidget() {
  delete ui_;
}
