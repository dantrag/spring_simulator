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
  ui_->enabled_checkbox->setChecked(actuator->enabled());

  if (dynamic_cast<const Polygon*>(actuator->shape())) {
    auto polygon = dynamic_cast<const Polygon*>(actuator->shape());
    for (auto point : polygon->points()) {
      ui_->polygon_coordinates_line_edit->insert(QString("%1 %2 ").arg(point.x).arg(point.y));
    }
    ui_->polygon_coordinates_line_edit->backspace();
    ui_->polygon_coordinates_button->setChecked(true);
  } else if (dynamic_cast<const Circle*>(actuator->shape())) {
    auto circle = dynamic_cast<const Circle*>(actuator->shape());
    ui_->circle_coordinates_line_edit->insert(QString("%1 %2 %3").arg(circle->centroid().x)
                                                                 .arg(circle->centroid().y)
                                                                 .arg(circle->radius()));
    ui_->circle_coordinates_button->setChecked(true);
  }

  for (auto point : actuator->path().points()) {
    ui_->passes_text_edit->insertPlainText(QString("%1 %2 ").arg(point.x).arg(point.y));
  }

  ui_->actuator_preview->setScene(new QGraphicsScene(ui_->actuator_preview));

  connect(ui_->polygon_coordinates_line_edit, &QLineEdit::textChanged, this, &QActuatorWidget::updatePreview);
  connect(ui_->circle_coordinates_line_edit, &QLineEdit::textChanged, this, &QActuatorWidget::updatePreview);
  connect(ui_->polygon_coordinates_button, &QRadioButton::clicked, this, &QActuatorWidget::updatePreview);
  connect(ui_->circle_coordinates_button, &QRadioButton::clicked, this, &QActuatorWidget::updatePreview);

  connect(ui_->orientation_spinbox, QDoubleSpinBoxChanged, this, &QActuatorWidget::updatePreview);
  connect(ui_->speed_spinbox, QDoubleSpinBoxChanged, this, &QActuatorWidget::actuatorSpeedChanged);
  connect(ui_->passes_text_edit, &QPlainTextEdit::textChanged, this, &QActuatorWidget::actuatorPathChanged);
  connect(ui_->enabled_checkbox, &QCheckBox::clicked, this, &QActuatorWidget::actuatorEnabledChanged);
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

const Shape* QActuatorWidget::getShape() {
  if (ui_->polygon_coordinates_button->isChecked()) {
    auto string = ui_->polygon_coordinates_line_edit->text().toStdString();
    std::stringstream stream;
    stream << string;
    double x = 0.0;
    double y = 0.0;
    std::vector<Point> points;
    while (stream >> x >> y) {
      points.push_back(Point(x, y));
    }
    return new Polygon(points);
  } else if (ui_->circle_coordinates_button->isChecked()) {
    auto string = ui_->circle_coordinates_line_edit->text().toStdString();
    std::stringstream stream;
    stream << string;
    double x = 0.0;
    double y = 0.0;
    double r = 0.0;
    stream >> x >> y >> r;
    return new Circle(Point(x, y), r);
  } else {
    // should not happen
    return nullptr;
  }
}

double QActuatorWidget::getOrientation() {
  return M_PI * ui_->orientation_spinbox->value();
}

double QActuatorWidget::getSpeed() {
  return ui_->speed_spinbox->value();
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

QAbstractGraphicsShapeItem* drawActuator(QGraphicsScene* scene, const Shape* shape, double angle,
                                         QColor main_color, QColor secondary_color) {
  auto origin = shape->centroid();
  QAbstractGraphicsShapeItem* shape_item = nullptr;
  if (dynamic_cast<const Polygon*>(shape)) {
    QPolygonF polygon;
    for (auto point : dynamic_cast<const Polygon*>(shape)->points()) {
      polygon << QPointF(point.x, point.y);
    }
    polygon = QTransform().translate(origin.x, origin.y)
                          .rotateRadians(angle)
                          .translate(-origin.x, -origin.y)
                          .map(polygon);
    polygon = QTransform().translate(-origin.x, -origin.y)
                          .map(polygon);
    shape_item = scene->addPolygon(polygon,
                                   QPen(QBrush(main_color), 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
                                   QBrush(secondary_color));
  } else if (dynamic_cast<const Circle*>(shape)) {
    auto circle = dynamic_cast<const Circle*>(shape);
    shape_item = scene->addEllipse(origin.x - circle->radius(),
                                   origin.y - circle->radius(),
                                   circle->radius() * 2, circle->radius() * 2,
                                   QPen(QBrush(main_color), 1,
                                        Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
                                   QBrush(secondary_color));
  } else {
    // should not happen
  }
  return shape_item;
}

void QActuatorWidget::updatePreview() {
  ui_->actuator_preview->scene()->clear();

  auto shape = this->getShape();
  auto origin = shape->centroid();
  auto view_size = shape->diameter() * 2;
  drawActuator(ui_->actuator_preview->scene(), shape, getOrientation(),
               QColor::fromRgb(128, 37, 190), QColor::fromRgb(192, 146, 223));

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
