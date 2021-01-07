#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath>
#include <set>
#include <iostream>
#include <exception>

#include <QDesktopWidget>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QWindow>
#include <QScreen>
#include <QFileDialog>
#include <QMessageBox>

#include "backend/Spring.h"
#include "ui/qcustomgraphicsscene.h"

int kBlobScale = 4;

void MainWindow::clearUI() {
  displayPasses(false);
  for (auto p : particle_ui_) if (p.second) {
    ui_->graphicsView->scene()->removeItem(p.second);
    delete p.second;
  }
  particle_ui_.clear();
  for (auto s : spring_ui_) if (s.second) {
    ui_->graphicsView->scene()->removeItem(s.second);
    delete s.second;
  }
  spring_ui_.clear();
  ui_->graphicsView->scene()->clear();
}

void MainWindow::initializeUI() {
  delete ui_->graphicsView->scene();
  ui_->graphicsView->setScene(new QCustomGraphicsScene(ui_->graphicsView));
  if (ui_->show_passes_checkbox->isChecked()) displayPasses();

  for (auto p : sim_->particles_) {
    particle_ui_[p] = ui_->graphicsView->scene()->addEllipse(p->x() - p->radius(),
                                                             p->y() - p->radius(),
                                                             2 * p->radius(),
                                                             2 * p->radius(),
                                                             QPen(Qt::darkRed), QBrush(Qt::darkRed));
    // TODO: use constants for Z values;
    particle_ui_[p]->setZValue(20);

    for (auto s : p->springs()) {
      if (!spring_ui_.count(s)) {
        spring_ui_[s] = ui_->graphicsView->scene()->addLine(s->particle1()->x(), s->particle1()->y(),
                                                            s->particle2()->x(), s->particle2()->y(),
                                                            QPen(Qt::darkGreen));
        // TODO: use constants for Z values;
        spring_ui_[s]->setZValue(10);
      }
    }
  }

  ui_->graphicsView->fitInView(ui_->graphicsView->scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
  /*double scale = double(width()) / (ui_->graphicsView->width() + 1);
  ui_->graphicsView->scale(scale, scale);*/
  ui_->zoom_slider->setValue(int(ui_->graphicsView->matrix().m11() * 100));
  updateZoom();
}

void MainWindow::initializeFieldCircle() {
  clearUI();
  sim_->clear();

  double x = ui_->init_circle_x_spinbox->value();
  double y = ui_->init_circle_y_spinbox->value();
  double r = ui_->init_circle_radius_spinbox->value();
  auto mode = static_cast<SpringSimulator::InitializationGrid>(ui_->init_mode_button_group->checkedId());
  sim_->initializeCircle(Point(x, y), r, mode);
  initializeUI();
}

void MainWindow::initializeFieldRectangle() {
  clearUI();
  sim_->clear();

  double left = ui_->init_rect_left_spinbox->value();
  double top = ui_->init_rect_top_spinbox->value();
  double right = ui_->init_rect_right_spinbox->value();
  double bottom = ui_->init_rect_bottom_spinbox->value();
  auto mode = static_cast<SpringSimulator::InitializationGrid>(ui_->init_mode_button_group->checkedId());
  sim_->initializeRectangle(Point(left, top), Point(right, bottom), mode);
  initializeUI();
}

void MainWindow::initializeFieldImage() {
  auto filename = QFileDialog::getOpenFileName(this, "Open image for initialization",
                                               QApplication::applicationDirPath(),
                                               "Portable Network Graphics (*.png);;"
                                               "JPG/JPEG (*.jpg);;Windows Bitmap (*.bmp);;All Files (*)");
  if (!filename.isEmpty()) {
    try {
      QPixmap pixmap(filename);
      auto image = pixmap.toImage();
      std::vector<std::vector<int>> rgb_data(image.height());
      for (int i = 0; i < image.height(); ++i) {
        for (int j = 0; j < image.width(); ++j) rgb_data[i].push_back(static_cast<int>(image.pixel(j, i)));
      }

      if (!rgb_data.empty()) {
        clearUI();
        sim_->clear();
        auto mode = static_cast<SpringSimulator::InitializationGrid>(ui_->init_mode_button_group->checkedId());
        sim_->initializeFromPixelArray(rgb_data, 5.0, [](int rgb) {
          return qGray(static_cast<QRgb>(rgb)) < 128;
        }, mode);
        initializeUI();

      }
    } catch (const std::exception&) {
    }
  }
}

void MainWindow::doHeat() {
  auto scene = dynamic_cast<QCustomGraphicsScene*>(ui_->graphicsView->scene());
  auto rect = scene->getSelection();
  double left = std::min(rect.left(), rect.right());
  double right = std::max(rect.left(), rect.right());
  double top = std::min(rect.top(), rect.bottom());
  double bottom = std::max(rect.top(), rect.bottom());
  scene->releaseSelection();
  if (left == right || top == bottom) return;

  for (auto p : sim_->particles_) {
    if (p->x() >= left && p->x() <= right &&
        p->y() >= top && p->y() <= bottom) {
      p->setMolten(true);
      p->setMovable(true);
    }
  }
  for (auto p : sim_->particles_) {
    for (auto s : p->springs()) s->updateForce();
  }

  sim_->relaxHeat();
  updateFieldUI();
}

void MainWindow::doCool() {
  auto scene = dynamic_cast<QCustomGraphicsScene*>(ui_->graphicsView->scene());
  auto rect = scene->getSelection();
  double left = std::min(rect.left(), rect.right());
  double right = std::max(rect.left(), rect.right());
  double top = std::min(rect.top(), rect.bottom());
  double bottom = std::max(rect.top(), rect.bottom());
  scene->releaseSelection();

  for (auto p : sim_->particles_) {
    if (p->x() >= left && p->x() <= right &&
        p->y() >= top && p->y() <= bottom &&
        p->isMolten()) {
      p->setMolten(false);
      p->setMovable(true);
    }
  }
  for (auto p : sim_->particles_) {
    for (auto s : p->springs()) s->updateForce();
  }

  sim_->relaxHeat();
  updateFieldUI();
}

std::vector<std::vector<Point>> MainWindow::getPasses() {
  std::vector<std::vector<Point>> points;
  QStringList lines = ui_->passes_text_edit->toPlainText().split('\n', QString::SkipEmptyParts);
  for (QString line : lines) {
    points.push_back(std::vector<Point>());
    QStringList coordinates = line.split(' ', QString::SkipEmptyParts);
    for (int i = 0; i < coordinates.count() / 2; ++i)
      points.rbegin()->push_back(Point(coordinates[i * 2].toInt(), coordinates[i * 2 + 1].toInt()));
  }
  return points;
}

void MainWindow::runPasses() {
  auto passes = getPasses();
  for (auto& pass : passes) {
    sim_->runLinearPasses(pass);
  }

  updateFieldUI();
}

void MainWindow::updateFieldUI() {
  for (auto s : sim_->recentlyDeletedSprings()) {
    if (spring_ui_.count(s)) {
      ui_->graphicsView->scene()->removeItem(spring_ui_[s]);
      spring_ui_.erase(s);
    }
  }
  for (auto s : sim_->recentlyAddedSprings()) {
    spring_ui_[s] = ui_->graphicsView->scene()->addLine(s->particle1()->x(), s->particle1()->y(),
                                                        s->particle2()->x(), s->particle2()->y(),
                                                        QPen(Qt::darkGreen));
    // TODO: use constants for Z values;
    spring_ui_[s]->setZValue(10);
  }
  sim_->clearRecent();

  for (auto p : sim_->particles_) {
    particle_ui_[p]->setRect(p->x() - p->radius() * blob_scale_,
                             p->y() - p->radius() * blob_scale_,
                             2 * p->radius() * blob_scale_,
                             2 * p->radius() * blob_scale_);
    particle_ui_[p]->setToolTip(QString("(%1, %2)").arg(p->x(), 0, 'f', 0)
                                                   .arg(p->y(), 0, 'f', 0));
    for (auto s : p->springs()) {
      spring_ui_[s]->setLine(s->particle1()->x(), s->particle1()->y(),
                             s->particle2()->x(), s->particle2()->y());
      spring_ui_[s]->setToolTip(QString("L: %1\nE: %2").arg(s->length(), 0, 'f', 1)
                                                       .arg(s->actualLength() / s->length(), 0, 'f', 2));
    }
  }
  if (ui_->graphicsView->scene()) ui_->graphicsView->scene()->update();
}

void MainWindow::toggleBlobMode() {
  blob_scale_ = ui_->blob_mode_checkbox->isChecked() ? 4.0 : 1.0;
  for (auto p : sim_->particles_) {
    particle_ui_[p]->setRect(p->x() - p->radius() * blob_scale_,
                             p->y() - p->radius() * blob_scale_,
                             2 * p->radius() * blob_scale_,
                             2 * p->radius() * blob_scale_);
  }
  updateFieldUI();
}

void MainWindow::updateZoom() {
  auto scale = double(ui_->zoom_slider->value()) / 100.0;
  ui_->graphicsView->resetMatrix();
  ui_->graphicsView->scale(scale, scale);
}

void MainWindow::displayPasses(bool show) {
  if (show) {
    auto passes = getPasses();
    auto color = static_cast<int>(Qt::red);
    double width = sim_->settings()->springDefaultLength() / 2;
    for (auto& pass : passes) {
      passes_ui_.push_back(std::vector<QGraphicsItem*>());
      for (size_t i = 0; i < pass.size() - 1; ++i) {
        auto line = ui_->graphicsView->scene()->addLine(pass[i].x, pass[i].y,
                                                        pass[i + 1].x, pass[i + 1].y,
                                                        QPen(QBrush(QColor(static_cast<Qt::GlobalColor>(color))),
                                                             width, Qt::DotLine, Qt::RoundCap));
        // TODO: set Z values
        line->setOpacity(0.1);
        passes_ui_.rbegin()->push_back(line);
      }
      QVector2D vect = {0.0, 1.0};
      if (pass.size() > 1) {
        vect[0] = pass[1].x - pass[0].x;
        vect[1] = pass[1].y - pass[0].y;
        vect.normalize();
      }
      QPointF rotated_point = QTransform().rotate(135).map(QPointF(vect[0], vect[1]));
      vect[0] = rotated_point.x();
      vect[1] = rotated_point.y();
      auto startpoint = ui_->graphicsView->scene()->addLine(pass.begin()->x, pass.begin()->y,
                                                            pass.begin()->x + vect[0] * width * 2,
                                                            pass.begin()->y + vect[1] * width * 2,
                                                            QPen(QBrush(QColor(static_cast<Qt::GlobalColor>(color))),
                                                                 width, Qt::SolidLine, Qt::RoundCap));
      startpoint->setOpacity(0.1);
      passes_ui_.rbegin()->push_back(startpoint);
      rotated_point = QTransform().rotate(90).map(QPointF(vect[0], vect[1]));
      vect[0] = rotated_point.x();
      vect[1] = rotated_point.y();
      startpoint = ui_->graphicsView->scene()->addLine(pass.begin()->x, pass.begin()->y,
                                                       pass.begin()->x + vect[0] * width * 2,
                                                       pass.begin()->y + vect[1] * width * 2,
                                                       QPen(QBrush(QColor(static_cast<Qt::GlobalColor>(color))),
                                                            width, Qt::SolidLine, Qt::RoundCap));
      startpoint->setOpacity(0.1);
      passes_ui_.rbegin()->push_back(startpoint);

      vect = {0.0, -1.0};
      if (pass.size() > 1) {
        vect[0] = pass.rbegin()->x - pass[pass.size() - 2].x;
        vect[1] = pass.rbegin()->y - pass[pass.size() - 2].y;
        vect.normalize();
      }
      rotated_point = QTransform().rotate(135).map(QPointF(vect[0], vect[1]));
      vect[0] = rotated_point.x();
      vect[1] = rotated_point.y();
      auto endpoint = ui_->graphicsView->scene()->addLine(pass.rbegin()->x, pass.rbegin()->y,
                                                          pass.rbegin()->x + vect[0] * width * 2,
                                                          pass.rbegin()->y + vect[1] * width * 2,
                                                          QPen(QBrush(QColor(static_cast<Qt::GlobalColor>(color))),
                                                               width, Qt::SolidLine, Qt::RoundCap));
      endpoint->setOpacity(0.1);
      passes_ui_.rbegin()->push_back(endpoint);
      rotated_point = QTransform().rotate(90).map(QPointF(vect[0], vect[1]));
      vect[0] = rotated_point.x();
      vect[1] = rotated_point.y();
      endpoint = ui_->graphicsView->scene()->addLine(pass.rbegin()->x, pass.rbegin()->y,
                                                     pass.rbegin()->x + vect[0] * width * 2,
                                                     pass.rbegin()->y + vect[1] * width * 2,
                                                     QPen(QBrush(QColor(static_cast<Qt::GlobalColor>(color))),
                                                          width, Qt::SolidLine, Qt::RoundCap));
      endpoint->setOpacity(0.1);
      passes_ui_.rbegin()->push_back(endpoint);

      color++;
    }
  } else {
    for (auto pass : passes_ui_) {
      for (auto line : pass) {
        ui_->graphicsView->scene()->removeItem(line);
        delete line;
      }
    }
    passes_ui_.clear();
  }
  ui_->graphicsView->update();
}

void MainWindow::displayContour(bool show) {
  if (show) {
    double width = sim_->settings()->particleDefaultRadius();
    auto contour = sim_->fieldContour();
    auto count = static_cast<int>(contour.size());
    for (int i = 0; i < count; ++i) {
      auto line = ui_->graphicsView->scene()->addLine(contour[i].x, contour[i].y,
                                                      contour[(i + 1) % count].x, contour[(i + 1) % count].y,
                                                      QPen(QBrush(Qt::darkGreen), width,
                                                           Qt::SolidLine, Qt::RoundCap));
      // TODO: set Z values
      line->setOpacity(0.5);
      contour_ui_.push_back(line);
    }
  } else {
    for (auto line : contour_ui_) {
      ui_->graphicsView->scene()->removeItem(line);
      delete line;
    }
    contour_ui_.clear();
  }
  ui_->graphicsView->update();
}

void MainWindow::loadSettings() {
  auto filename = QFileDialog::getOpenFileName(this, "Load simulator settings from file",
                                               QApplication::applicationDirPath(),
                                               "Configuration file (*.cfg);;All Files (*)");
  if (filename.isEmpty())
    return;
  else {
    sim_->settings()->loadFromFile(filename);
    populateSettings();
    updateFieldUI();
  }
}

void MainWindow::saveSettings() {
  auto filename = QFileDialog::getSaveFileName(this, "Save simulator settings to file",
                                               QApplication::applicationDirPath(),
                                               "Configuration file (*.cfg);;All Files (*)");
  if (filename.isEmpty())
    return;
  else {
    sim_->settings()->saveToFile(filename);
  }
}

void MainWindow::populateSettings() {
  ui_->particle_radius_spinbox->setValue(sim_->settings()->particleDefaultRadius());
  ui_->particle_molten_radius_spinbox->setValue(sim_->settings()->moltenParticleDefaultRadius());
  ui_->cooldown_time_spinbox->setValue(sim_->settings()->moltenParticleCooldownTime());

  ui_->spring_length_spinbox->setValue(sim_->settings()->springDefaultLength());
  ui_->spring_stiffness_spinbox->setValue(sim_->settings()->springDefaultStiffness());
  ui_->spring_connection_spinbox->setValue(sim_->settings()->springConnectionThreshold());
  ui_->spring_disconnection_spinbox->setValue(sim_->settings()->springDisconnectionThreshold());

  ui_->iteration_limit_spinbox->setValue(sim_->settings()->relaxationIterationLimit());
  ui_->relaxation_convergence_spinbox->setValue(sim_->settings()->relaxationConvergenceLimit());

  ui_->heater_size_spinbox->setValue(sim_->settings()->heaterSize());
  ui_->heater_speed_spinbox->setValue(sim_->settings()->heaterSpeed());
}

void MainWindow::connectSettingsSignals() {
  connect(ui_->particle_radius_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setParticleDefaultRadius(value); });
  connect(ui_->particle_molten_radius_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setMoltenParticleDefaultRadius(value); });
  connect(ui_->cooldown_time_spinbox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          [&](int value) { sim_->settings()->setMoltenParticleCooldownTime(value); });
  connect(ui_->spring_length_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setSpringDefaultLength(value); });
  connect(ui_->spring_stiffness_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setSpringDefaultStiffness(value); });
  connect(ui_->spring_connection_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setSpringConnectionThreshold(value); });
  connect(ui_->spring_disconnection_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setSpringDisconnectionThreshold(value); });
  connect(ui_->relaxation_convergence_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setRelaxationConvergenceLimit(value); });
  connect(ui_->iteration_limit_spinbox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          [&](int value) { sim_->settings()->setRelaxationIterationLimit(value); });
  connect(ui_->heater_size_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setHeaterSize(value); });
  connect(ui_->heater_speed_spinbox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [&](double value) { sim_->settings()->setHeaterSpeed(value); });
}

MainWindow::MainWindow(SpringSimulator* simulator, QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow), sim_(simulator) {
  ui_->setupUi(this);

  ui_->graphicsView->setScene(new QCustomGraphicsScene(ui_->graphicsView));

  populateSettings();
  toggleBlobMode();

  connect(ui_->heat_button, &QPushButton::clicked, this, &MainWindow::doHeat);
  connect(ui_->cool_button, &QPushButton::clicked, this, &MainWindow::doCool);
  connect(ui_->submit_passes_button, &QPushButton::clicked, this, &MainWindow::runPasses);

  connect(ui_->blob_mode_checkbox, &QCheckBox::clicked, this, &MainWindow::toggleBlobMode);
  connect(ui_->zoom_slider, &QSlider::valueChanged, this, &MainWindow::updateZoom);

  connect(ui_->load_settings_button, &QPushButton::clicked, this, &MainWindow::loadSettings);
  connect(ui_->save_settings_button, &QPushButton::clicked, this, &MainWindow::saveSettings);
  connect(ui_->init_circle_button, &QPushButton::clicked, this, &MainWindow::initializeFieldCircle);
  connect(ui_->init_rectangle_button, &QPushButton::clicked, this, &MainWindow::initializeFieldRectangle);
  connect(ui_->init_select_image, &QPushButton::clicked, this, &MainWindow::initializeFieldImage);
  connect(ui_->passes_text_edit, &QPlainTextEdit::textChanged, [&](){ ui_->show_passes_checkbox->setChecked(false); });
  connect(ui_->show_passes_checkbox, &QCheckBox::stateChanged, this, &MainWindow::displayPasses);
  connect(ui_->show_contour_checkbox, &QCheckBox::stateChanged, this, &MainWindow::displayContour);
  connectSettingsSignals();

  ui_->init_mode_button_group->setId(ui_->init_hexagonal_button,
                                     static_cast<int>(SpringSimulator::InitializationGrid::kHexagonal));
  ui_->init_mode_button_group->setId(ui_->init_square_button,
                                     static_cast<int>(SpringSimulator::InitializationGrid::kSquare));
  ui_->init_hexagonal_button->setChecked(true);

  auto rect = this->screen()->geometry();
  resize(rect.width() / 3 * 2, rect.height() / 3 * 2);

  updateZoom();
}

MainWindow::~MainWindow() {
  delete ui_;
}

