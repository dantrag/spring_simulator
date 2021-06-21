#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath>
#include <set>
#include <iostream>
#include <exception>
#include <sstream>

#include <QDesktopWidget>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QWindow>
#include <QScreen>
#include <QFileDialog>
#include <QMessageBox>
#include <QButtonGroup>

#include "backend/ElasticSimulator.h"
#include "backend/WaxSimulator.h"
#include "backend/Spring.h"
#include "ui/qcustomgraphicsscene.h"

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

  if (bkg_image_ui_) ui_->graphicsView->scene()->removeItem(bkg_image_ui_);
  delete bkg_image_ui_;
  bkg_image_ui_ = nullptr;

  dynamic_cast<QCustomGraphicsScene*>(ui_->graphicsView->scene())->clearField();
}

void MainWindow::addNewState() {
  current_sim_state_ = new SpringSimulatorState(sim_, static_cast<int>(sim_states_.size()));
  sim_states_.push_back(current_sim_state_);
}

void MainWindow::restoreCurrentState() {
  //
}

void MainWindow::recreateSimulator() {
  auto settings = sim_->settings();
  delete sim_;
  auto simulator_type = static_cast<SimulatorType>(ui_->simulator_type_button_group->checkedId());
  switch (simulator_type) {
    case SimulatorType::kElastic: {
      sim_ = new ElasticSimulator(settings);
      break;
    }
    case SimulatorType::kInelastic: {
      sim_ = new WaxSimulator(settings);
      break;
    }
    default: {
      // should not happen, raise a warning!
      sim_ = nullptr;
    }
  }
  if (heater_) {
    delete heater_;
    heater_ = nullptr;
  }
  if (pusher_) {
    delete pusher_;
    pusher_ = nullptr;
  }
  if (sim_) {
    heater_ = new Heater();
    heater_->setSize(sim_->settings()->heaterSize());
    pusher_ = new Pusher();
    pusher_->setSpeed(1);
    sim_->addActuator(heater_);
    sim_->addActuator(pusher_);
  }
}

void MainWindow::createScene() {
  auto scene = new QCustomGraphicsScene(ui_->graphicsView);

  connect(scene, &QCustomGraphicsScene::mouseMoved, [this](double x, double y) {
    coordinates_label->setText(QString("X: %1  Y: %2").arg(x, 0, 'f', 0)
                                                      .arg(y, 0, 'f', 0));
  });
  connect(scene, &QCustomGraphicsScene::drawingFinished, [this]() {
    auto current_scene = dynamic_cast<QCustomGraphicsScene*>(ui_->graphicsView->scene());
    if (current_scene->currentMode() == QCustomGraphicsScene::MouseMode::kPassDrawing) {
      auto line = current_scene->getPass();
      ui_->passes_text_edit->appendPlainText(QString("%1 %2 %3 %4").arg(line.x1(), 0, 'f', 0)
                                                                   .arg(line.y1(), 0, 'f', 0)
                                                                   .arg(line.x2(), 0, 'f', 0)
                                                                   .arg(line.y2(), 0, 'f', 0));
      current_scene->releasePass();
    }
  });
  scene->setMode(static_cast<QCustomGraphicsScene::MouseMode>(ui_->drawing_mode_button_group->checkedId()));
  ui_->graphicsView->setScene(scene);
}

void MainWindow::initializeUI() {
  addNewState();

  delete ui_->graphicsView->scene();
  createScene();
  updateFieldUI();

  if (ui_->show_passes_checkbox->isChecked()) displayPasses();

  fitToView();
}

void MainWindow::initializeFieldCircle() {
  clearUI();
  recreateSimulator();

  double x = ui_->init_circle_x_spinbox->value();
  double y = ui_->init_circle_y_spinbox->value();
  double r = ui_->init_circle_radius_spinbox->value();
  auto mode = static_cast<SpringSimulator::InitializationGrid>(ui_->init_mode_button_group->checkedId());
  sim_->initializeCircle(Point(x, y), r, mode);
  initializeUI();
}

void MainWindow::initializeFieldRectangle() {
  clearUI();
  recreateSimulator();

  sim_ = new SpringSimulator();


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
        recreateSimulator();
        auto mode = static_cast<SpringSimulator::InitializationGrid>(ui_->init_mode_button_group->checkedId());
        sim_->initializeFromPixelArray(rgb_data, 1.0, [](int rgb) {
          return qGray(static_cast<QRgb>(rgb)) > 128;
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

  for (auto p : sim_->particles()) {
    if (p->x() >= left && p->x() <= right &&
        p->y() >= top && p->y() <= bottom) {
      p->setMolten(true);
      p->setMovable(true);
    }
  }
  for (auto p : sim_->particles()) {
    for (auto s : p->springs()) s->updateForce();
  }

  sim_->relax();
  addNewState();
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

  for (auto p : sim_->particles()) {
    if (p->x() >= left && p->x() <= right &&
        p->y() >= top && p->y() <= bottom &&
        p->isMolten()) {
      p->setMolten(false);
      p->setMovable(true);
    }
  }
  for (auto p : sim_->particles()) {
    for (auto s : p->springs()) s->updateForce();
  }

  sim_->relax();
  addNewState();
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

Actuator* MainWindow::currentActuator() {
  auto actuator_type = static_cast<ActuatorType>(ui_->actuator_button_group->checkedId());
  switch (actuator_type) {
    case ActuatorType::kHeater: {
      return heater_;
    }
    case ActuatorType::kPusher: {
      return pusher_;
    }
    default: {
      // should not happen
      return nullptr;
    }
  }
}

void MainWindow::runPasses() {
  auto passes = getPasses();
  for (auto& pass : passes) {
    sim_->runLinearPasses(currentActuator(), pass);
    addNewState();
  }

  updateFieldUI();
}

void MainWindow::makeTriangle() {
  /*
  auto contour = sim_->fieldContour();
  for (int i = 0; i < 1000; ++i) {
    double x = ((rand() % 1000 - 500) / 500.1) * 100;
    double y = ((rand() % 1000 - 500) / 500.1) * 100;
    ui_->graphicsView->scene()->addEllipse(x-2,y-2,4,4,QPen(contour.contains(Point(x, y)) ? Qt::blue : Qt::red));
  }
  return;*/

  for (int k = 0; k < 5; ++k) {
    auto contour = sim_->fieldContour();
    auto area = contour.area();
    auto side = std::sqrt(area * 4 / std::sqrt(3));
    Point p1(0.0, 0.0);
    Point p2(side, 0.0);
    Point p3(side / 2, side * std::sqrt(3) / 2);
    Shape triangle({p1, p2, p3});
    /*for (int i = 0; i < 3; ++i)
      ui_->graphicsView->scene()->addLine(triangle[i].x, triangle[i].y,
                                          triangle[(i + 1) % 3].x, triangle[(i + 1) % 3].y,
                                          QPen(Qt::gray));
    */
    int repeats = 4;
    auto best_pass = sim_->predictMoves(triangle, currentActuator(), sim_->settings()->heaterSize(), sim_->settings()->heaterSize(), 20, repeats);
    std::stringstream log_;
    //log_ << "triangle = [(" << p1.x << " " << p1.y << "), (" << p2.x << " " << p2.y << "), (" << p3.x << " " << p3.y << ")];" << std::endl;
    //ui_->passes_text_edit->insertPlainText(QString::fromStdString(log_.str()));
    //log_.str(std::string());
    //log_ << "contour = [";
    //for (auto point: contour) log_ << "(" << point.x << " " << point.y << "), ";
    //log_ << "];" << std::endl;
    //ui_->passes_text_edit->insertPlainText(QString::fromStdString(log_.str()));
    //log_.str(std::string());
    /*
    int x = 0;
    for (auto& pass : passes) {
      SpringSimulator test_sim(sim_);
      test_sim.runLinearPasses(pass);
      x++;

      //int n = pass.size();
      //for (int i = 0; i < n - 1; ++i)
      //  ui_->graphicsView->scene()->addLine(pass[i].x, pass[i].y, pass[(i + 1) % n].x, pass[(i + 1) % n].y, QPen(QColor::fromHslF((double(x) / passes.size()) * 0.8, 0.95, 0.5)));

      auto new_contour = test_sim.fieldContour();
      double diff = compareShapes(triangle, new_contour, false);
      if (diff < best_diff) {
        best_diff = diff;
        best_pass = pass;
      }

      //log_ << "Candidate: ";
      //for (auto point: new_contour) log_ << "(" << point.x << " " << point.y << "), ";
      //log_ << "];" << std::endl;
      //ui_->passes_text_edit->insertPlainText(QString::fromStdString(log_.str()));
      //log_.str(std::string());
      //log_ << "  diff: " << diff << std::endl;
    }
    //log_ << "best diff: " << best_diff;
    */
    int n = best_pass.size();
    for (int i = 0; i < n - 1; ++i) {
      ui_->graphicsView->scene()->addLine(best_pass[i].x, best_pass[i].y,
                                          best_pass[(i + 1) % n].x, best_pass[(i + 1) % n].y,
                                          QPen(Qt::blue));
    }
    QString filename = QString("move %1.png").arg(k);
    QImage image(QRect(-200, -200, 400, 400).size(), QImage::Format_ARGB32);
    image.fill(Qt::transparent);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    ui_->graphicsView->scene()->render(&painter);
    image.save(filename);
    for (int i = 0; i < repeats; ++i) sim_->runLinearPasses(currentActuator(), best_pass);
    addNewState();
    updateFieldUI();
  }
}

void MainWindow::updateFieldUI() {
  clearUI();

  if (current_sim_state_) {
    ui_->state_label->setText(QString("State %1").arg(current_sim_state_->id()));

    for (auto p : current_sim_state_->particles()) {
      particle_ui_[p] = ui_->graphicsView->scene()->addEllipse(p->x() - p->radius(),
                                                               p->y() - p->radius(),
                                                               2 * p->radius(),
                                                               2 * p->radius(),
                                                               QPen(Qt::darkRed), QBrush(Qt::darkRed));
      particle_ui_[p]->setToolTip(QString("(%1, %2)").arg(p->x(), 0, 'f', 0)
                                                     .arg(p->y(), 0, 'f', 0));
      particle_ui_[p]->setZValue(20);
    }
    for (auto s : current_sim_state_->springs()) {
      spring_ui_[s] = ui_->graphicsView->scene()->addLine(s->particle1()->x(), s->particle1()->y(),
                                                          s->particle2()->x(), s->particle2()->y(),
                                                          QPen(Qt::darkGreen));
      spring_ui_[s]->setToolTip(QString("L: %1\nS: %2").arg(s->actualLength(), 0, 'f', 1)
                                                       .arg(s->stretch(), 0, 'f', 2));
      spring_ui_[s]->setZValue(10);
    }
  }
  if (ui_->graphicsView->scene()) ui_->graphicsView->scene()->update();

  ui_->log_text_edit->clear();
  ui_->log_text_edit->insertPlainText(QString::fromStdString(sim_->log()));
  ui_->log_text_edit->ensureCursorVisible();
}

void MainWindow::displayState(SpringSimulatorState* state) {
  current_sim_state_ = state;
  updateFieldUI();
}

void MainWindow::decrementState() {
  auto iterator = std::find(sim_states_.begin(), sim_states_.end(), current_sim_state_);
  if (iterator != sim_states_.end() && iterator != sim_states_.begin()) {
    iterator--;
    displayState(*iterator);
  }
}

void MainWindow::incrementState() {
  auto iterator = std::find(sim_states_.begin(), sim_states_.end(), current_sim_state_);
  if (iterator != sim_states_.end() && *iterator != *sim_states_.rbegin()) {
    iterator++;
    displayState(*iterator);
  }
}

void MainWindow::changeBackground() {
  auto filename = QFileDialog::getOpenFileName(this, "Load background image",
                                               QApplication::applicationDirPath(),
                                               "Portable Network Graphics (*.png);;"
                                               "JPG/JPEG (*.jpg);;Windows Bitmap (*.bmp);;All Files (*)");
  if (!filename.isEmpty()) {
    try {
      QPixmap pixmap(filename);
      if (bkg_image_ui_) ui_->graphicsView->scene()->removeItem(bkg_image_ui_);
      delete bkg_image_ui_;
      bkg_image_ui_ = ui_->graphicsView->scene()->addPixmap(pixmap);
    } catch (const std::exception&) {
    }
  }
}

void MainWindow::changeBackgroundOpacity(int value) {
  if (bkg_image_ui_) bkg_image_ui_->setOpacity(double(value) / 100.0);
}

void MainWindow::updateZoom() {
  auto scale = double(ui_->zoom_slider->value()) / 100.0;
  ui_->graphicsView->resetMatrix();
  ui_->graphicsView->scale(scale, scale);
}

void MainWindow::fitToView() {
  ui_->graphicsView->fitInView(
        dynamic_cast<QCustomGraphicsScene*>(ui_->graphicsView->scene())->fieldBoundingRect(),
        Qt::KeepAspectRatio);
  ui_->zoom_slider->setValue(int(ui_->graphicsView->matrix().m11() * 100));
  updateZoom();
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
  fitToView();
}

void MainWindow::displayContour(bool show) {
  if (show) {
    double width = sim_->settings()->particleDefaultRadius() * 2;
    auto contour = sim_->fieldContour().points();
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

  connect(ui_->drawing_mode_button_group, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
          [&](int) { dynamic_cast<QCustomGraphicsScene*>(ui_->graphicsView->scene())->setMode(static_cast<QCustomGraphicsScene::MouseMode>(ui_->drawing_mode_button_group->checkedId())); });
}

MainWindow::MainWindow(SpringSimulator* simulator, QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow), sim_(simulator) {
  ui_->setupUi(this);

  coordinates_label = new QLabel(this);
  coordinates_label->setText("X: ---  Y: ---");
  ui_->statusbar->addPermanentWidget(coordinates_label);

  ui_->init_mode_button_group->setId(ui_->init_hexagonal_button,
                                     static_cast<int>(SpringSimulator::InitializationGrid::kHexagonal));
  ui_->init_mode_button_group->setId(ui_->init_square_button,
                                     static_cast<int>(SpringSimulator::InitializationGrid::kSquare));
  ui_->init_hexagonal_button->setChecked(true);

  ui_->simulator_type_button_group->setId(ui_->inelastic_button,
                                     static_cast<int>(SimulatorType::kInelastic));
  ui_->simulator_type_button_group->setId(ui_->elastic_button,
                                     static_cast<int>(SimulatorType::kElastic));
  ui_->inelastic_button->setChecked(true);

  ui_->actuator_button_group->setId(ui_->heater_button,
                                     static_cast<int>(ActuatorType::kHeater));
  ui_->actuator_button_group->setId(ui_->pusher_button,
                                     static_cast<int>(ActuatorType::kPusher));
  ui_->heater_button->setChecked(true);

  ui_->drawing_mode_button_group->setId(ui_->selection_button,
                                     static_cast<int>(QCustomGraphicsScene::MouseMode::kSelection));
  ui_->drawing_mode_button_group->setId(ui_->pass_drawing_button,
                                     static_cast<int>(QCustomGraphicsScene::MouseMode::kPassDrawing));
  ui_->selection_button->setChecked(true);

  createScene();

  populateSettings();

  connect(ui_->heat_button, &QPushButton::clicked, this, &MainWindow::doHeat);
  connect(ui_->cool_button, &QPushButton::clicked, this, &MainWindow::doCool);
  connect(ui_->submit_passes_button, &QPushButton::clicked, this, &MainWindow::runPasses);

  connect(ui_->zoom_slider, &QSlider::valueChanged, this, &MainWindow::updateZoom);
  connect(ui_->bkg_opacity_slider, &QSlider::valueChanged, this, &MainWindow::changeBackgroundOpacity);

  connect(ui_->load_settings_button, &QPushButton::clicked, this, &MainWindow::loadSettings);
  connect(ui_->save_settings_button, &QPushButton::clicked, this, &MainWindow::saveSettings);
  connect(ui_->init_circle_button, &QPushButton::clicked, this, &MainWindow::initializeFieldCircle);
  connect(ui_->init_rectangle_button, &QPushButton::clicked, this, &MainWindow::initializeFieldRectangle);
  connect(ui_->init_select_image, &QPushButton::clicked, this, &MainWindow::initializeFieldImage);
  connect(ui_->passes_text_edit, &QPlainTextEdit::textChanged, [&](){ ui_->show_passes_checkbox->setChecked(false); });
  connect(ui_->show_passes_checkbox, &QCheckBox::stateChanged, this, &MainWindow::displayPasses);
  connect(ui_->show_contour_checkbox, &QCheckBox::stateChanged, this, &MainWindow::displayContour);
  connect(ui_->change_bkg_button, &QPushButton::clicked, this, &MainWindow::changeBackground);
  connect(ui_->previous_state_button, &QToolButton::clicked, this, &MainWindow::decrementState);
  connect(ui_->next_state_button, &QToolButton::clicked, this, &MainWindow::incrementState);
  connect(ui_->restore_state_button, &QPushButton::clicked, this, &MainWindow::restoreCurrentState);
  connect(ui_->triangle_button, &QPushButton::clicked, this, &MainWindow::makeTriangle);
  connectSettingsSignals();

  auto screen = QApplication::screenAt(this->pos());
  if (screen) {
    auto rect = screen->availableGeometry();
    resize(rect.width() / 4 * 3, rect.height() / 4 * 3);
  }

  ui_->log_text_edit->setCenterOnScroll(true);

  updateZoom();
}

MainWindow::~MainWindow() {
  delete ui_;
}

