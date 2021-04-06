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

  ui_->graphicsView->scene()->clear();
}

void MainWindow::addNewState() {
  current_sim_state_ = new SpringSimulatorState(sim_, static_cast<int>(sim_states_.size()));
  sim_states_.push_back(current_sim_state_);
}

void MainWindow::restoreCurrentState() {
  //
}

void MainWindow::initializeUI() {
  addNewState();

  delete ui_->graphicsView->scene();
  ui_->graphicsView->setScene(new QCustomGraphicsScene(ui_->graphicsView));
  updateFieldUI();

  if (ui_->show_passes_checkbox->isChecked()) displayPasses();

  fitToView();
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
        sim_->initializeFromPixelArray(rgb_data, 1.0, [](int rgb) {
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

  sim_->relaxHeat();
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

  sim_->relaxHeat();
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

void MainWindow::runPasses() {
  auto passes = getPasses();
  for (auto& pass : passes) {
    sim_->runLinearPasses(pass);
    addNewState();
  }

  updateFieldUI();
}

double interior_area(const std::vector<Point>& closed_curve) {
  double s = 0.0;
  int n = static_cast<int>(closed_curve.size());
  for (int i = 0; i < n; ++i) {
    s += (closed_curve[i].x - closed_curve[(i + 1) % n].x) *
         (closed_curve[i].y + closed_curve[(i + 1) % n].y);
  }
  return std::abs(s) / 2;
}

Point centroid(const std::vector<Point>& closed_curve) {
  Point center(0.0, 0.0);
  int n = static_cast<int>(closed_curve.size());
  if (n) {
    for (auto& point : closed_curve) {
      center.x += point.x;
      center.y += point.y;
    }
    center.x /= n;
    center.y /= n;
  }
  return center;
}

struct Line {
  Line() {}
  Line(const Point& p1, const Point& p2) {
    a = 1;
    if (p1.x == p2.x) {
      b = 0;
      c = -p1.x;
    } else {
      b = - a * (p2.x - p1.x) / (p2.y - p1.y);
      c = -a * p1.x - b * p1.y;
    }
  }

  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
};

bool isInside(const Point& point, const std::vector<Point>& curve) {
  int intersections = 0;
  int n = static_cast<int>(curve.size());
  if (n < 3) return false;
  double max_x = curve[0].x;
  double max_y = curve[0].y;
  for (int i = 0; i < n; ++i) {
    max_x = std::max(max_x, curve[i].x);
    max_y = std::max(max_y, curve[i].y);
  }
  // create a ray from the point, make sure it does not hit a vertex
  Point infinity(max_x + 100.0, max_y + 100.0);
  for (int i = 0; i < n; ++i) {
    const auto& p1 = curve[i];
    const auto& p2 = curve[(i + 1) % n];
    Line line(p1, p2);
    if (segmentsIntersect(p1, p2, point, infinity)) intersections++;
  }
  return (intersections & 1);
}

double curveLength(const std::vector<Point>& curve) {
  double length = 0.0;
  int n = static_cast<int>(curve.size());
  for (int i = 0; i < n; ++i) {
    length += distance(curve[i], curve[(i + 1) % n]);
  }
  return length;
}

std::vector<std::vector<Point>> generatePasses(const std::vector<Point>& contour, double margin, int how_many) {
  auto center = centroid(contour);
  std::vector<std::pair<double, double>> radius_vectors;
  for (int i = 0; i < 20; ++i) {
    radius_vectors.push_back(std::make_pair(margin * std::cos(2 * M_PI / 20 * i),
                                            margin * std::sin(2 * M_PI / 20 * i)));
  }

  std::vector<Point> sampling_contour;
  for (const auto& point : contour) {
    double separation_from_shape = -1;
    Point best_sampling_point(0.0, 0.0);
    for (const auto& radius : radius_vectors) {
      Point sampling_point(point.x + radius.first,
                           point.y + radius.second);
      if (isInside(sampling_point, contour)) continue;
      double separation = std::numeric_limits<double>::max();
      for (const auto& contour_point : contour) {
        separation = std::min(distance(sampling_point, contour_point), separation);
      }
      if (separation > separation_from_shape && separation != std::numeric_limits<double>::max()) {
        best_sampling_point = sampling_point;
        separation_from_shape = separation;
      }
    }
    if (separation_from_shape > 0) sampling_contour.push_back(best_sampling_point);
  }

  double curve_length = curveLength(sampling_contour);
  int n = static_cast<int>(sampling_contour.size());

  std::vector<std::vector<Point>> passes;
  while (how_many--) {
    passes.push_back({});
    double percentile = static_cast<double>(std::rand()) / RAND_MAX;
    double current_length = 0.0;
    for (int i = 0; i < n; ++i) {
      const auto& p1 = sampling_contour[i];
      const auto& p2 = sampling_contour[(i + 1) % n];
      auto segment_length = distance(p1, p2);
      current_length += segment_length;
      if (percentile * curve_length <= current_length) {
        double segment_prefix = (percentile * curve_length - (current_length - segment_length)) / segment_length;
        passes.rbegin()->push_back(Point(p1.x + (p2.x - p1.x) * segment_prefix,
                                         p1.y + (p2.y - p1.y) * segment_prefix));
        break;
      }
    }
    passes.rbegin()->push_back(center);
    double second_percentile = 0.0;
    int tries = 100;
    do {
      second_percentile = static_cast<double>(std::rand()) / RAND_MAX;
    } while (std::abs(percentile - second_percentile) * curve_length <= margin * 2 && (tries-- > 0));
    current_length = 0.0;
    for (int i = 0; i < n; ++i) {
      const auto& p1 = sampling_contour[i];
      const auto& p2 = sampling_contour[(i + 1) % n];
      auto segment_length = distance(p1, p2);
      current_length += segment_length;
      if (second_percentile * curve_length <= current_length) {
        double segment_prefix = (second_percentile * curve_length - (current_length - segment_length)) / segment_length;
        passes.rbegin()->push_back(Point(p1.x + (p2.x - p1.x) * segment_prefix,
                                         p1.y + (p2.y - p1.y) * segment_prefix));
        break;
      }
    }
    if (static_cast<int>(passes.rbegin()->size()) < 3) passes.pop_back();
  }
  return passes;
}

double compareShapes(const std::vector<Point>& shape1, const std::vector<Point>& shape2, bool hausdorff) {
  double length1 = curveLength(shape1);
  double length2 = curveLength(shape2);
  std::vector<Point> sampled1, sampled2;
  const int sample_points1 = 100;
  int n = static_cast<int>(shape1.size());
  for (int i = 0; i < sample_points1; ++i) {
    double current_length = 0.0;
    double percentile = double(i) / sample_points1;
    for (int j = 0; j < n; ++j) {
      const auto& p1 = shape1[j];
      const auto& p2 = shape1[(j + 1) % n];
      auto segment_length = distance(p1, p2);
      current_length += segment_length;
      if (percentile * length1 <= current_length) {
        double segment_prefix = (percentile * length1 - (current_length - segment_length)) / segment_length;
        sampled1.push_back(Point(p1.x + (p2.x - p1.x) * segment_prefix,
                                 p1.y + (p2.y - p1.y) * segment_prefix));
        break;
      }
    }
  }
  const int sample_points2 = hausdorff ? curveLength(shape2) / curveLength(shape1) * sample_points1
                                       : sample_points1;
  n = static_cast<int>(shape2.size());
  for (int i = 0; i < sample_points2; ++i) {
    double current_length = 0.0;
    double percentile = double(i) / sample_points2;
    for (int j = 0; j < n; ++j) {
      const auto& p1 = shape2[j];
      const auto& p2 = shape2[(j + 1) % n];
      auto segment_length = distance(p1, p2);
      current_length += segment_length;
      if (percentile * length2 <= current_length) {
        double segment_prefix = (percentile * length2 - (current_length - segment_length)) / segment_length;
        sampled2.push_back(Point(p1.x + (p2.x - p1.x) * segment_prefix,
                                 p1.y + (p2.y - p1.y) * segment_prefix));
        break;
      }
    }
  }
  // align two sets of points
  int closest_to_point0 = 0;
  for (int i = 1; i < static_cast<int>(sampled2.size()); ++i) {
    if (distance2(sampled1[0], sampled2[i]) < distance2(sampled1[0], sampled2[closest_to_point0]))
      closest_to_point0 = i;
  }
  std::rotate(sampled2.begin(), sampled2.begin() + closest_to_point0, sampled2.end());

  double diff = 0.0;
  if (hausdorff) {
    for (auto p1 : sampled1) {
      double dist = std::numeric_limits<double>::max();
      for (auto p2 : sampled2) {
        dist = std::min(dist, distance2(p1, p2));
      }
      diff = std::max(diff, dist);
    }
    for (auto p1 : sampled2) {
      double dist = std::numeric_limits<double>::max();
      for (auto p2 : sampled1) {
        dist = std::min(dist, distance2(p1, p2));
      }
      diff = std::max(diff, dist);
    }
  } else {
    for (int i = 0; i < static_cast<int>(sampled1.size()) &&
                    i < static_cast<int>(sampled2.size()); ++i) {
      diff += distance2(sampled1[i], sampled2[i]);
    }
  }
  return std::sqrt(diff);
}

void MainWindow::makeTriangle() {
  for (int k = 0; k < 20; ++k) {
    auto contour = sim_->fieldContour();
    auto area = interior_area(contour);
    auto side = std::sqrt(area * 4 / std::sqrt(3));
    auto shape_center = centroid(contour);
    Point p1(0.0, 0.0);
    Point p2(side, 0.0);
    Point p3(side / 2, side * std::sqrt(3) / 2);
    std::vector<Point> triangle = {p1, p2, p3};
    auto triangle_center = centroid(triangle);
    for (auto& triangle_vertex : triangle) {
      triangle_vertex.x -= (triangle_center.x - shape_center.x);
      triangle_vertex.y -= (triangle_center.y - shape_center.y);
    }
    for (int i = 0; i < 3; ++i)
      ui_->graphicsView->scene()->addLine(triangle[i].x, triangle[i].y,
                                          triangle[(i + 1) % 3].x, triangle[(i + 1) % 3].y,
                                          QPen(Qt::gray));

    auto passes = generatePasses(contour, sim_->settings()->heaterSize(), 20);
    if (passes.empty()) break;
    auto best_pass = *passes.begin();
    double best_diff = std::numeric_limits<double>::max();
    std::stringstream log_;
    //log_ << "triangle = [(" << p1.x << " " << p1.y << "), (" << p2.x << " " << p2.y << "), (" << p3.x << " " << p3.y << ")];" << std::endl;
    //ui_->passes_text_edit->insertPlainText(QString::fromStdString(log_.str()));
    //log_.str(std::string());
    //log_ << "contour = [";
    //for (auto point: contour) log_ << "(" << point.x << " " << point.y << "), ";
    //log_ << "];" << std::endl;
    //ui_->passes_text_edit->insertPlainText(QString::fromStdString(log_.str()));
    //log_.str(std::string());
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
    int n = best_pass.size();
    for (int i = 0; i < n - 1; ++i) {
      ui_->graphicsView->scene()->addLine(best_pass[i].x, best_pass[i].y,
                                          best_pass[(i + 1) % n].x, best_pass[(i + 1) % n].y,
                                          QPen(i ? Qt::red : Qt::blue));
    }
    sim_->runLinearPasses(best_pass);
    addNewState();
  }
  updateFieldUI();
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
  ui_->graphicsView->fitInView(ui_->graphicsView->scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
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

  ui_->init_mode_button_group->setId(ui_->init_hexagonal_button,
                                     static_cast<int>(SpringSimulator::InitializationGrid::kHexagonal));
  ui_->init_mode_button_group->setId(ui_->init_square_button,
                                     static_cast<int>(SpringSimulator::InitializationGrid::kSquare));
  ui_->init_hexagonal_button->setChecked(true);

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

