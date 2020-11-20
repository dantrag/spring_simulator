#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <deque>
#include <vector>
#include <unordered_map>

#include <QMainWindow>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>

#include "backend/SpringSimulator.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct State;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(SpringSimulator* simulator, QWidget *parent = nullptr);
  ~MainWindow();

  void setSimulator(SpringSimulator* simulator) { sim_ = simulator; }

private:
  Ui::MainWindow* ui_;
  std::deque<State*> states;
  std::deque<State*>::iterator current_state;

  void clearUI();
  void initializeFieldCircle();
  void initializeUI();
  void updateFieldUI();
  void doHeat();
  void doCool();
  // each heater pass is a piecewise linear curve defined by a vector of points
  std::vector<std::vector<Point>> getPasses();
  void runPasses();

  void toggleBlobMode();
  void updateZoom();
  void populateSettings();
  void loadSettings();
  void saveSettings();
  void connectSettingsSignals();
  void displayPasses(bool show = true);

  std::unordered_map<Particle*, QGraphicsEllipseItem*> particle_ui_;
  std::unordered_map<Spring*, QGraphicsLineItem*> spring_ui_;
  std::vector<std::vector<QGraphicsItem*>> passes_ui_;
  double blob_scale_ = 1.00;

  SpringSimulator* sim_;
};
#endif // MAINWINDOW_H
