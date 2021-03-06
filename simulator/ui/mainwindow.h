#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <deque>
#include <vector>
#include <unordered_map>

#include <QMainWindow>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>

#include "backend/SpringSimulator.h"
#include "backend/SpringSimulatorState.h"

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
  void initializeFieldRectangle();
  void initializeFieldImage();
  void initializeUI();
  void updateFieldUI();
  void addNewState();
  void restoreState(SpringSimulatorState* state);
  void decrementState();
  void incrementState();

  void doHeat();
  void doCool();
  // each heater pass is a piecewise linear curve defined by a vector of points
  std::vector<std::vector<Point>> getPasses();
  void runPasses();

  void changeBackground();
  void changeBackgroundOpacity(int value);
  void updateZoom();
  void fitToView();
  void populateSettings();
  void loadSettings();
  void saveSettings();
  void connectSettingsSignals();
  void displayPasses(bool show = true);
  void displayContour(bool show = true);

  std::unordered_map<ParticleState*, QGraphicsEllipseItem*> particle_ui_;
  std::unordered_map<SpringState*, QGraphicsLineItem*> spring_ui_;
  std::vector<SpringSimulatorState*> sim_states_;
  SpringSimulatorState* current_sim_state_ = nullptr;

  std::vector<std::vector<QGraphicsItem*>> passes_ui_;
  std::vector<QGraphicsItem*> contour_ui_;
  QGraphicsPixmapItem* bkg_image_ui_ = nullptr;

  SpringSimulator* sim_;
};
#endif // MAINWINDOW_H
