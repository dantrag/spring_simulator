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
#include "backend/Heater.h"
#include "backend/Pusher.h"

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
  enum class SimulatorType {
    kElastic = 0,
    kInelastic,
  };

  enum class ActuatorType {
    kHeater = 0,
    kPusher,
  };

  Ui::MainWindow* ui_;
  std::deque<State*> states;
  std::deque<State*>::iterator current_state;

  void clearUI();
  void recreateSimulator();
  void initializeFieldCircle();
  void initializeFieldRectangle();
  void initializeFieldImage();
  void initializeUI();
  void updateFieldUI();
  void addNewState();
  void displayState(SpringSimulatorState* state);
  void decrementState();
  void incrementState();
  void restoreCurrentState();

  void doHeat();
  void doCool();
  // each heater pass is a piecewise linear curve defined by a vector of points
  std::vector<std::vector<Point>> getPasses();
  Actuator* currentActuator();
  void runPasses();
  void makeTriangle();

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

  Heater* heater_ = nullptr;
  Pusher* pusher_ = nullptr;
  SpringSimulator* sim_ = nullptr;
};
#endif // MAINWINDOW_H
