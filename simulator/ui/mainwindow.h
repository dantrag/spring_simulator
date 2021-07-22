#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <deque>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <QMainWindow>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QLabel>

#include "backend/SpringSimulator.h"
#include "backend/SpringSimulatorState.h"
#include "backend/Heater.h"
#include "backend/Pusher.h"

#include "ui/qtoverloads.h"
#include "ui/qactuatorwidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

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

  static const int kActuatorTypeCount = 2;

  Ui::MainWindow* ui_;

  void clearUI();
  void recreateSimulator();
  void initializeFieldCircle();
  void initializeFieldRectangle();
  void initializeFieldImage();
  void initializeUI();
  void updateFieldUI();
  void createScene();
  void addNewState();
  void displayState(SpringSimulatorState* state);
  void decrementState();
  void incrementState();
  void restoreCurrentState();
  void loadStateFromFile();
  void saveStateToFile();
  void loadSimulatorFromFile();
  void saveSimulatorToFile();
  inline bool isLatestState();

  void doHeat();
  void doCool();
  Actuator* createActuatorByType(int type, bool& loaded);
  void addActuator();
  void addActuatorUI(Actuator* actuator, bool actuator_loaded);
  void removeActuator();
  void runPasses();
  void makeTriangle();

  void changeBackground();
  void updateBackgroundOpacity();
  void updateZoom();
  void fitToView();
  void populateSettings();
  void loadSettings();
  void saveSettings();
  void connectSettingsSignals();
  void eraseActuator(Actuator* actuator);
  void eraseActuators();
  void redrawActuator(Actuator* actuator);
  void redrawActuators();
  void displayActuators(bool show = true);
  void displayContour(bool show = true);
  void selectionChanged();

  void identifySelectedStates();
  std::vector<Particle*> getSelectedParticles();
  std::vector<Spring*> getSelectedSprings();

  template <class Item, class ItemState, class ItemUI>
  void restoreSelection(const std::vector<Item*>& items,
                        const std::unordered_map<ItemState*, ItemUI*> ui);

  QWidget* actuator_placeholder_ = nullptr;
  std::vector<Actuator*> actuators_;
  std::unordered_map<Actuator*, QActuatorWidget*> actuator_widgets_;
  std::unordered_map<QActuatorWidget*, Actuator*> widget_to_actuator_;

  std::unordered_map<ParticleState*, QGraphicsEllipseItem*> particle_ui_;
  std::unordered_map<SpringState*, QGraphicsLineItem*> spring_ui_;
  std::unordered_map<Actuator*, std::vector<QGraphicsItem*>> actuators_ui_;
  std::vector<SpringSimulatorState*> sim_states_;
  SpringSimulatorState* current_sim_state_ = nullptr;

  std::unordered_set<ParticleState*> selected_particle_states_;
  std::unordered_set<SpringState*> selected_spring_states_;

  std::vector<QGraphicsItem*> contour_ui_;
  QPixmap bkg_image_;
  QGraphicsPixmapItem* bkg_image_ui_ = nullptr;
  QLabel* coordinates_label = nullptr;

  SpringSimulator* sim_ = nullptr;
};
#endif // MAINWINDOW_H
