#ifdef QT_CORE_LIB
#include <QApplication>
#include <QString>
#include <iostream>

#include "ui/mainwindow.h"
#endif

//My includes
#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

#include "backend/SpringSimulator.h"
#include "backend/Spring.h"

int main(int argc, char *argv[])
{
#ifdef QT_CORE_LIB
  QApplication a(argc, argv);
  auto simulator = new SpringSimulator(QApplication::applicationDirPath() + "/default.cfg");

  MainWindow window(simulator);
  window.show();
  return a.exec();
#else
  auto simulator = new SpringSimulator();
  // change settings here, if needed, e.g.:
  simulator->settings()->setHeaterSpeed(20.0);

  // initialize the field from a:
  //   - predefined shape
  //
  //   - RGB pixel array
  //std::vector<std::vector<int>> rgb; // rgb values, i.e. green = 00ff00 = 65280
  //simulator->initializeFromPixelArray(rgb, 1.0, [](int color) { return color == 0; });

  // run a linear/piecewise linear pass of a heater:
  int trials = 10;
  int trajectory = 4;
  for (int i = 0; i < trials; ++i)
  {
    simulator->initializeCircle(Point(0, 0), 100.0);
    std::cout << "Trajectory: " << i << endl;
    for (int j = 0; j < trajectory; ++j)
    {
      // "Random deformations"

      int range = 50;
      // int v1 = -range + rand() % (2 * range);
      // int v2 = -range + rand() % (2 * range);
      // int v3 = -range + rand() % (2 * range);
      // int v4 = -range + rand() % (2 * range);

      int v1 = -rand() % (range);
      int v2 = rand() % (range);

      cout << v1 << ", " << v2 << endl;

      simulator->runLinearPass(Point(v1, 0), Point(v2, 0));

      // Save somewhere
      string save_dir = "saved_states/trial_" + to_string(i);
      const char *cstr = save_dir.c_str();
      mkdir(cstr, 0777);
      string fileName = save_dir + "/" + "traj_" + to_string(j) + ".txt";

      //Get state
      auto particles = simulator->particles();

      simulator->saveAction(Point(v1, 0), Point(v2, 0), fileName);
      simulator->getGraph(particles, fileName);
    }
  }

  // access the result in a form of:
  //   - piecewise linear borderline
  // auto contour = simulator->fieldContour();
  // //   - direct access of particles' coordinates

  // auto particles = simulator->particles();

  // vector<vector<Spring*>> graph;
  // //vector<vector<tuple<tuple<double>, tuple<double> > >> tuple_list;

  // vector<tuple<tuple<double, double>, tuple<double, double>>> tuple_list;
  // tuple_list = simulator->getGraph(particles);

  //simulator->saveTupleList(dir, index)

  // for (auto tpl : tuple_list){
  //   cout << "Spring: " << endl;
  //   cout << get<0>(get<0>(tpl)) << ", " << get<1>(get<0>(tpl)) << endl;
  //   cout << get<0>(get<1>(tpl)) << ", " << get<1>(get<1>(tpl)) << endl;
  // }

#endif
}
