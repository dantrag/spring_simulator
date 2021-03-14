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
// #ifdef QT_CORE_LIB
//   QApplication a(argc, argv);
//   auto simulator = new SpringSimulator(QApplication::applicationDirPath() + "/default.cfg");

//   MainWindow window(simulator);
//   window.show();
//   return a.exec();
// #else
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
    int trials = 5;
    int trajectory = 10;
    int range = 100;

    string saved_states_dir = "saved_states";
    mkdir(saved_states_dir.c_str(), 0777);
    
    for (int i = 0; i < trials; ++i)
    {
      simulator->initializeCircle(Point(0, 0), range);
      //simulator->initializeFromFileName("test.txt");
      std::cout << "Trial: " << i << endl;

      string save_dir = "saved_states/trial_" + to_string(i);
      const char *cstr = save_dir.c_str();
      mkdir(cstr, 0777);

      for (int j = 0; j < trajectory; ++j)
      {
        // "Random deformations"
        std::cout << "\t" << "Trajectory: " << j << endl;

        int x1;
        int y1;
        int x2;
        int y2;
        
        srand((i+1) * j);

        // Sample inside circle
        int m1 = -range / 2 + rand() % range;
        int m2 = -range / 2 + rand() % range;

        int random = rand() % 2;

        if(random == 0){
          x1 = -range + rand() % (2 * range);
          y1 = -range;

          x2 = -range + rand() % (2 * range);
          y2 = -y1;
        }
        else{
          x1 = -range;
          y1 = -range + rand() % (2 * range);

          x2 = -x1;
          y2 = -range + rand() % (2 * range);
        }


        std::cout << "Running linear pass " << "(" << x1 << "," << y1 << ")" << "(" << x2 << "," << y2 << ") (" << m1 << "," << m2 << ")" << std::endl;

        // Piecewise linear
        simulator->runLinearPass(Point(x1, y1), Point(m1, m2));
        simulator->runLinearPass(Point(m1, m2), Point(x2, y2));

        simulator->runLinearPass(Point(x1, y1), Point(m1, m2));
        simulator->runLinearPass(Point(m1, m2), Point(x2, y2));

        simulator->runLinearPass(Point(x1, y1), Point(m1, m2));
        simulator->runLinearPass(Point(m1, m2), Point(x2, y2));

        simulator->runLinearPass(Point(x1, y1), Point(m1, m2));
        simulator->runLinearPass(Point(m1, m2), Point(x2, y2));


        // Save somewhere

        string fileName = save_dir + "/" + "traj_" + to_string(j) + ".txt";

        //Get state
        auto particles = simulator->particles();
        simulator->saveAction(Point(x1, y1), Point(x2, y2), Point(m1, m2), fileName);
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

// #endif
}
