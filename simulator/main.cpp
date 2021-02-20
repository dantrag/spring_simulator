#ifdef QT_CORE_LIB
#include <QApplication>
#include <QString>

#include "ui/mainwindow.h"
#endif

#include "backend/SpringSimulator.h"

int main(int argc, char* argv[]) {
#ifdef QT_CORE_LIB
  QApplication a(argc, argv);
  auto simulator = new SpringSimulator(QApplication::applicationDirPath() + "/default.cfg");

  MainWindow window(simulator);
  window.show();
  return a.exec();
#else
  auto simulator = new SpringSimulator();
  // change settings here, if needed, e.g.:
  // simulator->settings()->setHeaterSpeed(20.0);

  // initialize the field from a:
  //   - predefined shape
  // simulator->initializeCircle(Point(0, 0), 500.0);
  //
  //   - RGB pixel array
  // std::vector<std::vector<int>> rgb; // rgb values, i.e. green = 00ff00 = 65280
  // simulator->initializeFromPixelArray(rgb, 1.0, [](int color) { return color == 0; });

  // run a linear/piecewise linear pass of a heater:
  // simulator->runLinearPass(Point(-1000, 0), Point(1000, 0));

  // access the result in a form of:
  //   - piecewise linear borderline
  // auto contour = simulator->fieldContour();
  //   - direct access of particles' coordinates
  // auto particles = simulator->particles();
  // auto x0 = particles[0]->x(); auto y0 = particles[0]->y();
#endif
}
