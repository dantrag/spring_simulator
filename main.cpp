#include <QApplication>
#include <QString>

#include "backend/SpringSimulator.h"
#include "ui/mainwindow.h"

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);

  auto simulator = new SpringSimulator(QApplication::applicationDirPath() + "/default.cfg");

  MainWindow window(simulator);
  window.show();
  return a.exec();
}
