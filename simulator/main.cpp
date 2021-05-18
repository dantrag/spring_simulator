#include <cstdio>
#include <iostream>
#include <fstream>

#ifdef QT_CORE_LIB
#include <QApplication>
#include <QString>

#include "ui/mainwindow.h"
#endif

#include "backend/SpringSimulator.h"

#ifndef QT_CORE_LIB
#include "tclap/CmdLine.h"

void pngToArray(std::string filename, std::vector<std::vector<int>>& pixel_array, std::string current_path) {
  std::string array_filename = filename + ".array";
  std::string command = "python " + current_path + "/png_to_array.py " + filename + " " + array_filename;
  std::system(command.c_str());

  std::ifstream array_file;
  array_file.open(array_filename);
  int height = 0, width = 0;
  array_file >> height >> width;
  pixel_array.resize(height, std::vector<int>(width));
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      array_file >> pixel_array[i][j];
    }
  }
  array_file.close();

  std::remove(array_filename.c_str());
}
#endif

int main(int argc, char* argv[]) {
#ifdef QT_CORE_LIB
  QApplication a(argc, argv);
  auto simulator = new SpringSimulator(QApplication::applicationDirPath() + "/default.cfg");

  MainWindow window(simulator);
  window.show();
  return a.exec();
#else
  try {
    TCLAP::CmdLine args("Simulator control script");
    std::vector<std::string> allowed_commands = {"init", "pass", "predict"};
    TCLAP::ValuesConstraint<std::string> commands_constraint(allowed_commands);
    TCLAP::ValueArg<std::string> command_argument("c", "command", "init (initialize from image), pass (simulate a laser pass), or predict (find a pass to reach a desired target shape)", true, "", &commands_constraint);
    TCLAP::MultiArg<double> params_argument("p", "params", "laser pass XY coordinates (2 numbers per point, at least 2 points)", false, "coordinates");
    TCLAP::ValueArg<std::string> settings_argument("s", "settings", "simulator parameters file (*.cfg)", false, "", "settings file");
    TCLAP::ValueArg<std::string> input_argument("i", "input", "*.png if initialize from bitmask, or *.xml if read from saved state", true, "", "input file");
    TCLAP::ValueArg<std::string> target_argument("t", "target", "file with XY coordinates of shape outline (2 numbers per line)", false, "", "target file");
    TCLAP::ValueArg<std::string> output_argument("o", "output", "will write a suggested pass (XY coordinates of points) for command ""predict"", or XML with resulting state for ""init"" and ""pass""", false, "", "output file");

    args.add(command_argument);
    args.add(params_argument);
    args.add(settings_argument);
    args.add(input_argument);
    args.add(target_argument);
    args.add(output_argument);
    args.parse(argc, argv);

    auto simulator = new SpringSimulator();

    std::string settings_file = settings_argument.getValue();
    if (!settings_file.empty()) {
      // todo: load from settings file
    } else {
      std::cerr << "Warning: no settings file provided, using default (use -s)" << std::endl;
    }

    std::string command = command_argument.getValue();
    std::string input_file = input_argument.getValue();

    if (command == "init") {
      try {
        std::vector<std::vector<int>> pixel_array;
        std::string current_path = argv[0];
        current_path = current_path.substr(0, current_path.find_last_of("\\/"));
        pngToArray(input_file, pixel_array, current_path);
        if (pixel_array.empty() || pixel_array[0].empty()) {
          throw std::invalid_argument("");
        }
        if (pixel_array[0][0] != 0) {
          for (auto& row : pixel_array) {
            for (auto& pixel : row) {
              pixel = 255 - pixel;
            }
          }
        }
        simulator->initializeFromPixelArray(pixel_array, 1.0, [](int pixel) { return pixel > 0; });
      } catch (...) {
        std::cerr << "Error: cannot read pixel array from the input file" << std::endl;
      }
    } else if (command == "pass") {
      // todo: run linear pass
    } else if (command == "predict") {
      // todo: run prediction
    }
  } catch (TCLAP::ArgException &e) {
    std::cerr << "Error: " << e.error() << " for argument " << e.argId() << std::endl;
  } catch (...) {
    std::cerr << "Unknown error" << std::endl;
  }

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
