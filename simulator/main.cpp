#include <cstdio>
#include <iostream>
#include <fstream>
#include <algorithm>

#ifdef QT_CORE_LIB
#include <QApplication>
#include <QString>

#include "ui/mainwindow.h"
#endif

#include "backend/Particle.h"
#include "backend/Shape.h"
#include "backend/Heater.h"
#include "backend/Pusher.h"
#include "backend/ElasticSimulator.h"
#include "backend/WaxSimulator.h"

#ifndef QT_CORE_LIB
#include "tclap/CmdLine.h"

void pngToArray(std::string filename, std::vector<std::vector<int>>& pixel_array, std::string current_path) {
  std::string array_filename = filename + ".array";
  std::string command = "python " + current_path + "/mask_to_array.py " + filename + " " + array_filename;
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
    std::vector<std::string> allowed_commands = {"simulate", "predict"};
    TCLAP::ValuesConstraint<std::string> commands_constraint(allowed_commands);
    TCLAP::ValueArg<std::string> command_argument("c", "command", "simulate (simulate actuator passes), or predict (find an actuator pass to reach a desired target shape)", true, "", &commands_constraint);
    TCLAP::ValueArg<std::string> settings_argument("s", "settings", "simulator parameters file (*.cfg)", false, "", "settings file");
    TCLAP::ValueArg<std::string> input_argument("i", "input", "*.png - black/white mask, *.csv - shape outline XY coordinates, *.xml or XML string in quotes - saved state or simulator", true, "", "input file");
    TCLAP::MultiArg<std::string> actuator_argument("a", "actuators", "XML filenames or XML strings of actuators", false, "actuator files");
    TCLAP::ValueArg<std::string> target_argument("t", "target", "file with XY coordinates of shape outline (2 numbers per line)", false, "", "target file");
    TCLAP::ValueArg<std::string> output_argument("o", "output", "will write a suggested pass (CSV with XY coordinates of points) for command ""predict"", and CSV/XML file or XML string with resulting state for ""simulate""", false, "", "output file");
    TCLAP::SwitchArg wax_argument("w", "wax", "indicates that wax (inelastic) simulator should be used", false);

    args.add(command_argument);
    args.add(settings_argument);
    args.add(input_argument);
    args.add(actuator_argument);
    args.add(target_argument);
    args.add(output_argument);
    args.add(wax_argument);
    args.parse(argc, argv);

    std::string settings_file = settings_argument.getValue();
    if (!settings_file.empty()) {
      // todo: load from settings file
    } else {
      std::cerr << "Warning: no settings file provided, using default (use -s)" << std::endl;
    }

    std::string command = command_argument.getValue();
    std::string input_filename = input_argument.getValue();

    SpringSimulator* simulator = nullptr;
    if (wax_argument.getValue())
      simulator = new WaxSimulator();
    else
      simulator = new SpringSimulator();

    if (!input_filename.empty()) {
      auto extension = input_filename.substr(input_filename.find_last_of(".") + 1, std::string::npos);
      std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
      if (extension == "png") {
        try {
          std::vector<std::vector<int>> pixel_array;
          std::string current_path = argv[0];
          current_path = current_path.substr(0, current_path.find_last_of("\\/"));
          pngToArray(input_filename, pixel_array, current_path);
          if (pixel_array.empty() || pixel_array[0].empty()) {
            throw std::invalid_argument("");
          }

          simulator->initializeFromPixelArray(pixel_array, 1.0, [](int pixel) { return pixel > 0; });
        } catch (...) {
          std::cerr << "Error: cannot read pixel array from the input file" << std::endl;
        }
      } else if (extension == "csv") {
        Shape initial_shape(input_filename);
        simulator->initializeFromShape(initial_shape);
      } else {
        delete simulator;
        simulator = nullptr;
        if (simulator == nullptr) simulator = tryLoadingSimulatorFromFile<WaxSimulator>(input_filename);
        if (simulator == nullptr) simulator = tryLoadingSimulatorFromFile<ElasticSimulator>(input_filename);
        if (simulator == nullptr) simulator = new SpringSimulator();
        if ((dynamic_cast<WaxSimulator*>(simulator) == nullptr) == wax_argument.getValue()) {
          std::cerr << "Warning: loaded simulator has a different type than provided by -w switch" << std::endl;
        }
      }
      if (simulator == nullptr) {
        std::cerr << "Error: unknown input file format (PNG, CSV, XML files and XML strings are allowed)" << std::endl;
      }
    } else {
      std::cerr << "Error: no input file specified (PNG, CSV, XML file or XML string) (use -i)" << std::endl;
    }

    if ((simulator == nullptr) || simulator->particles().empty()) {
      std::cerr << "Error: could not initialize non-empty simulator, no particles created" << std::endl;
    } else {
      auto actuator_files = actuator_argument.getValue();
      std::vector<Actuator*> actuators;
      for (auto& actuator_file : actuator_files) {
        Actuator* actuator = nullptr;
        if (actuator == nullptr) actuator = tryLoadingActuatorFromFile<Heater>(actuator_file);
        if (actuator == nullptr) actuator = tryLoadingActuatorFromFile<Pusher>(actuator_file);
        if (actuator != nullptr) actuators.push_back(actuator);
      }

      // override existing actuators in simulator with the supplied ones
      if (!actuators.empty()) simulator->removeAllActuators();
      for (auto actuator : actuators) simulator->addActuator(actuator);

      if (simulator->actuators().empty()) {
        std::cerr << "Warning: no valid actuators provided (use -a)! Only an input-output type conversion will occur." << std::endl;
      } else {
        bool enabled_actuator_present = false;
        for (auto actuator : simulator->actuators()) {
          if (actuator->enabled()) enabled_actuator_present = true;
        }
        if (!enabled_actuator_present) {
          std::cerr << "Warning: all actuators are disabled! Only an input-output type conversion will occur." << std::endl;
        }
      }

      if (command == "simulate") {
        simulator->runLinearPasses();
      } else if (command == "predict") {
        std::string target_filename = target_argument.getValue();
        if (!target_filename.empty()) {
          auto actuator = simulator->actuators()[0];
          if (simulator->actuators().size() > 1u) {
            std::cerr << "Warning: too many actuators for prediction! Only the first one (" << actuator->name() << ") will be used." << std::endl;
          }
          std::string output_filename = output_argument.getValue();
          if (output_filename.empty()) {
            std::cerr << "Error: no CSV output filename provided (use -o)" << std::endl;
          } else {
            Shape target_shape(target_filename);
            auto moves = simulator->predictMoves(target_shape, actuator, 10, 10);
            Shape(moves.points()).saveToFile(output_filename);
          }
        } else {
          std::cerr << "Error: no target shape file provided for prediction (use -t)" << std::endl;
        }
      }

      std::string output_filename = output_argument.getValue();
      if (output_filename.empty()) {
        std::cerr << "Warning: no output filename provided, result is written to console as XML string" << std::endl;

        std::cout << simulator->toXMLString() << std:: endl;
      } else {
        auto extension = output_filename.substr(output_filename.find_last_of(".") + 1, std::string::npos);
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

        if (extension == "csv") {
          simulator->fieldContour().saveToFile(output_filename);
        } else if (extension == "xml") {
          simulator->saveToXML(output_filename);
        } else {
          std::cout << simulator->toXMLString() << std:: endl;
        }
      }
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
  // simulator->initializeFromPixelArray(rgb, 1.0, [](int color) { return color > 0; });

  // run a linear/piecewise linear pass of a heater:
  // auto heater = new Heater();
  // heater->setSize(20);
  // heater->setPath(Path({Point(-100, 0}, Point(100, 0)}));
  // heater->enable();
  // simulator->addActuator(heater);
  // simulator->runLinearPass();

  // access the result in a form of:
  //   - piecewise linear borderline
  // auto contour = simulator->fieldContour();
  //   - direct access of particles' coordinates
  // auto particles = simulator->particles();
  // auto x0 = particles[0]->x(); auto y0 = particles[0]->y();
  //   - saveable XML file or XML string
  // auto xml = simulator->toXML();
  // simulator->saveToXML("result.xml");
  // auto xml_string = simulator->toXMLString();
#endif
}
