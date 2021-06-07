# Wax Simulator
C++-based simulator of 2d spots capable of changing shape via localized melting/cooling.

- [Background](#background)
  * [Settings](#settings)
- [Repository structure](#repository-structure)
- [For users](#for-users)
  * [File formats](#file-formats)
- [For developers](#for-developers)
  * [Build instructions](#build-instructions)
    + [Console version](#non-gui)
    + [GUI](#gui)

## Background

The simulated systems consists of a mass-spring model, where round particles are connected with springs. By default particles are frozen and cannot move. By using a heater (an imaginary circle that heats all particles inside it), one can change the state of particles from frozen to molten. This changes the size of the particles, simulating [thermal expansion](https://en.wikipedia.org/wiki/Thermal_expansion), and also allows particles to move freely to realize expansion. Molten particles become frozen again after a certain timeout and stop moving.

Slowly moving the heater across a spot of wax allows to achieve a flow (_phase change pumping_), resulting in the transfer of mass from one side of the spot to another. This is how the shape change is achieved. To enable a flow, the global connectivity of the particles needs to be changed. This is realized via removal of too long springs and addition of the springs between the particles that came too close.

### Settings

#### Particle
- _Particle default radius_ - radius of a frozen particle

- _Molten particle default radius_ - radius of a molten particle (typically larger than frozen one to simulate thermal expansion)

- _Molten particle cooldown time_ - time after which a just-molten particle becomes frozen again; time is measured in ticks (arbitraty time unit = time resolution of the simulation)

####  Spring
- _Spring default stiffness_ - [spring constant](https://en.wikipedia.org/wiki/Hooke%27s_law)

- _Spring default length_ - equilibrium spring length (i.e., the spring expands if the length is smaller than this value and contracts if the length is larger than this value)

- _Spring connection threshold_ - distance at which two particles become connected with a new spring, relative to the equilibrium spring length (i.e., if the threshold is 0.8, two particles become connected with a spring if they are closer than 80% of the equilibrium spring length)

- _Spring disconnection threshold_ - distance at which two connected particles become disconnected, relative to the equilibrium spring length

#### Simulator

- _Relaxation iteration limit_ - maximum number of iterations that simulator performs to equilibrate the newly molten and frozen particles

- _Relaxation convergence limit_ - primary stopping criterion for the simulator is "maximum displacement of a particle during iteration is less than convergence limit"
  

## Repository structure

#### ./builds/
Contains the executables of intermediate stable versions

#### ./configs/

Files containing simulator settings that allow to simulate realistic _phase change pumping_ observed in real experiments (each \*.cfg is a separate configuration).

#### ./simulator/backend
Standalone C++ code of simulator. This code is called a non-GUI version.

#### ./simulator/ui
Qt-based UI for the simulator

## For users

Non-GUI version of the simulator has a command line interface. The following options are supported:

| Option       | Details               |
|:-------------:|:-------------|
| -c      | specify command (*pass* simulates a piecewise-linear move of a heater; *predict* suggests the best piecewise-linear move of a heater to achieve target shape from current shape) |
| -i      | input file from which the current state of the spot is read; either a PNG image (currently only supports masks - black in grayscale is empty), a CSV file with the coordinates of a shape outline, or XML with the prreviously saved simulator state (not yet implemented) |
| -o      | output file; XML with the final state of the simulator if the *pass* command is supplied; CSV file with the coordinates of the piecewise-linear move if the *predict* command is supplied |
| -t      | target shape file; a CSV file with the coordinates of a target shape outline. The shape is scaled to the current spot area and translated to match shapes' barycentres. Used only with *predict* command. |
| -s      | settings file; see format above |
| -p      | parameters for the *pass* command; X and Y coordinates of the piecewise-linear move of a heater |
| -h      | show help |

GUI is mostly self-explanatory. The workflow is as follows. Adjust the simulator settings or load a configuration file. Then initialize the spot as a predefined shape or from an image. Then either select a square region and press 'Heat' or 'Cool' buttons to melt or freeze particles in the selected region, or specify a number of piecewise-linear passes of a heater in the corresponding text field and press 'Submit passes'.

### File formats

| CSV          | PNG               | XML |
|:-------------|:-------------|:---------|
| X,Y<br>point1x,point1y<br>point2x,point2y<br>...<br>pointNx,pointNy | regular PNG format<br>image is first converted to grayscale and then to a 2d array of numbers 0-255 (0 is nothing, non-zero numbers mean there are wax particles in that point) | TBD |

## For developers
  
### Build instructions

#### non-GUI

Make sure `make` and `g++` are installed. For Windows, use can use [Cygwin](https://www.cygwin.com/). Then run `make` in the root folder. This should create an executable in the _./builds/nongui_ folder.

#### GUI

On the top of `make` and `g++`, you need to have `Qt` installed, for example, via 'apt-get install qt5-default'. Go to _./simulator_, run `qmake` and then `make`. This should create an executable
