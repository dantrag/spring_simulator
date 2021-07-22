# Spring Simulator
C++-based simulator of 2d spring-connected particle nets capable of changing shape via manipulating with actuators. Currently actuators include heaters (capable of melting/freezing the material) and pushers (grippers that can grip a piece of material and move).

- [Background](#background)
  * [Actuators](#actuators)
    + [Heater](#heater)
    + [Pusher](#pusher)
  * [Settings](#settings)
- [Repository structure](#repository-structure)
- [For users](#for-users)
  * [File formats](#file-formats)
- [For developers](#for-developers)
  * [Build instructions](#build-instructions)
    + [Console version](#non-gui)
    + [GUI](#gui)
  * [Simulation details](#simulation-details)
  * [Class reference](#class-reference)

## Background

The simulated system consists of a mass-spring model, where round particles are connected with springs. There are two types of simulators: elastic and inelastic. Elastic simulator does not change the connectivity of the particles, i.e. springs do not break, and new springs do not appear. Inelastic simulator allows springs to break when they become too long and creates springs when two non-connected particles appear too close.

A spring acts as a physical spring: it exerts the force on particles attached to its ends when stretched or compressed beyond its equilibrium length. Ideal spring obeys a [Hooke's law](https://en.wikipedia.org/wiki/Hooke%27s_law), according to which the force is proportional to deformation: <a href="#"><img src="https://render.githubusercontent.com/render/math?math=F = k\Delta x" alt="F = k dx"></a> (_k_ is a force constant). However, real springs have a limit of compression, and so do the materials. Therefore, in current version, the springs follow the Hooke's law upon stretching and the following handcrafted equation upon compressing:

<p align="center"><a href="#"><img src="https://render.githubusercontent.com/render/math?math=F(x) = \frac{1}{2} k(x - x_0) x_0/x" alt="F = k dx" align="center"></a></p>

Check [simulation details](#simulation-details) to learn how to change force equations and how the removal/addition of springs are implemented in an inelastic simulator.

### Actuators

Actuators are certain types of manipulators that can act on selected particles. Currently two types are supported: heaters and pushers (movable grippers). Typically, each actuator has a defined shape and orientation. This shape is used to capture the particles (to be molten by a heater or grasped by a pusher). Then, each actuator has a defined piecewise linear path which it follows when the simulation begins. This allows a heater to go across the material melting it, or a pusher to grasp and stretch the material. Detailed information can be found in the [class reference](#class-reference).

#### Heater

By default particles are frozen and cannot move. By using a heater (a shape that heats all particles inside it), one can change the state of particles from frozen to molten. This changes the size of the particles, simulating [thermal expansion](https://en.wikipedia.org/wiki/Thermal_expansion), and also allows molten particles to move freely to realize expansion. Molten particles become frozen again after a certain timeout and stop moving.

Slowly moving the heater across a spot of meltable material allows to achieve a flow (process also known as _phase change pumping_), resulting in the transfer of mass from one side of the spot to another (Figure 1). This is how the shape change is achieved. To enable a flow, the global connectivity of the particles needs to be changed. This is realized via removal of too long springs and addition of the springs between the particles that came too close. Therefore, heaters are only useful in an _inelastic_ simulator.

<p align="center">
 <img src="/docs/heater.png" width=40%/>
</p>
<p align="center">
 Figure 1. Application of a heater on a particle network with <i>inelastic simulator</i> (note changes in particle connectivity).
</p>

#### Pusher

Pusher acts as a typical robotic manipulator that grasps a piece of material and drags it. With just one pusher, theoretically, the piece of material should simply be dragged without deformation. However, with the low force constant settings and fewer iterations, the latency in the force transfer through the particle network becomes an important factor (Figure 2a). The simulators do not have any potential fields (like gravity), but the combination of settings like these can emulate a friction with the surface. Higher number of iterations/lower convergence limit makes the simulation more realistic, resulting in an expected behavior with just one pusher - movement of a piece of material without noticeable deformation (Figure 2b).

<p align="center">
 <img src="/docs/single-pusher.png" width="400" />
</p>
<p align="center">
 Figure 2. Different responses based on different number of <a href="#simulator">iterations</a>.
<p>
 
Pushers are more helpful when two or more are used. With several pushers one can stretch the material (Figure 3).

<p align="center">
 <img src="/docs/two-pushers.png" width="600" />
</p>
<p align="center">
 Figure 3. Stretching a rectangular particle network with two pushers.
</p>

### Settings

These are the simulation settings stored in a \*.cfg file that can be supplied to the simulator.

#### Particle
- _Particle default radius_ - default radius of a frozen particle

- _Molten particle default radius_ - default radius of a molten particle (typically larger than frozen one to simulate thermal expansion)

- _Molten particle cooldown time_ - time after which a just-molten particle becomes frozen again; time is measured in ticks (arbitrary time unit = time resolution of the simulation)

####  Spring
- _Spring default stiffness_ - [spring force constant](https://en.wikipedia.org/wiki/Hooke%27s_law)

- _Spring default length_ - equilibrium spring length (i.e., the spring expands if the length is smaller than this value and contracts if the length is larger than this value)

- _Spring connection threshold_ - distance at which two particles become connected with a new spring, relative to the equilibrium spring length (i.e., if the threshold is 0.8, two particles become connected with a spring if they are closer than 80% of the equilibrium spring length)

- _Spring disconnection threshold_ - distance at which two connected particles become disconnected, relative to the equilibrium spring length

#### Simulator

- _Relaxation iteration limit_ - maximum number of iterations that simulator performs

- _Relaxation convergence limit_ - primary stopping criterion for the simulator is "maximum displacement of a particle during iteration is less than convergence limit"

## Repository structure

#### ./builds/
Contains the executables of intermediate stable versions

#### ./configs/

Files containing some simulator settings that, for example, allow to simulate realistic _phase change pumping_ observed in real experiments (each \*.cfg is a separate configuration).

#### ./simulator/backend
Standalone C++ code of simulator. This code is called a non-GUI version.

#### ./simulator/ui
Qt-based UI for the simulator. It is already integrated in the main.cpp, so can be compiled directly, using qmake or Qt Creator.

## For users

### Non-GUI

Non-GUI version of the simulator has a command line interface. The following options are supported:

| Option       | Details               |
|:-------------:|:-------------|
| -i      | input file from which the current state of the shape is read; either a PNG image, or a CSV file with the coordinates of a shape outline, or XML with the simulator state, or XML with the full simulator information (including actuators); see [file formats](#file-formats) below |
| -a      | input XML file(s) with actuators; if the state supplied with -i option is a full simulator state which includes actuators, those actuators will be overwritten |
| -c      | specify command (*simulate* takes the initial states and runs the actuators according to their paths; *predict* suggests the best piecewise-linear move of the first supplied actuator to achieve target shape from current shape) |
| -o      | output file; XML with the final state of the simulator if the *simulate* command is supplied; CSV file with the coordinates of the piecewise-linear move if the *predict* command is supplied |
| -t      | target shape file for *predict* command: a CSV file with the coordinates of a target shape outline. The shape is scaled to the current spot area and translated to match shapes' barycenters. Used only with *predict* command. |
| -s      | settings file; see format below |
| -S      | XML output file for full simulator data to be saved |
| -w      | switch that turns on inelasticity |
| -h      | show help |

### GUI

GUI is mostly self-explanatory. The workflow is as follows. Adjust the simulator settings or load a configuration file. Then initialize the spot as a predefined shape (circle/rectangle) or from an image. Alternatively load an XML file with a simulator or a state. Then add actuators (from file or manually), specify their shapes, draw their paths, and press 'Submit'.

### File formats

#### Input/output

| CSV          | PNG               | XML | XML string |
|:-------------|:-------------|:---------| :----------|
| X,Y<br>point1x,point1y<br>point2x,point2y<br>...<br>pointNx,pointNy | regular PNG format<br>image is first converted to grayscale and then to a 2d array of numbers 0-255 (0 is nothing, non-zero numbers mean there are wax particles in that point) | see examples below | XML string is an XML file content, concatenated into one string. To be able to use it in a command line, all `"` symbols should follow a backslash: `\"`, and entire string must be put inside double quotation marks. |

<details>
 <summary>Actuator XML file</summary>
 
```xml
 <actuator type="Pusher" name="Pusher-2" enabled="false" speed="2" path-advancement="42" orientation="126" spring-crossing-allowed="false" firm-grip="true" final-release="false">
	<shape>
		<point x="0" y="0" />
		<point x="50" y="0" />
		<point x="50" y="10" />
		<point x="0" y="10" />
	</shape>
	<path>
		<point x="-36" y="-20" />
		<point x="-78" y="-20" />
	</path>
</actuator>
```
</details>

<details>
 <summary>Simulator state XML file</summary>
 
```xml
  <state id="0">
  <particles>
  <particle x="0.0" y="0.0" radius="1" id="0" /> 
  <particle x="4.0" y="0.0" radius="1" id="1" /> 
  <particle x="4.4" y="4.4" radius="1" id="2" /> 
  <particle x="0.0" y="4.0" radius="1" id="3" /> 
  </particles>
  <springs>
  <spring particle1id="0" particle2id="1" equilibrium_length="2.0" force_constant="0.01" /> 
  <spring particle1id="1" particle2id="2" equilibrium_length="2.0" force_constant="0.01" /> 
  <spring particle1id="2" particle2id="3" equilibrium_length="2.0" force_constant="0.01" /> 
  <spring particle1id="3" particle2id="0" equilibrium_length="2.0" force_constant="0.01" /> 
  <spring particle1id="1" particle2id="2" equilibrium_length="2.0" force_constant="0.01" /> 
  </springs>
  </state>
```
</details>


<details>
 <summary>Simulator XML file (includes actuators and a state)</summary>
 
```xml
<?xml version="1.0"?>
<simulator type="Simulator" time="17" scale="1">
	<state id="-1">
		<particles>
			<particle x="8.485" y="9.303" radius="1" id="0" />
			<particle x="17.164" y="6.643" radius="1" id="1" />
			<particle x="26" y="5" radius="1" id="2" />
			<particle x="0" y="12.5" radius="1" id="3" />
			<particle x="12.923" y="12.5" radius="1" id="4" />
			<particle x="26" y="12.5" radius="1" id="5" />
			<particle x="8.485" y="15.697" radius="1" id="6" />
			<particle x="17.164" y="18.357" radius="1" id="7" />
			<particle x="26" y="20" radius="1" id="8" />
		</particles>
		<springs>
			<spring particle1id="0" particle2id="1" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="0" particle2id="3" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="2" particle2id="1" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="2" particle2id="5" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="4" particle2id="1" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="4" particle2id="3" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="4" particle2id="5" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="4" particle2id="7" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="5" particle2id="8" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="6" particle2id="3" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="6" particle2id="7" equilibrium_length="5.5" force_constant="0.01" />
			<spring particle1id="7" particle2id="8" equilibrium_length="5.5" force_constant="0.01" />
		</springs>
	</state>
	<actuators>
		<actuator type="Pusher" name="Pusher-1" enabled="false" speed="2" path-advancement="7" orientation="90" spring-crossing-allowed="false" firm-grip="true" final-release="false">
			<shape>
				<point x="0" y="0" />
				<point x="20" y="0" />
				<point x="20" y="10" />
				<point x="0" y="10" />
			</shape>
			<path>
				<point x="20" y="13" />
				<point x="27" y="13" />
			</path>
		</actuator>
		<actuator type="Pusher" name="Pusher-2" enabled="false" speed="2" path-advancement="5" orientation="0" spring-crossing-allowed="false" firm-grip="true" final-release="false">
			<shape>
				<point x="0" y="0" />
				<point x="10" y="0" />
				<point x="10" y="10" />
				<point x="0" y="10" />
			</shape>
			<path>
				<point x="5" y="12" />
				<point x="0" y="12" />
			</path>
		</actuator>
	</actuators>
</simulator>
```
 
 This simulator snapshot corresponds to the result of stretching of a 9-particle network with two pushers.
</details>

#### Settings

A standard INI-file with the following options supported: <details><summary>see format</summary>

```
[Particle]
DefaultRadius=
MoltenDefaultRadius=
CooldownTime=

[Spring]
DefaultStiffness=
DefaultLength=
ConnectionThreshold=
DisconnectionThreshold=

[Relaxation]
IterationLimit=
ConvergenceLimit=

[Actuator]
Speed=

[Heater] (legacy option)
Size=
```
</details>

## For developers
  
### Build instructions

#### non-GUI

First, make sure `make` and `g++` are installed (`sudo apt-get install make g++`). For Windows, use can use [Cygwin](https://www.cygwin.com/). Then run `make` in the root of the repository. This should create an executable in _./builds/nongui_.

#### GUI

On the top of `make` and `g++`, you need to have `Qt` installed, for example, via 'apt-get install qt5-default'. Qt 5.12 is supported, as it is default version for Ubuntu 20.04. Go to _./builds_, run `qmake ../simulator/SpringSystem.pro` and then `make`. This should create an executable in _./builds/release-x64_.

### Simulation details

Simulator uses discrete time. Each simulation step is called a _tick_, and all speeds are specified per tick. First, simulator calculates how many ticks it will take for each actuator to complete its pass. The simulation lasts until the latest actuator finishes its pass. Then, for each tick, all actuators are moved along their respective paths according to their speeds. Finally, processing of particles begins. First step is preprocessing - for example, particles which freeze at this point of time, are marked as frozen but still allowed to move to simulate the contraction of matter upon cooling down. Second step is the processing of the particles by actuators: heaters melt particles they cover, and pushers forcefully move the particles they cover. Third step is so-called relaxation which includes calculation of forces and displacements. In short, the force is calculated for each spring, and the force acting on a particle is a vector sum of all forces from the attached springs. The displacement of a particle is then calculated based on the force. Normally a spring crossing is not allowed, meaning that the displacement is limited in such a way that particles do not cross other springs. After all displacements are calculated, all particles are moved accordingly. In the fourth step the postprocessing of particles takes place: for example, newly frozen particles become immovable. After all actuators finished theirs passes, all particles reset their states: molten particles are cooled down, and (if the release option is set) particles grasped by a pusher are released. After this, the final relaxation is performed, and all actuators are disabled.

The primary difference between elastic and inelastic simulator is the connectivity during the simulation. Inelastic simulator includes an updateConnectivity() method which first checks if there are too long springs. In case there are such springs, the method also checks whether removal of such springs will create large cycles ("holes" inside the material). For each too long spring that cannot be removed without creating a large cycle (typically longer than 3), the algorithm tries to fill this cycle with another, shorter spring. This approach allows to keep the connectivity without creating holes and also to avoid getting too long springs. Second stage of the algorithm checks if there are particles that came close enough to form a spring between them. Since spring crossing is not allowed, it also checks if a new spring will cross any already existing springs.

### Class reference

#### Summary

| Class | Description |
|:-------------:|:-------------|
| Point | Wrapper class for a point in <a href="#"><img src="https://render.githubusercontent.com/render/math?math=\mathbb{R}^2" alt="R2"></a> |
| Path | Wrapper class for an array of points. Path can be cyclic. Method sampleFraction() samples a point on the path such that cumulative length from the beginning of the path to this point is a certain fraction of the entire path length. |
| Shape | Wrapper class for a polygon. Includes useful methods, such as area, perimeter, center etc. Can be translated, scaled, rotated. The distance between two Shapes is calculated by sampling more points on the boundary and then using the Hausdorff or a modified Procrustes metric. |
| Particle | Apart from coordinates, it contains radius, array of attached springs, and several parameters related to simulation: displacement (on each iteration, displacement for all particles for the next step are calculated, and only then applied simultaneously to all particles); time when molten particle should become frozen again; whether particle should be moved during simulation.  |
| ParticleState | Stores the crucial \[saveable\] information about a particle: coordinates and radius |
| Spring | Stores pointers to the particles which the spring connects, as well as its force constant (stiffness) and its equilibrium length. Force equation is calculated in this class. |
| SpringState | Stores the crucial \[saveable\] information about a spring: pointers of the corresponding particle states, force constant and equilibrium length |
| Actuator | Actuator has a shape, orientation (in radians), a planned path, current progress along this path (path advancement), and a number of settings, such as speed (movement per tick), enabled/disabled switch, and operation switches - whether to release the grip after the movement is completed, whether to hold a firm grip or just move and capture particles that are currently in the area of actuator, and some other. By default the particles inside the area of the actuator shape are captured, but if needed, a setCaptureFunction() method can be used to override this behavior. |
| Heater | Inherits Actuator. Implements the handling of particles, such as melting/freezing them. |
| Pusher | Inherits Actuator. Implements the handling of particles, such as moving capture particles with the gripper. |
| SimulatorSettings | Wrapper class for all settings used by a simulator. Stores the settings loaded from \*.cfg files. |
| SpringSimulator | The primary class for simulation. Contains the set of particles (connectivity is stored in the particles' spring arrays) and actuators. |
| SpringSimulatorState | Stores the crucial information about the state of simulator, namely, positions of particles and their connectivity through an array of ParticleState objects. Useful for display purposes. A SpringSimulator instance can read a state and load it. |
| ElasticSimulator | Same as SpringSimulator. |
| InelasticSimulator | Implements the connectivity changes in the particle network. Same as WaxSimulator. |
