from os import system
from math import pi, sin, cos
from simulator_interface import Point, ParticleMesh, Manipulator

# Creates a particle mesh in a rectangle 100x70 centered at (0, 0)
# with particle radii 1 and spring lengths 5
cloth = ParticleMesh(Point(0, 0),
                     100, 70,
                     1, 5)

# Initially all force constants are 0, so set the values here
for spring in cloth.springs:
    spring.force_constant = 0.02

# For example, make all springs attached to particle with id=50 stiffer
# To be safe, try to keep force constant small, say, below 1
for spring in cloth.springs:
    if 50 in [spring.id1, spring.id2]:
        spring.force_constant = 0.2

# Write initial state to file
cloth.save_to_file("initial.xml")

# Create a manipulator with the shape of a vertical rectangle 10x80
hand1 = Manipulator(shape = [Point(0, 0),
                             Point(0, 80),
                             Point(10, 80),
                             Point(10, 0)])

# Set a path (in this case just make it stand still at the left edge)
hand1.path = [Point(-48, 0)]
hand1.speed = 0
hand1.save_to_file("hand1.xml")

# Create another manipulator
hand2 = Manipulator(shape = [Point(0, 0),
                             Point(0, 10),
                             Point(10, 10),
                             Point(10, 0)])

# Rotate by 30 degrees, if you want
hand2.orientation = 30

# Higher speed makes simulation faster -- less intermediate states
# in extreme cases possibly involving self-penetration it is better to keep it low
hand2.speed = 2

# Try pulling the top right corner in different directions
for angle in range(0, 180, 30):
    radians = angle / 180 * pi

    # top right with some margin to capture just one particle
    x = 39
    y = 32
    distance = 50
    hand2.path = [Point(x, y),
                  Point(x + distance * sin(radians),
                        y + distance * cos(radians))]
    hand2.save_to_file("hand2.xml")
    
    # run the simulator
    system("./../builds/nongui/springsim -s stretching.cfg -c simulate"\
           " -a hand1.xml -a hand2.xml"\
           " -i initial.xml -o angle%d.xml" % angle)
    # if you want to save full simulator output, including actuators,
    # add -S parameter with a file name
