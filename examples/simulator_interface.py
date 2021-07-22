from xml.etree import ElementTree
from math import sqrt

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Particle:
    def __init__(self, x, y, radius, id_):
        self.x = x
        self.y = y
        self.r = radius
        self.id_ = id_

class Spring:
    def __init__(self, particle1_id, particle2_id,
                 equilibrium_length,
                 force_constant = 0):
        self.id1 = particle1_id
        self.id2 = particle2_id
        self.equilibrium_length = equilibrium_length
        self.force_constant = force_constant



class ParticleMesh:
    """Collection of particles and springs
    """

    def __init__(self, centre, width, height, particle_size, spring_size,
                 include_point = None):
        """Creates a particle network inside a bounding box, based on the lambda
        mask

        Args:
            centre:
                Centre of a bounding box
            width:
                Bounding box width
            height:
                Bounding box height
            particle_size:
                Particle radius
            spring_size:
                Spring length
            include_point:
                Lambda function with x, y coordinates as arguments, returning
                whether this point should be included in the mask;
                by default any point in bounding box is included
        """
        self.particles = []
        self.springs = []
        if include_point == None:
            include_point = lambda x, y: abs(x - centre.x) < width / 2 and\
                                         abs(y - centre.y) < height / 2

        x_interval = particle_size * 2 + spring_size
        y_interval = x_interval * sqrt(3) / 2
        
        size_x = int((width / 2 - x_interval / 2) / x_interval)
        size_y = int((height / 2 - x_interval / 2) / y_interval)
        if size_x <= 0 or size_y <= 0:
            return False

        grid = [[None] * (2 * size_x + 1) for _ in range(2 * size_y + 1)]

        for i in range(-size_y, size_y + 1):
            for j in range(-size_x, size_x + 1):
                x = centre.x + j * x_interval
                if i & 1:
                    x -= x_interval / 2
                y = centre.y + i * y_interval 
                if include_point(x, y):
                    grid[i + size_y][j + size_x] = Particle(x, y, particle_size,
                                                            len(self.particles))
                    self.particles.append(grid[i + size_y][j + size_x])

        for i in range(2 * size_y + 1):
            for j in range(2 * size_x + 1):
                if grid[i][j]:
                    if j > 0:
                        self.springs.append(Spring(grid[i][j].id_,
                                                   grid[i][j - 1].id_,
                                                   spring_size))
                    if i > 0:
                        self.springs.append(Spring(grid[i][j].id_,
                                                   grid[i - 1][j].id_,
                                                   spring_size))
                        if (i - size_y) & 1:
                            if j > 0:
                                self.springs.append(Spring(
                                                        grid[i][j].id_,
                                                        grid[i - 1][j - 1].id_,
                                                        spring_size))
                        else:
                            if j < 2 * size_x:
                                self.springs.append(Spring(
                                                        grid[i][j].id_,
                                                        grid[i - 1][j + 1].id_,
                                                        spring_size))

    def to_xml(self):
        root = ElementTree.Element("state")
        particles_node =  ElementTree.SubElement(root, "particles")
        for particle in self.particles:
            ElementTree.SubElement(particles_node,
                                   "particle",
                                   x = str(particle.x),
                                   y = str(particle.y),
                                   radius = str(particle.r),
                                   id = str(particle.id_))
        springs_node =  ElementTree.SubElement(root, "springs")
        for spring in self.springs:
            ElementTree.SubElement(springs_node,
                                   "spring",
                                   particle1id = str(spring.id1),
                                   particle2id = str(spring.id2),
                                   force_constant = str(spring.force_constant),
                                   equilibrium_length = str(spring.equilibrium_length))
        return ElementTree.ElementTree(root)

    def to_string(self):
        xml = self.to_xml()
        return ElementTree.to_string(xml)

    def save_to_file(self, filename):
        xml = self.to_xml()
        xml.write(filename)

class Manipulator:
    def __init__(self, shape = [], path = [], orientation = 0, speed = 1.0):
        self.shape = shape
        self.path = path
        self.orientation = orientation
        self.speed = speed

    def to_xml(self):
        root = ElementTree.Element("actuator",
                                   type = "Pusher",
                                   enabled = "true",
                                   spring_crossing_allowed = "false",
                                   orientation = str(self.orientation),
                                   speed = str(self.speed))
        shape_node =  ElementTree.SubElement(root, "shape")
        for point in self.shape:
            ElementTree.SubElement(shape_node,
                                   "point",
                                   x = str(point.x),
                                   y = str(point.y))
        path_node =  ElementTree.SubElement(root, "path")
        for point in self.path:
            ElementTree.SubElement(path_node,
                                   "point",
                                   x = str(point.x),
                                   y = str(point.y))
        return ElementTree.ElementTree(root)

    def save_to_file(self, filename):
        xml = self.to_xml()
        xml.write(filename)
