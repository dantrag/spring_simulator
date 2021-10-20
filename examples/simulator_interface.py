from xml.etree import ElementTree
from math import sqrt, sin, cos, radians

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

    def point(self):
        return Point(self.x, self.y)

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
            return None

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

    def load_from_xml(self, filename):
        xml = ElementTree.parse(filename)
        root = xml.getroot()
        particles_node = root.find("particles")
        self.particles = []
        for particle_node in particles_node.iter("particle"):
            x = float(particle_node.get("x"))
            y = float(particle_node.get("y"))
            radius = float(particle_node.get("radius"))
            id = int(particle_node.get("id"))
            self.particles.append(Particle(x, y, radius, id))

        self.particles.sort(key=lambda particle : particle.id_)
        for i in range(len(self.particles)):
            if self.particles[i].id_ != i:
                print("Error: Incomplete set of particle id!")
                self.particles = []
                return

        springs_node = root.find("springs")
        self.springs = []
        for spring_node in springs_node.iter("spring"):
            id1 = int(spring_node.get("particle1id"))
            id2 = int(spring_node.get("particle2id"))
            length = float(spring_node.get("equilibrium_length"))
            force_constant = float(spring_node.get("force_constant"))
            self.springs.append(Spring(id1, id2, length, force_constant))
    
    def force_on_particle(self, particle_id):
        force_x = 0.0
        force_y = 0.0

        for spring in self.springs:
            if particle_id in [spring.id1, spring.id2]:
                other_id = spring.id1 + spring.id2 - particle_id
                dx = self.particles[other_id].x - self.particles[particle_id].x
                dy = self.particles[other_id].y - self.particles[particle_id].y
                distance = sqrt(dx * dx + dy * dy)
                dx /= distance
                dy /= distance

                # NOTE: this formula is copied from the current simulator code
                # and might change in future!
                force = 0.0
                length = distance - self.particles[particle_id].r \
                                  - self.particles[other_id].r
                if length < spring.equilibrium_length:
                    force = (1.0 / length - 1.0 / spring.equilibrium_length) *\
                            spring.equilibrium_length ** 2 *\
                            spring.force_constant / 2
                else:
                    force = (spring.equilibrium_length - length) *\
                            spring.force_constant

                force_x -= dx * force
                force_y -= dy * force

        return (force_x, force_y)

    def to_string(self):
        xml = self.to_xml()
        return ElementTree.to_string(xml)

    def save_to_file(self, filename):
        xml = self.to_xml()
        xml.write(filename)

class Manipulator:
    def __init__(self, shape = [], path = [], orientation = 0, speed = 1.0,
                 force_limited = False, force = 0.0):
        self.shape = shape
        self.path = path
        self.orientation = orientation
        self.speed = speed
        self.force_limited = force_limited
        self.force = force

    def contains_point(self, point: Point, position: Point):
        def cross_product(p1: Point, p2: Point, p3: Point):
            return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)

        #centre, rotate and translate the shape
        shape = [Point(p.x, p.y) for p in self.shape]
        centre_x = 0.0
        centre_y = 0.0
        for i in range(len(shape)):
            centre_x += shape[i].x
            centre_y += shape[i].y
        centre_x /= len(shape)
        centre_y /= len(shape)

        for i in range(len(shape)):
            shape[i].x -= centre_x
            shape[i].y -= centre_y
            x = shape[i].x
            y = shape[i].y
            angle = radians(self.orientation)
            shape[i].x = x * cos(angle) - y * sin(angle) + position.x
            shape[i].y = x * sin(angle) + y * cos(angle) + position.y

        winding_number = 0
        id_list = list(range(len(shape)))
        for i, j in zip(id_list, id_list[1:] + id_list[:1]):
            current = shape[i]
            next = shape[j]

            if current.y <= point.y:
                if next.y > point.y:
                    if cross_product(current, next, point) > 1e-5:
                        winding_number += 1
            else:
                if next.y <= point.y:
                    if cross_product(current, next, point) < -1e-5:
                        winding_number -= 1

        return winding_number != 0

    def to_xml(self):
        root = ElementTree.Element("actuator",
                                   type = "Pusher",
                                   enabled = "true",
                                   spring_crossing_allowed = "false",
                                   force_limited = str(self.force_limited),
                                   force_limit = str(self.force),
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
