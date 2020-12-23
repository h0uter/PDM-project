import numpy as np
import config as cfg

class Ellipse:

    def __init__(self, start, end, domain):
        self.start = start
        self.end = end
        self.walls = Walls(domain=domain)

        self.a = np.linalg.norm(end - start) / 2 + cfg.dronehitbox_r
        self.r = None
        self.transformation_matrix = None
        self.origin = (end - start) / 2 + start

        primary_axis = end - start
        self.primary_axis = primary_axis / np.linalg.norm(primary_axis)

        np.random.seed(42)

    def update(self, longest_dis, secondary_axis):
        #define maxium radius, define local axes, and transformation matrix using these axes and origin
        self.r = longest_dis + cfg.dronehitbox_r
        self.secondary_axis = secondary_axis
        self.tertiary_axis = np.cross(self.primary_axis, self.secondary_axis)
        self.transformation_matrix = np.zeros((4,4))

        rotation_matrix = np.column_stack((self.primary_axis, 
                                           self.secondary_axis, 
                                           self.tertiary_axis))

        self.transformation_matrix[:-1, :-1] = rotation_matrix
        self.transformation_matrix[:-1, -1] = self.origin
        self.transformation_matrix[-1, -1] = 1.

        print(f"new ellipse defined, radius is {self.r}")

    def define_ellipse(self, path):
        #vector along which ellipse exists
        B = self.end - self.start

        longest_dis = 0
        secondary_axis = None

        for point in path:
            A = point.pos - self.start
            longitudinal_vector = (np.dot(A, B) / np.dot(B, B)) * B
            point_to_line = A - longitudinal_vector
            local_radius = np.linalg.norm(point_to_line)

            longitudinal_vector_mag = np.linalg.norm(longitudinal_vector)
            x_coor = longitudinal_vector_mag - np.linalg.norm(B) / 2
            radius = local_radius / np.sqrt(1 - x_coor**2 / self.a**2)

            #find what the minimal radius would need to be to include all points
            if radius > longest_dis:
                longest_dis = radius
                secondary_axis = point_to_line / local_radius
        
        if self.r is None:
            self.update(longest_dis, secondary_axis)
        elif self.r - cfg.dronehitbox_r > longest_dis:
            self.update(longest_dis, secondary_axis)

    def get_random_point(self):
        #get random position using cylindrical coordinates
        x = np.random.uniform(-self.a, self.a)
        phi = np.random.uniform(0, 2*np.pi)

        #find which is shorter, the local radius of the ellipse or the minimum distance to hit a 'domain wall'
        local_radius_ellipse = np.sqrt(1 - x**2 / self.a**2) * self.r

        global_origin = np.dot(self.transformation_matrix, np.asarray([x, 0., 0., 1]))[:-1]
        unit_dir = np.dot(self.transformation_matrix[:-1, :-1], np.asarray([np.cos(phi), np.sin(phi), 0.]))
        local_radius_domain = self.walls.get_minimum_distance(line_origin=global_origin, unit_dir=unit_dir)

        r = np.random.uniform(0, min(local_radius_ellipse, local_radius_domain))

        #convert to cartesian
        y, z = np.cos(phi) * r, np.sin(phi) * r

        #transform to global coordinates using transformation matrix
        global_vector = np.dot(self.transformation_matrix, np.asarray([x, y, z, 1]))[:-1]
        return global_vector
    
    def ellipse_exists(self):
        return self.r is not None

#make sure that created ellipse does not contain sample space that is not in the original space, ie is a subset
class Walls:

    def __init__(self, domain):
        x_domain, y_domain, z_domain = domain
        self.walls = []

        self.walls.append(Wall(normal_vector=np.asarray([0, 0, 1]), p0=np.asarray([0, 0, z_domain[0] + cfg.dronehitbox_r]))) #floor
        self.walls.append(Wall(normal_vector=np.asarray([0, 0, 1]), p0=np.asarray([0, 0, z_domain[1] - cfg.dronehitbox_r]))) #roof
        self.walls.append(Wall(normal_vector=np.asarray([0, 1, 0]), p0=np.asarray([0, y_domain[0] + cfg.dronehitbox_r, 0]))) #wall #1
        self.walls.append(Wall(normal_vector=np.asarray([0, 1, 0]), p0=np.asarray([0, y_domain[1] - cfg.dronehitbox_r, 0]))) #wall #2
        self.walls.append(Wall(normal_vector=np.asarray([1, 0, 0]), p0=np.asarray([x_domain[0] + cfg.dronehitbox_r, 0, 0]))) #wall #3
        self.walls.append(Wall(normal_vector=np.asarray([1, 0, 0]), p0=np.asarray([x_domain[1] - cfg.dronehitbox_r, 0, 0]))) #wall #4

    def get_minimum_distance(self, line_origin, unit_dir):
        min_dis = 1e6
        for wall in self.walls:
            d = wall.get_distance(line_origin, unit_dir)
            if d is not None:
                if d < min_dis and d >= 0.:
                    min_dis = d
        
        return min_dis

class Wall:

    def __init__(self, normal_vector, p0):
        self.normal_vector = normal_vector
        self.p0 = p0
    
    def get_distance(self, line_origin, unit_dir):
        numerator = np.dot(self.p0 - line_origin, self.normal_vector)
        denominator = np.dot(unit_dir, self.normal_vector)

        if denominator == 0:
            return None
        
        elif numerator == 0:
            return 0.0
        
        else:
            return numerator / denominator