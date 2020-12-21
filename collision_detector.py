import numpy as np
import math

class CollisionDetector:

    def __init__(self, safety_margin, sphere_array, prism_array, beam_array, dronehitbox_r):
        self.sphere_collision_point = []
        self.polygon_collision_point = []
        self.spheres = sphere_array
        self.prisms = prism_array
        self.beams = beam_array
        self.safety_margin = safety_margin
        self.dronehitbox_r = dronehitbox_r
        self.drone_vector = [] # list containing direction and origin of drone vector respectively

    def update(self, drone_vector_array):
        n_segments = len(drone_vector_array) - 1
        line_segment_array = []

        for i in range(n_segments):

            line_segment_array.append([drone_vector_array[i], drone_vector_array[i+1]])

        for line_segment in line_segment_array:

            if self.check_collision(self.spheres, self.prisms, self.beams, line_segment):
                return True

        return False

    def check_collision(self, spheres, prism_array, beam_array, drone_vector_points):

        self.drone_vector.append((drone_vector_points[1] - drone_vector_points[0]) /
        np.linalg.norm((drone_vector_points[1] - drone_vector_points[0]))) # normalized direction vector
        self.drone_vector.append(drone_vector_points[0]) # origin of vector
        self.drone_vector.append(np.linalg.norm((drone_vector_points[1] - drone_vector_points[0]))) # magnitude of direction vector

        if self.sphere_collision_detector(self.dronehitbox_r, self.spheres) or self.figure_collision_detector(self.dronehitbox_r, self.prisms, self.beams):
            self.drone_vector = []
            return True

        self.drone_vector = []
        return False

    def sphere_collision_detector(self):

        for sphere in self.spheres:

            origin_to_center    = sphere.pos - self.drone_vector[1] # distance from origin drone vector to center sphere
            d_center            = np.dot(origin_to_center, self.drone_vector[0]) # distance from center projected to drone vector

            center_to_line_sq   = abs((d_center ** 2) - np.linalg.norm(origin_to_center)**2) # smallest distance from center to line squared

            r_sq                = (sphere.r + self.dronehitbox_r + self.safety_margin) ** 2

            if center_to_line_sq > r_sq: continue

            return True

        return False

    def figure_collision_detector(self):

        for prism in self.prisms:
            if self.polygon_collision_detector(prism.polygons_col_array):
                return True

        for beam in self.beams:
            if self.polygon_collision_detector(beam.polygons_col_array):
                return True

        return False


    def polygon_collision_detector(self, polygons):

        for polygon in polygons:
            edges = polygon.polygon_edges()
            normal_plane = np.cross(edges[0], edges[2])

            # check if drone vector and polygon are parallel
            line_on_normal = np.dot(normal_plane, self.drone_vector[0])
            if np.isclose(abs(line_on_normal), 0): continue

            d = np.dot(normal_plane, polygon.points[0])

            t = (np.dot(normal_plane, self.drone_vector[1]) + d) / line_on_normal
            # check if collision happens behind or in front of line segment
            if t < 0 or t > self.drone_vector[2]: continue

            elif self.polygon_surface_collision(polygon, edges, normal_plane, t):

                return True

        return False

    def polygon_surface_collision(self, polygon, edges, normal_plane, t):

        if self.point_inside_polygon(edges, normal_plane, polygon, t):
            return True

        return False

    def point_inside_polygon(self, edges, normal_plane, polygon, t):

        collision_point = self.drone_vector[1] + self.drone_vector[0] * t

        # edge 0
        vp0 = collision_point - polygon.points[0]
        C = np.cross(edges[0], vp0) / np.linalg.norm(np.cross(edges[0], vp0))
        if np.dot(normal_plane, C) < 0: return False

        # edge 1
        vp1 = collision_point - polygon.points[1]
        C = np.cross(edges[1], vp1) / np.linalg.norm(np.cross(edges[1], vp1))
        if np.dot(normal_plane, C) < 0: return False

        # edge 2
        vp2 = collision_point - polygon.points[2]
        edge_2 = polygon.points[0] - polygon.points[2]
        C = np.cross(edge_2, vp2) / np.linalg.norm(np.cross(edges[2], vp2))
        if np.dot(normal_plane, C) < 0: return False

        return True
