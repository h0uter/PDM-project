import numpy as np
import time
import math

class DroneHitbox:

    def __init__(self, s0, r):
        self.s = np.asarray(s0[:3]) # starting location of drone
        self.r = r # radius of drone hitbox

    def update(self, s):
        self.s = s[:3]

class CollisionDetector:

    def __init__(self, safety_margin):
        self.sphere_collision_point = []
        self.polygon_collision_point = []
        self.safety_margin = safety_margin
        self.drone_vector = [] # list containing direction and origin of drone vector respectively

    def check_collision(self, dronehitbox, spheres, prism_array, beam_array, drone_vector_points):
        collision = False
        self.drone_vector.append((drone_vector_points[1] - drone_vector_points[0]) /
        np.linalg.norm((drone_vector_points[1] - drone_vector_points[0]))) # normalized direction vector
        self.drone_vector.append(drone_vector_points[0]) # origin of vector
        self.drone_vector.append(np.linalg.norm((drone_vector_points[1] - drone_vector_points[0]))) # magnitude of direction vector

        detection_radius = 4
        time1 = time.perf_counter()
        if self.sphere_collision_detector(dronehitbox, spheres) or self.figure_collision_detector(dronehitbox, detection_radius, prism_array, beam_array):
            time2       = time.perf_counter()
            time_taken  = time2 - time1
            print(f"Detecting collsions took {time_taken} seconds.")
            self.drone_vector = []
            return True

        time2       = time.perf_counter()
        time_taken  = time2 - time1
        print(f"Detecting collsions took {time_taken} seconds.")
        self.drone_vector = []
        return collision

    def sphere_collision_detector(self, dronehitbox, spheres):
        sphere_collision = False
        if spheres == []:
            pass
        else:
            for sphere in spheres:

                origin_to_center    = sphere.pos - self.drone_vector[1] # distance from origin drone vector to center sphere
                d_center            = np.dot(origin_to_center, self.drone_vector[0]) # distance from center projected to drone vector
                if d_center < 0: continue

                center_to_line_sq   = abs((d_center ** 2) - np.linalg.norm(origin_to_center)**2) # smallest distance from center to line squared

                r_sq                = (sphere.r + dronehitbox.r + self.safety_margin) ** 2

                if center_to_line_sq > r_sq: continue

                center_to_intsect   = math.sqrt(r_sq - center_to_line_sq)
                t_point             = d_center - center_to_intsect # t of first intersection point

                if t_point < 1*self.drone_vector[2]:
                    self.sphere_collision_point = (self.drone_vector[1] + self.drone_vector[0] * t_point).tolist()

                return True

        return sphere_collision

    def figure_collision_detector(self, dronehitbox, detection_radius, prism_array, beam_array):
        if prism_array == []:
            pass

        else:
            for prism in prism_array:
                if np.linalg.norm(dronehitbox.s - prism.center) < detection_radius:
                    if self.polygon_collision_detector(dronehitbox, prism.polygons_col_array):
                        return True

        if beam_array == []:
            pass

        else:
            for beam in beam_array:
                if np.linalg.norm(dronehitbox.s - beam.center) < detection_radius:
                    if self.polygon_collision_detector(dronehitbox, beam.polygons_col_array):
                        return True

        return False


    def polygon_collision_detector(self, dronehitbox, polygons):
        polygon_collision = False

        if polygons == []:
            pass

        else:
            for i, polygon in enumerate(polygons):
                edges = polygon.polygon_edges()
                normal_plane = np.cross(edges[0], edges[2])

                norm_of_normal = normal_plane / np.linalg.norm(normal_plane)

                # check if drone vector and polygon are parallel
                line_on_normal = np.dot(normal_plane, self.drone_vector[0])
                if np.isclose(abs(line_on_normal), 0): continue

                d = np.dot(normal_plane, polygon.points[0])

                t = (np.dot(normal_plane, self.drone_vector[1]) + d) / line_on_normal
                if t < 0 or t > 1*self.drone_vector[2]: continue

                elif self.polygon_surface_collision(dronehitbox, polygon, edges, normal_plane, t):

                    self.polygon_collision_point = (self.drone_vector[1] + self.drone_vector[0] * t).tolist()
                    print("surface")
                    return True

        return polygon_collision

    def polygon_surface_collision(self, dronehitbox, polygon, edges, normal_plane, t):
        inside_polygon = False

        norm_of_normal = normal_plane / np.linalg.norm(normal_plane)

        if self.point_inside_polygon(edges, normal_plane, polygon, norm_of_normal, dronehitbox, t):
            return True

        return inside_polygon

    def point_inside_polygon(self, edges, normal_plane, polygon, norm_of_normal, dronehitbox, t):

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
