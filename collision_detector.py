import numpy as np

class DroneHitbox:

    def __init__(self, s0, r=0.2):
        self.s = np.asarray(s0[:3]) # starting location of drone
        self.r = r # radius of drone hitbox

    def update(self, s):
        self.s = s[:3]

class CollisionDetector:

    def __init__(self):
        pass

    def check_collision(self, dronehitbox, spheres, polygons, polygons_vertices):
        collision = False
        if self.sphere_collision_detector(dronehitbox, spheres) or self.polygon_collision_detector(dronehitbox, polygons, polygons_vertices):
            collision = True

        return collision

    def sphere_collision_detector(self, dronehitbox, spheres):
        collision = False
        if spheres == []:
            pass
        else:
            for sphere in spheres:
                if np.linalg.norm(dronehitbox.s - sphere.pos) < (dronehitbox.r + sphere.r): # checks for collision using euclidian distance
                    collision = True
        return collision

    def polygon_collision_detector(self, dronehitbox, polygons, polygons_vertices):
        collision = False
        if polygons == []:
            pass
        else:
            for i, polygon in enumerate(polygons):
                if self.polygon_vertices_collision(dronehitbox, polygon, i):
                    collision = True
        return collision

    def polygon_vertices_collision(self, dronehitbox, polygon, i):
        collision = False
        for j, vertix in enumerate(polygon.polygon_vertices()):
            circle_vector_x = dronehitbox.s[0] - polygon.points[0][j]
            circle_vector_y = dronehitbox.s[1] - polygon.points[1][j]
            circle_vector_z = dronehitbox.s[2] - polygon.points[2][j]
            circle_vector = [circle_vector_x, circle_vector_y, circle_vector_z]

            len_vertix = np.linalg.norm(np.asarray(vertix))
            norm_vertix = np.asarray(vertix) / len_vertix

            dot_product = np.dot(np.asarray(circle_vector), norm_vertix)

            if dot_product <= 0: # dronehitbox is closest to start of vector
                distance_vector = [polygon.points[0][j], polygon.points[1][j], polygon.points[2][j]]

            elif dot_product >= len_vertix: # dronehitbox is closest to end of vector
                distance_vector = [polygon.points[0][j-1], polygon.points[1][j-1], polygon.points[2][j-1]]

            else: # dronehitbox is in between
                point_on_line = norm_vertix * dot_product
                distance_vector = [point_on_line[0] + polygon.points[0][j], point_on_line[1] + polygon.points[1][j], point_on_line[2] + polygon.points[2][j]]

            if np.linalg.norm(dronehitbox.s - distance_vector) < dronehitbox.r:
                collision = True

        return collision
