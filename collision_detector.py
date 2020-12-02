import numpy as np
import time

class DroneHitbox:

    def __init__(self, s0, r=0.2):
        self.s = np.asarray(s0[:3]) # starting location of drone
        self.r = r # radius of drone hitbox

    def update(self, s):
        self.s = s[:3]

class CollisionDetector:

    def __init__(self):
        pass

    def check_collision(self, dronehitbox, spheres, polygons, polygons_edges):
        collision = False
        detection_radius = 1
        time1 = time.perf_counter()
        if self.sphere_collision_detector(dronehitbox, spheres) or self.polygon_collision_detector(dronehitbox, polygons, polygons_edges, detection_radius):
            time2 = time.perf_counter()
            time_taken = time2 - time1
            print(f"Detecting collsions took {time_taken} seconds.")
            return True

        time2 = time.perf_counter()
        time_taken = time2 - time1
        print(f"Detecting collsions took {time_taken} seconds.")
        return collision

    def sphere_collision_detector(self, dronehitbox, spheres):
        sphere_collision = False
        if spheres == []:
            pass
        else:
            for sphere in spheres:

                if np.linalg.norm(dronehitbox.s - sphere.pos) < (dronehitbox.r + sphere.r): # checks for collision using euclidian distance
                    return True

        return sphere_collision

    def polygon_collision_detector(self, dronehitbox, polygons, polygons_edges, detection_radius):
        polygon_collision = False
        if polygons == []:
            pass
        else:
            for i, polygon in enumerate(polygons):

                if (np.linalg.norm(dronehitbox.s - polygon.center)) > detection_radius:
                    polygon_collision = False

                elif self.polygon_edges_collision(dronehitbox, polygon):
                    return True

                elif self.polygon_surface_collision(dronehitbox, polygon):
                    return True

        return polygon_collision

    def polygon_edges_collision(self, dronehitbox, polygon):
        edges_collision = False
        for j, edge in enumerate(polygon.polygon_edges()):
            # calculating vector between hitbox and vertix of the polygon edge
            circle_vector_x = dronehitbox.s[0] - polygon.xpoints[j]
            circle_vector_y = dronehitbox.s[1] - polygon.ypoints[j]
            circle_vector_z = dronehitbox.s[2] - polygon.zpoints[j]
            circle_vector = np.add([circle_vector_x, circle_vector_y, circle_vector_z], polygon.pos)

            len_edge = np.linalg.norm(np.asarray(edge))
            norm_edge = np.asarray(edge) / len_edge

            # projecting the circle vector onto the edge
            dot_product = np.dot(np.asarray(circle_vector), norm_edge)

            if dot_product <= 0: # dronehitbox is closest to start of vector
                distance_vector = np.add([polygon.xpoints[j], polygon.ypoints[j], polygon.zpoints[j]], polygon.pos)

            elif dot_product >= len_edge: # dronehitbox is closest to end of vector
                distance_vector = np.add([polygon.xpoints[j-1], polygon.ypoints[j-1], polygon.zpoints[j-1]], polygon.pos)

            else: # dronehitbox is in between
                point_on_line = norm_edge * dot_product
                distance_vector =   [point_on_line[0] + polygon.xpoints[j],
                                     point_on_line[1] + polygon.ypoints[j],
                                     point_on_line[2] + polygon.zpoints[j]] + polygon.pos

            if np.linalg.norm(dronehitbox.s - distance_vector) < dronehitbox.r:
                return True

        return edges_collision

    def polygon_surface_collision(self, dronehitbox, polygon):
        intsect_plane = False
        inside_polygon = False

        edges = polygon.polygon_edges()
        normal_plane = np.cross(edges[0], edges[1])
        norm_of_normal = normal_plane / np.linalg.norm(normal_plane)

        D_plane = -normal_plane[0] * (polygon.xpoints[0] + polygon.pos[0]) - normal_plane[1] * (polygon.ypoints[0] + polygon.pos[1]) - normal_plane[2] * (polygon.zpoints[0] + polygon.pos[2])

        distance_to_plane = (normal_plane[0] * dronehitbox.s[0] + normal_plane[1] * dronehitbox.s[1] + normal_plane[2] * dronehitbox.s[2] + D_plane) / np.linalg.norm(normal_plane)

        if distance_to_plane <= dronehitbox.r:
            intsect_plane = True

        else:
            return False

        if self.projected_inside_plane(edges, normal_plane, polygon, norm_of_normal, dronehitbox):
            inside_polygon = True

        return (intsect_plane and inside_polygon)

    def projected_inside_plane(self, edges, normal_plane, polygon, norm_of_normal, dronehitbox):
            # projecting circle centre point to polygon plane
            X_of_plane = edges[0] / np.linalg.norm(normal_plane)
            Y_of_plane = np.cross(norm_of_normal, edges[0]) / np.linalg.norm(np.cross(norm_of_normal, edges[0]))

            projected_drone_center = [np.dot(np.asarray(dronehitbox.s), X_of_plane), np.dot(np.asarray(dronehitbox.s), Y_of_plane)]
            projected_polygon_points =  [[np.dot((polygon.points[0] + polygon.pos), X_of_plane), np.dot((polygon.points[0] + polygon.pos), Y_of_plane)],
                                         [np.dot((polygon.points[1] + polygon.pos), X_of_plane), np.dot((polygon.points[1] + polygon.pos), Y_of_plane)],
                                         [np.dot((polygon.points[2] + polygon.pos), X_of_plane), np.dot((polygon.points[2] + polygon.pos), Y_of_plane)]]

            total_area = self.triangle_area(projected_polygon_points[0], projected_polygon_points[1], projected_polygon_points[2])
            area1 = self.triangle_area(projected_drone_center, projected_polygon_points[0], projected_polygon_points[1])
            area2 = self.triangle_area(projected_drone_center, projected_polygon_points[1], projected_polygon_points[2])
            area3 = self.triangle_area(projected_drone_center, projected_polygon_points[0], projected_polygon_points[2])

            if (area1 + area2 + area3) > total_area:
                return False

            else:
                return True

    def triangle_area(self, p1, p2, p3):
        # uses 1/2 determinant method to calculate area of triangle
        det = ((p1[0] - p3[0]) * (p2[1] - p3[1])) - ((p2[0] - p3[0]) * (p1[1] - p3[1]))

        return (0.5 * det)
