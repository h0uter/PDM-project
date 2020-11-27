import numpy as np
import matplotlib.pyplot as plt

class Sphere:

    def __init__(self, r, pos):
        self.r = r #radius of sphere
        self.pos = pos #position of sphere (x, y, z)

    def create_sphere(self):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]

        x = self.r * np.cos(u) * np.sin(v) + self.pos[0]
        y = self.r * np.sin(u) * np.sin(v) + self.pos[1]
        z = self.r * np.cos(v) + self.pos[2]

        return x, y, z

class Polygon:

    def __init__(self, points, pos):
        self.xpoints = points[0] #array with x-coordinates of polygon points
        self.ypoints = points[1] #array with y-coordinates of polygon points
        self.zpoints = points[2] #array with z-coordinates of polygon points
        self.points = points
        self.pos = pos #position of polygon

    def create_polygon(self):
        x = (np.asarray(self.xpoints) + self.pos[0]).tolist()
        y = (np.asarray(self.ypoints) + self.pos[1]).tolist()
        z = (np.asarray(self.zpoints) + self.pos[2]).tolist()
        verts = [list(zip(x, y, z))]

        return verts

    def polygon_vertices(self):
        vertices = []
        for i, points in enumerate(self.points):
            x = self.points[0][i-1] - self.points[0][i]
            y = self.points[1][i-1] - self.points[1][i]
            z = self.points[2][i-1] - self.points[2][i]
            vertices.append([x, y ,z])

        return vertices


class PolygonManager:

    def __init__(self, n_polygons, points_array, pos_array):
        self.n_polygons = n_polygons
        self.points_array = points_array #array of polygon points ([[[x_array_1], [y_array_1], [z_array_1]], [[x_array_2], [y_array_2], [z_array_2]], ...])
        self.pos_array = pos_array

    def create_polygons(self):
        polygon_array = []

        for i in range(self.n_polygons):
            polygon_array.append(Polygon(self.points_array[i], self.pos_array[i]))

        return polygon_array

    def get_vertices(self, polygon_array):
        polygons_vertices = []

        for polygon in polygon_array:
            polygons_vertices.append(polygon.polygon_vertices())

        return polygons_vertices

class SphereManager:

    def __init__(self, n_spheres, pos_array, r_array):
        self.n_spheres = n_spheres
        self.pos_array = pos_array
        self.r_array   = r_array

    def create_spheres(self):
        sphere_array = []

        for i in range(self.n_spheres):
            sphere_array.append(Sphere(self.r_array[i], self.pos_array[i]))

        return sphere_array
