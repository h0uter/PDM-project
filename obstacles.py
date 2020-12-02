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

class Polygon:

    def __init__(self, points, pos):
        self.xpoints = np.asarray([points[0][0], points[1][0], points[2][0]]) #array with x-coordinates of polygon points
        self.ypoints = np.asarray([points[0][1], points[1][1], points[2][1]]) #array with y-coordinates of polygon points
        self.zpoints = np.asarray([points[0][2], points[1][2], points[2][2]]) #array with z-coordinates of polygon points
        self.points = points # array of polygon points [[xyz_p1_pol1], [xyz_p2_pol1], [xyz_p3_pol1]]
        self.center = [sum(self.xpoints)/len(self.points), sum(self.ypoints)/len(self.points), sum(self.zpoints)/len(self.points)]
        self.pos = np.asarray(pos) #position of polygon

    def create_polygon(self):
        x = (self.xpoints + self.pos[0]).tolist()
        y = (self.ypoints + self.pos[1]).tolist()
        z = (self.zpoints + self.pos[2]).tolist()
        verts = [list(zip(x, y, z))]

        return verts

    def polygon_edges(self):
        edges = []
        for i, points in enumerate(self.points):
            x = self.xpoints[i-1] - self.xpoints[i]
            y = self.ypoints[i-1] - self.ypoints[i]
            z = self.zpoints[i-1] - self.zpoints[i]
            edges.append([x, y ,z])

        return edges

class PolygonManager:

    def __init__(self, n_polygons, points_array, pos_array):
        self.n_polygons = n_polygons
        self.points_array = points_array #array of polygon points ([[[xyz_p1_pol1], [xyz_p2_pol1], [xyz_p3_pol1]], [[xyz_p1_pol2], [xyz_p2_pol2], [xyz_p3_pol2]], ...])
        self.pos_array = pos_array

    def create_polygons(self):
        polygon_array = []

        for i in range(self.n_polygons):
            polygon_array.append(Polygon(self.points_array[i], self.pos_array[i]))

        return polygon_array

    def get_edges(self, polygon_array):
        polygons_edges = []

        for polygon in polygon_array:
            polygons_edges.append(polygon.polygon_edges())

        return polygons_edges
