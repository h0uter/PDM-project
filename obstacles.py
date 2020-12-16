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

    def __init__(self, points):
        self.xpoints = np.asarray([points[0][0], points[1][0], points[2][0]]) #array with x-coordinates of polygon points
        self.ypoints = np.asarray([points[0][1], points[1][1], points[2][1]]) #array with y-coordinates of polygon points
        self.zpoints = np.asarray([points[0][2], points[1][2], points[2][2]]) #array with z-coordinates of polygon points
        self.points = points # array of polygon points [[xyz_p1_pol1], [xyz_p2_pol1], [xyz_p3_pol1]]
        print(self.xpoints)

    def create_polygon(self):
        x = (self.xpoints).tolist()
        y = (self.ypoints).tolist()
        z = (self.zpoints).tolist()
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

    def __init__(self, n_polygons, points_array):
        self.n_polygons = n_polygons
        self.points_array = points_array #array of polygon points ([[[xyz_p1_pol1], [xyz_p2_pol1], [xyz_p3_pol1]], [[xyz_p1_pol2], [xyz_p2_pol2], [xyz_p3_pol2]], ...])

    def create_polygons(self):
        polygon_array = []

        for i in range(self.n_polygons):
            polygon_array.append(Polygon(self.points_array[i]))

        return polygon_array

    def get_edges(self, polygon_array):
        polygons_edges = []

        for polygon in polygon_array:
            polygons_edges.append(polygon.polygon_edges())

        return polygons_edges

class Prism:

    def __init__(self, width, length, height, pos):
        self.h = height
        self.l = length
        self.w = width
        self.center = pos
        self.polygon_array = []
        self.edges = []
        self.set_points()

    def set_points(self):
        point1 = [self.center[0] - 0.5 * self.w,   self.center[1] - 0.5 * self.l,     self.center[2] - 0.5 * self.h]
        point2 = [self.center[0] + 0.5 * self.w,   self.center[1] - 0.5 * self.l,     self.center[2] - 0.5 * self.h]
        point3 = [self.center[0] - 0.5 * self.w,   self.center[1] + 0.5 * self.l,     self.center[2] - 0.5 * self.h]
        point4 = [self.center[0] + 0.5 * self.w,   self.center[1] + 0.5 * self.l,     self.center[2] - 0.5 * self.h]
        point5 = [self.center[0]               ,   self.center[1] - 0.5 * self.l,     self.center[2] + 0.5 * self.h]
        point6 = [self.center[0]               ,   self.center[1] + 0.5 * self.l,     self.center[2] + 0.5 * self.h]

        points = [point1, point2, point3, point4, point5, point6]
        self.set_polygons(points)

    def set_polygons(self, point):

        polygon1 = [point[5], point[2], point[4]]
        polygon2 = [point[4], point[0], point[2]]
        polygon3 = [point[0], point[1], point[2]]
        polygon4 = [point[1], point[2], point[3]]
        polygon5 = [point[1], point[5], point[3]]
        polygon6 = [point[1], point[4], point[5]]
        polygon7 = [point[2], point[3], point[5]]
        polygon8 = [point[0], point[1], point[4]]

        polygon_point_array     = [polygon1, polygon2, polygon3, polygon4, polygon5, polygon6, polygon7, polygon8]
        polygon_manager         = PolygonManager(len(polygon_point_array), polygon_point_array)
        self.polygons_array     = polygon_manager.create_polygons()
        self.edges              = polygon_manager.get_edges(self.polygons_array)

class PrismManager:

    def __init__(self, n_prisms, dimensions_array, pos_array):
        self.n_prisms = n_prisms
        self.pos_array = pos_array
        self.dimensions_array = dimensions_array

    def create_prisms(self):
        prism_array = []

        for i in range(self.n_prisms):
            prism_array.append(Prism(self.dimensions_array[i][0], self.dimensions_array[i][1], self.dimensions_array[i][2], self.pos_array[i]))

        return prism_array

class Beam:

    def __init__(self, height, length, width, pos):
        self.h = height
        self.l = length
        self.w = width
        self.center = pos
        self.polygon_array = []
        self.set_points()

    def set_points(self):
        point1 = [self.center[0] - 0.5 * self.w,   self.center[1] - 0.5 * self.l,     self.center[2] - 0.5 * self.h]
        point2 = [self.center[0] + 0.5 * self.w,   self.center[1] - 0.5 * self.l,     self.center[2] - 0.5 * self.h]
        point3 = [self.center[0] - 0.5 * self.w,   self.center[1] + 0.5 * self.l,     self.center[2] - 0.5 * self.h]
        point4 = [self.center[0] + 0.5 * self.w,   self.center[1] + 0.5 * self.l,     self.center[2] - 0.5 * self.h]
        point5 = [self.center[0] - 0.5 * self.w,   self.center[1] - 0.5 * self.l,     self.center[2] + 0.5 * self.h]
        point6 = [self.center[0] + 0.5 * self.w,   self.center[1] - 0.5 * self.l,     self.center[2] + 0.5 * self.h]
        point7 = [self.center[0] - 0.5 * self.w,   self.center[1] + 0.5 * self.l,     self.center[2] + 0.5 * self.h]
        point8 = [self.center[0] + 0.5 * self.w,   self.center[1] + 0.5 * self.l,     self.center[2] + 0.5 * self.h]

        points = [point1, point2, point3, point4, point5, point6, point7, point8]
        self.set_polygons(points)

    def set_polygons(self, point):

        polygon1    = [point[5], point[6], point[7]]
        polygon2    = [point[5], point[4], point[6]]
        polygon3    = [point[4], point[0], point[6]]
        polygon4    = [point[0], point[6], point[2]]
        polygon5    = [point[0], point[3], point[2]]
        polygon6    = [point[0], point[1], point[3]]
        polygon7    = [point[1], point[5], point[3]]
        polygon8    = [point[5], point[7], point[3]]
        polygon9    = [point[2], point[3], point[6]]
        polygon10   = [point[3], point[7], point[6]]
        polygon11   = [point[0], point[4], point[5]]
        polygon12   = [point[4], point[5], point[1]]

        polygon_point_array     = [polygon1, polygon2, polygon3, polygon4, polygon5, polygon6, polygon7, polygon8, polygon9, polygon10, polygon11, polygon12]
        polygon_manager         = PolygonManager(len(polygon_point_array), polygon_point_array)
        self.polygons_array     = polygon_manager.create_polygons()
        self.edges              = polygon_manager.get_edges(self.polygons_array)

class BeamManager:

    def __init__(self, n_beams, dimensions_array, pos_array):
        self.n_beams = n_beams
        self.pos_array = pos_array
        self.dimensions_array = dimensions_array

    def create_beams(self):
        beams_array = []

        for i in range(self.n_beams):
            beams_array.append(Beam(self.dimensions_array[i][0], self.dimensions_array[i][1], self.dimensions_array[i][2], self.pos_array[i]))

        return beams_array
